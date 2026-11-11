#include "rgb_strip.h"
#include <esp_rom_sys.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>

#include "settings.h"

#define TAG "RgbStrip"

// 效果代号：指向 RgbStrip::effect_gen_，由 StartEffect 设置。
extern volatile uint32_t* g_effect_gen;
extern uint32_t g_effect_my_gen;


// NVS key 上限 15 字符，下面这些都在范围内
static const char* kSlotKeys[RgbStrip::kSlotCount] = {
    "startup_init",   // startup_initial
    "startup_idle",   // startup_idle
    "lightshow",      // lightshow
    "listening",      // listening
    "speaking",       // speaking
};

static const char* kSlotDefaults[RgbStrip::kSlotCount] = {
    "7,50,W,0,80,80",     // 上电：青色常亮（亮度收到 80，满亮 255 太刺眼）
    "0,0,0",              // 空闲：灭
    "7,50,R",             // 彩灯秀：彩虹
    "7,90,K,255,0,0",     // 倾听：红色呼吸
    "7,90,P,0,128,255",   // 说话：蓝色脉冲
};

const char* RgbStrip::SlotName(Slot slot) {
    switch (slot) {
        case kSlotStartupInitial: return "startup_initial";
        case kSlotStartupIdle:    return "startup_idle";
        case kSlotLightshow:      return "lightshow";
        case kSlotListening:      return "listening";
        case kSlotSpeaking:       return "speaking";
        default:                  return "unknown";
    }
}

RgbStrip::RgbStrip(gpio_num_t gpio, uint16_t led_count)
    : gpio_(gpio), led_count_(led_count) {
    if (led_count_ == 0) {
        led_count_ = 1;
    }

    mutex_ = xSemaphoreCreateMutex();

    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = gpio_;
    strip_config.max_leds = led_count_;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000;  // 10MHz

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &strip_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip on GPIO %d: %s", (int)gpio_, esp_err_to_name(err));
        strip_ = nullptr;
        return;
    }

    led_strip_clear(strip_);
    ESP_LOGI(TAG, "LED strip initialized on GPIO %d with %u LEDs", (int)gpio_, (unsigned)led_count_);
}

RgbStrip::~RgbStrip() {
    // 先把效果任务停掉、等它真的退出，再删底层句柄。
    // 顺序反了的话任务会往已释放的 led_strip 里写，直接 panic。
    ++effect_gen_;   // 代号一变，任务下一轮就退出
    if (effect_task_ != nullptr) {
        // 按 tick 计时：100Hz 下 pdMS_TO_TICKS(5) = 0，vTaskDelay(0) 不会让出 CPU。
        TickType_t start = xTaskGetTickCount();
        bool exited = false;
        while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(800)) {
            if (effect_task_exited_) { exited = true; break; }
            vTaskDelay(1);
        }
        if (!exited) {
            ESP_LOGW(TAG, "effect task did not exit in time, force deleting");
            vTaskDelete(effect_task_);
        }
        effect_task_ = nullptr;
    }

    if (strip_ != nullptr) {
        led_strip_clear(strip_);
        led_strip_del(strip_);
        strip_ = nullptr;
    }

    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

// ---------- 全局亮度 ----------
// 默认 40%：彩虹、呼吸这些满亮度效果在暗环境里 255 太刺眼，
// 而且整条灯带满亮功耗能到几百毫安。
static volatile int g_brightness_pct = 100;   // 100 = 只走硬上限 100/255

void RgbStrip::SetBrightness(int percent) {
    if (percent < 1) percent = 1;
    if (percent > 100) percent = 100;
    g_brightness_pct = percent;

    // 关键：只改变量不重绘，灯是不会有任何变化的。
    // 拿最后一条生效的 gcode 重放一次，用户才能立刻看到效果。
    if (!last_gcode_.empty()) {
        std::string err;
        ApplyGcode(last_gcode_, &err);
    }
}

int RgbStrip::GetBrightness() {
    return g_brightness_pct;
}

// 所有真正写进灯带缓冲色的地方都要过这一层。
//
// 两层限制，缺一不可：
//   1) 硬上限 kChannelMax = 100（0-255 量程）。任何效果算出 255 也只输出 100，
//      这是"限制死"的那一条，跟用户调不调亮度无关。
//   2) 全局亮度百分比，在硬上限之内再缩。默认 100，也就是只吃硬上限。
// 顺序不能反：先按百分比缩再截顶，跟先截顶再缩，结果在低百分比下差很多。
static constexpr int kChannelMax = 100;

static inline StripColor Dim(StripColor c) {
    const int b = g_brightness_pct;
    int r = (int)c.r * b / 100;
    int g = (int)c.g * b / 100;
    int bl = (int)c.b * b / 100;
    if (r > kChannelMax) r = kChannelMax;
    if (g > kChannelMax) g = kChannelMax;
    if (bl > kChannelMax) bl = kChannelMax;
    return StripColor{(uint8_t)r, (uint8_t)g, (uint8_t)bl};
}

void RgbStrip::FillBuffer(StripColor color) {
    if (strip_ == nullptr) {
        return;
    }
    const StripColor c = Dim(color);
    for (uint16_t i = 0; i < led_count_; ++i) {
        led_strip_set_pixel(strip_, i, c.r, c.g, c.b);
    }
    led_strip_refresh(strip_);
}

void RgbStrip::SetAll(StripColor color) {
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "strip busy, skip this frame");
        return;
    }
    // 记下这条颜色，供改亮度时重绘。set_color 不走 ApplyGcode，
    // 这里补一条等价 gcode，否则调完亮度灯还停在旧值上。
    char buf[48];
    snprintf(buf, sizeof(buf), "%u,0,W,%u,%u,%u",
             (unsigned)led_count_, (unsigned)color.r, (unsigned)color.g, (unsigned)color.b);
    last_gcode_ = buf;

    FillBuffer(color);
    xSemaphoreGive(mutex_);
}

void RgbStrip::SetOne(int index, StripColor color) {
    if (index < 0 || index >= (int)led_count_) {
        return;
    }
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "strip busy, skip this frame");
        return;
    }
    if (strip_ == nullptr) {
        xSemaphoreGive(mutex_);
        return;
    }
    // 只改缓冲区，不在这里 refresh。
    // 之前每颗灯都刷一次，7 颗就是 7 次 RMT 传输，而且中间状态会被看见——
    // 第一颗已是新色、其余还是旧色，肉眼看就是爆闪。
    // 调用方循环结束后统一调 Refresh() 一次。
    const StripColor c = Dim(color);
    led_strip_set_pixel(strip_, index, c.r, c.g, c.b);
    xSemaphoreGive(mutex_);
}

void RgbStrip::Refresh() {
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "strip busy, skip this frame");
        return;
    }
    if (strip_ == nullptr) {
        xSemaphoreGive(mutex_);
        return;
    }
    led_strip_refresh(strip_);
    xSemaphoreGive(mutex_);
}

void RgbStrip::TurnOff() {
    if (strip_ == nullptr) {
        return;
    }
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "strip busy, skip this frame");
        return;
    }
    // 连发两次清零：WS2812 的复位间隔是 >50µs 低电平，如果上一条数据传输
    // 刚结束、复位窗口还没到，紧跟的这一条会被前面的灯当成自己数据的延续，
    // 结果就是"关了 9 颗、后面两颗反而亮起来"。发两次可以确保第二次一定
    // 落在干净的复位之后。
    led_strip_clear(strip_);
    esp_rom_delay_us(100);
    led_strip_clear(strip_);
    // 关灯后把"当前画面"记成全灭，避免下次调亮度又把灯点回来
    last_gcode_ = "0,0,0";
    xSemaphoreGive(mutex_);
}

void RgbStrip::StopEffect() {
    // 代号一变，所有在跑的效果任务下一轮就会看到不匹配并退出。
    ++effect_gen_;
    if (effect_task_ != nullptr) {
        TickType_t start = xTaskGetTickCount();
        bool exited = false;
        while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(800)) {
            if (effect_task_exited_) { exited = true; break; }
            vTaskDelay(1);
        }
        if (!exited) {
            ESP_LOGW(TAG, "effect task did not exit in time, force deleting");
            vTaskDelete(effect_task_);
            effect_task_exited_ = true;
        }
        effect_task_ = nullptr;
    }
}

void RgbStrip::StartEffect(void (*fn)(RgbStrip*), const char* name) {
    // StopEffect 会 ++effect_gen_ 并等旧任务真正退出，
    // 所以走到这里时保证没有别的效果在跑。
    StopEffect();

    // 取一个新的代号给本次任务。效果函数里拿 g_effect_my_gen 和
    // *g_effect_gen 比对，不等就退出——不会出现两个效果同时刷灯。
    g_effect_gen = &effect_gen_;
    g_effect_my_gen = ++effect_gen_;
    effect_task_exited_ = false;

    struct Args {
        RgbStrip* self;
        void (*fn)(RgbStrip*);
    };
    auto* args = new Args{this, fn};

    BaseType_t ok = xTaskCreate(
        [](void* p) {
            auto* a = static_cast<Args*>(p);
            a->fn(a->self);
            a->self->effect_task_exited_ = true;
            delete a;
            vTaskDelete(nullptr);
        },
        name, 4096, args, tskIDLE_PRIORITY + 1, &effect_task_);

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create effect task %s", name);
        delete args;
        effect_task_exited_ = true;
        effect_task_ = nullptr;
    } else {
        ESP_LOGI(TAG, "Effect task %s started (gen=%lu)", name, (unsigned long)g_effect_my_gen);
    }
}

// ---------------------------------------------------------------- 效果实现

static inline StripColor ScaleColor(StripColor c, float k) {
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    return StripColor{(uint8_t)(c.r * k), (uint8_t)(c.g * k), (uint8_t)(c.b * k)};
}

// HSV -> RGB，用于彩虹
static StripColor HsvToRgb(float h, float s, float v) {
    float c = v * s;
    float x = c * (1 - fabsf(fmodf(h / 60.0f, 2.0f) - 1));
    float m = v - c;
    float r = 0, g = 0, b = 0;
    if (h < 60)       { r = c; g = x; b = 0; }
    else if (h < 120) { r = x; g = c; b = 0; }
    else if (h < 180) { r = 0; g = c; b = x; }
    else if (h < 240) { r = 0; g = x; b = c; }
    else if (h < 300) { r = x; g = 0; b = c; }
    else              { r = c; g = 0; b = x; }
    return StripColor{(uint8_t)((r + m) * 255), (uint8_t)((g + m) * 255), (uint8_t)((b + m) * 255)};
}

// ============================================================================
// 四个效果任务：相位一律按 esp_timer_get_time() 的真实时间算，不按帧序号。
// TIME_BASED_EFFECTS
//
// 原因：vTaskDelay(pdMS_TO_TICKS(n)) 在 FreeRTOS 里只保证「至少 n 毫秒」，
// tick 是 10ms 一格，加上任务被抢占，实际间隔会在 n~n+10ms 之间跳。
// 如果按帧序号推相位（i/steps），这个抖动会原样体现成波形变形：
// 呼吸一顿一顿、速度忽快忽慢、误差还随帧数累积。
// 改成 phase = ((now - t0) / period) 之后，帧率抖动只影响采样密度，
// 不影响波形本身。
// ============================================================================

static inline int64_t NowMs() {
    return esp_timer_get_time() / 1000;
}

// 闪烁：亮-灭交替 times 次
static void BlinkTask(RgbStrip* self) {
    extern StripColor g_blink_color;
    extern int g_blink_times;
    extern int g_blink_interval;


    const int64_t half = (g_blink_interval > 0 ? g_blink_interval : 400) / 2;
    const int total_half = (g_blink_times > 0 ? g_blink_times : 3) * 2;

    self->SetAll(g_blink_color);          // 第 0 个半周期：亮
    int shown_half = 0;
    int64_t t0 = NowMs();

    for (;;) {
        if (g_effect_gen == nullptr || *g_effect_gen != g_effect_my_gen) break;
        int64_t idx = (NowMs() - t0) / half;
        if (idx > shown_half) {
            shown_half = (int)idx;
            if (shown_half % 2 == 0) {
                self->SetAll(g_blink_color);
            } else {
                self->TurnOff();
            }
        }
        if (shown_half >= total_half) break;
        // 5ms 在 100Hz tick 下是 0，等于不停转 —— 闪烁会变成几十赫兹的爆闪。
        // 一个 tick 是 10ms，够用了。
        vTaskDelay(1);
    }
    self->TurnOff();
}

// 彩虹：色相沿灯带滚动，frame 由真实时间换算
static void RainbowTask(RgbStrip* self) {
    extern int g_rainbow_speed;
    extern int g_rainbow_times;


    // gcode 第二段是"每步毫秒数"，每步走 6 度色相 → 60 步转完一圈。
    // 之前是一步一度、360 步才一圈，`7,50,R` 要 18 秒，看起来就像静止不动。
    const float step_ms = (float)(g_rainbow_speed > 0 ? g_rainbow_speed : 50);
    // cycles <= 0 表示一直转，不设上限。之前默认 1 圈，3 秒后 break 停在最后一帧，
    // 看起来就是"动一下就不动了"。
    const int cycles = g_rainbow_times;
    const bool infinite = (cycles <= 0);
    const float deg_per_step = 6.0f;
    const int64_t total_ms = infinite ? 0
                                      : (int64_t)(step_ms * (360.0f / deg_per_step)) * (int64_t)cycles;
    const int leds = self->LedCount();

    ESP_LOGI("RgbStrip", "Rainbow: %s x %.0f frames, %.0f ms/step, %d leds",
             infinite ? "infinite" : "cycles", 360.0f / deg_per_step, step_ms, leds);

    int64_t t0 = NowMs();
    for (;;) {
        if (g_effect_gen == nullptr || *g_effect_gen != g_effect_my_gen) break;
        int64_t elapsed = NowMs() - t0;
        if (!infinite && elapsed >= total_ms) break;

        float frame = fmodf((float)elapsed / step_ms * deg_per_step, 360.0f);
        for (int i = 0; i < leds; ++i) {
            float h = fmodf(frame + (float)i * (360.0f / (float)leds), 360.0f);
            self->SetOne(i, HsvToRgb(h, 1.0f, 1.0f));
        }
        self->Refresh();          // 整帧写完再刷一次，避免中间态被看见
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 呼吸：亮度按余弦升降，相位由真实时间算
static void BreatheTask(RgbStrip* self) {
    extern StripColor g_breathe_color;
    extern int g_breathe_interval;


    // gcode 第二段是"每步毫秒数"，一圈按 30 步算。
    // 之前直接把它当整圈周期，`7,90,K` 就变成 90ms 闪一次，肉眼就是爆闪。
    int64_t period = (int64_t)(g_breathe_interval > 0 ? g_breathe_interval : 60) * 30;
    if (period < 800) period = 800;

    int64_t t0 = NowMs();
    for (;;) {
        if (g_effect_gen == nullptr || *g_effect_gen != g_effect_my_gen) return;
        float phase = (float)((NowMs() - t0) % period) / (float)period;
        float k = 0.5f - 0.5f * cosf(2.0f * 3.1415926f * phase);
        self->SetAll(ScaleColor(g_breathe_color, k));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 脉冲：前 20% 快速亮起，后 80% 线性消退（STC8 的 Pulse 形状）
static void PulseTask(RgbStrip* self) {
    extern StripColor g_pulse_color;
    extern int g_pulse_interval;


    // 和呼吸同因：gcode 第二段是"每步毫秒数"，不是整圈周期。
    // 之前 period 直接取 90，10ms 采样只有 9 个点，每秒 11 次明暗 = 爆闪。
    int64_t period = (int64_t)(g_pulse_interval > 0 ? g_pulse_interval : 30) * 30;
    if (period < 800) period = 800;
    const int64_t attack = period / 5;          // 前 20%
    const int64_t decay = period - attack;

    int64_t t0 = NowMs();
    for (;;) {
        if (g_effect_gen == nullptr || *g_effect_gen != g_effect_my_gen) return;
        int64_t elapsed = (NowMs() - t0) % period;
        float k;
        if (elapsed < attack) {
            k = (float)elapsed / (float)attack;
        } else {
            k = 1.0f - (float)(elapsed - attack) / (float)decay;
        }
        self->SetAll(ScaleColor(g_pulse_color, k));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------------------------------------------------------- gcode 解析

namespace {

struct GcodeCmd {
    int count = 0;
    int speed_ms = 50;
    char effect = 'N';
    StripColor color;
    bool has_color = false;
};

bool ParseGcode(const std::string& gcode, GcodeCmd* out, std::string* error) {
    std::vector<std::string> parts;
    std::string cur;
    for (char ch : gcode) {
        if (ch == ',') {
            parts.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(ch);
        }
    }
    parts.push_back(cur);

    // 去掉每段首尾空白
    for (auto& p : parts) {
        size_t b = p.find_first_not_of(" \t\r\n");
        size_t e = p.find_last_not_of(" \t\r\n");
        if (b == std::string::npos) {
            p.clear();
        } else {
            p = p.substr(b, e - b + 1);
        }
    }

    if (parts.size() < 3) {
        if (error) *error = "格式应为 <数量>,<速度ms>,<特效>[,R,G,B]";
        return false;
    }

    char* endp = nullptr;
    long count = strtol(parts[0].c_str(), &endp, 10);
    if (endp == parts[0].c_str() || count < 0 || count > 100) {
        if (error) *error = "数量必须是 0-100 的整数";
        return false;
    }
    out->count = (int)count;

    endp = nullptr;
    long speed = strtol(parts[1].c_str(), &endp, 10);
    if (endp == parts[1].c_str() || speed < 0 || speed > 10000) {
        if (error) *error = "速度必须是 0-10000 的毫秒数";
        return false;
    }
    out->speed_ms = (int)speed;

    if (parts[2].empty()) {
        if (error) *error = "缺少特效代号";
        return false;
    }
    out->effect = (char)toupper((unsigned char)parts[2][0]);

    // v206 用 "0,0,0" 表示全灭——第三段是数字不是特效代号，
    // 这里特判成 N，否则会被当成"未知特效"拒掉。
    if (out->count == 0 && out->effect >= '0' && out->effect <= '9') {
        out->effect = 'N';
    }

    switch (out->effect) {
        case 'W':
        case 'K':
        case 'P':
            // 带 RGB 三元组
            if (parts.size() >= 6) {
                int rgb[3];
                for (int i = 0; i < 3; ++i) {
                    endp = nullptr;
                    long v = strtol(parts[3 + i].c_str(), &endp, 10);
                    if (endp == parts[3 + i].c_str() || v < 0 || v > 255) {
                        if (error) *error = "RGB 分量必须是 0-255 的整数";
                        return false;
                    }
                    rgb[i] = (int)v;
                }
                out->color = StripColor{(uint8_t)rgb[0], (uint8_t)rgb[1], (uint8_t)rgb[2]};
                out->has_color = true;
            } else if (out->effect != 'P') {
                // W 和 K 必须给颜色，P 可以不给（用当前色）
                if (error) *error = "W/K 特效需要跟 R,G,B 三个分量";
                return false;
            }
            break;
        case 'R':
            // 彩虹，不带 RGB
            break;
        case 'N':
            // 灭灯
            out->count = 0;
            break;
        default:
            if (error) {
                *error = "未知特效代号（支持 W/K/P/R/N）";
            }
            return false;
    }

    return true;
}

}  // namespace

// 任务间传参（单实例灯带，用全局变量最简单；效果任务同一时刻只有一个）
StripColor g_blink_color;
int g_blink_times = 0;
int g_blink_interval = 500;


int g_rainbow_speed = 50;
int g_rainbow_times = 0;   // 0 = 无限循环


StripColor g_breathe_color;
int g_breathe_interval = 900;


StripColor g_pulse_color;
int g_pulse_interval = 900;


bool RgbStrip::ValidateGcode(const std::string& gcode, std::string* error) {
    GcodeCmd cmd;
    return ParseGcode(gcode, &cmd, error);
}

bool RgbStrip::ApplyGcode(const std::string& gcode, std::string* error) {
    GcodeCmd cmd;
    if (!ParseGcode(gcode, &cmd, error)) {
        return false;
    }

    StopEffect();
    vTaskDelay(pdMS_TO_TICKS(20));

    switch (cmd.effect) {
        case 'N':
            TurnOff();
            break;

        case 'W':
            SetAll(cmd.color);
            break;

        case 'R':
            g_rainbow_speed = cmd.speed_ms;
            g_rainbow_times = 0;   // 0 = 无限循环，不会转一圈停在最后一帧
            StartEffect(RainbowTask, "strip_rainbow");
            break;

        case 'K':
            g_breathe_color = cmd.color;
            g_breathe_interval = cmd.speed_ms > 0 ? cmd.speed_ms : 900;
            StartEffect(BreatheTask, "strip_breathe");
            break;

        case 'P':
            g_pulse_color = cmd.has_color ? cmd.color : StripColor{0, 128, 255};
            g_pulse_interval = cmd.speed_ms > 0 ? cmd.speed_ms : 900;
            StartEffect(PulseTask, "strip_pulse");
            break;

        default:
            break;
    }

    last_gcode_ = gcode;
    return true;
}

void RgbStrip::Blink(StripColor color, int times, int interval_ms) {
    g_blink_color = color;
    g_blink_times = times > 0 ? times : 3;
    g_blink_interval = interval_ms > 0 ? interval_ms : 500;
    StartEffect(BlinkTask, "strip_blink");
}

void RgbStrip::Rainbow(int speed_ms, int times) {
    g_rainbow_speed = speed_ms > 0 ? speed_ms : 50;
    g_rainbow_times = times;   // <=0 表示无限循环
    StartEffect(RainbowTask, "strip_rainbow");
}

// 效果代号指针：指向 RgbStrip::effect_gen_
volatile uint32_t* g_effect_gen = nullptr;
uint32_t g_effect_my_gen = 0;

// ---------------------------------------------------------------- NVS

void RgbStrip::LoadFromNvs() {
    Settings settings("led_config", true);

    // 亮度独立于槽位版本，不做 cfg_ver 重置，用户调过就一直保留
    int b = settings.GetInt("bright_lvl", 100);   // 换过键名，旧值不再生效
    if (b < 1) b = 1;
    if (b > 100) b = 100;
    g_brightness_pct = b;
    ESP_LOGI(TAG, "Brightness = %d%%", b);

    // 槽位结构变了就整体重置一次：否则改了默认值，老 NVS 里的旧值会一直盖着，
    // 用户得手动清 flash 才看得到新默认。
    constexpr int kCurrentVersion = 6;   // 6: 灯珠数改回 7（电流限制）
    int stored_version = settings.GetInt("cfg_ver", 0);
    bool reset_all = (stored_version != kCurrentVersion);

    for (int i = 0; i < kSlotCount; ++i) {
        std::string def = kSlotDefaults[i];
        slot_gcode_[i] = reset_all ? def : settings.GetString(kSlotKeys[i], def);
        // 读出来的内容先校验一遍，坏数据直接回退到默认值，避免开机就报错
        std::string err;
        if (!ValidateGcode(slot_gcode_[i], &err)) {
            ESP_LOGW(TAG, "Slot %s has invalid gcode '%s' (%s), falling back to default",
                     SlotName((Slot)i), slot_gcode_[i].c_str(), err.c_str());
            slot_gcode_[i] = def;
        }
        ESP_LOGI(TAG, "Slot %s = %s", SlotName((Slot)i), slot_gcode_[i].c_str());
    }

    if (reset_all) {
        ESP_LOGI(TAG, "LED config version %d -> %d, slots reset to defaults",
                 stored_version, kCurrentVersion);
        SaveToNvs();
        settings.SetInt("cfg_ver", kCurrentVersion);
    }
}

void RgbStrip::SaveToNvs() {
    Settings settings("led_config", true);
    settings.SetInt("bright_lvl", g_brightness_pct);
    for (int i = 0; i < kSlotCount; ++i) {
        settings.SetString(kSlotKeys[i], slot_gcode_[i]);
    }
    ESP_LOGI(TAG, "LED slots saved to NVS");
}

void RgbStrip::SetSlotGcode(Slot slot, const std::string& gcode) {
    if (slot < 0 || slot >= kSlotCount) {
        return;
    }
    slot_gcode_[slot] = gcode;
}

std::string RgbStrip::GetSlotGcode(Slot slot) const {
    if (slot < 0 || slot >= kSlotCount) {
        return {};
    }
    return slot_gcode_[slot];
}

void RgbStrip::ApplySlot(Slot slot) {
    if (slot < 0 || slot >= kSlotCount) {
        return;
    }
    std::string err;
    if (!ApplyGcode(slot_gcode_[slot], &err)) {
        ESP_LOGW(TAG, "Failed to apply slot %s: %s", SlotName(slot), err.c_str());
    }
}
