#include "rgb_strip.h"

#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "settings.h"

#define TAG "RgbStrip"

// NVS key 上限 15 字符，下面这些都在范围内
static const char* kSlotKeys[RgbStrip::kSlotCount] = {
    "startup_init",   // startup_initial
    "startup_idle",   // startup_idle
    "lightshow",      // lightshow
    "listening",      // listening
    "speaking",       // speaking
};

// 默认值按官方 v2.0 协议写：第三段给数字表示常亮，字母才是效果。
// 长度超过 7 的部分会被忽略（实际灯珠数由构造参数决定）。
static const char* kSlotDefaults[RgbStrip::kSlotCount] = {
    "7,50,0,80,80",       // 上电：青色常亮（80 是压暗过的青，满 255 刺眼）
    "0,0,0",              // 空闲：灭
    "7,50,R",             // 彩灯秀：彩虹
    "0,0,0",              // 倾听：灭（说话的人不是它，亮着只是干扰）
    "7,200,B,255,255,255" // 说话：语音律动，跟着播放音量从中间往外扩散
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

uint32_t RgbStrip::NowMs() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

RgbStrip::RgbStrip(gpio_num_t gpio, uint16_t led_count, uint16_t strip_length)
    : gpio_(gpio), led_count_(led_count), strip_length_(strip_length) {
    if (led_count_ == 0) {
        led_count_ = 1;
    }
    if (led_count_ > kMaxLeds) {
        led_count_ = kMaxLeds;   // 效果颗数上限
    }
    if (strip_length_ < led_count_) {
        strip_length_ = led_count_;
    }
    if (strip_length_ > kMaxStripLength) {
        strip_length_ = kMaxStripLength;
    }

    mutex_ = xSemaphoreCreateMutex();

    // 先明确置成"灭"。StripParams 值初始化后 color_count 也是 0，
    // FirstOrDefault 会兜底成白色 —— 上电到 NVS 读完的这几十毫秒，
    // 整条灯会白闪一下，夜里很刺眼。
    std::memset(&params_, 0, sizeof(params_));
    params_.effect = kFxSolid;
    params_.speed = 128;
    params_.intensity = 128;
    params_.color_count = 1;
    params_.colors[0] = StripColor{0, 0, 0};

    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = gpio_;
    strip_config.max_leds = strip_length_;
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

uint16_t RgbStrip::LoadLedCount(uint16_t fallback) {
    if (fallback == 0) {
        fallback = 1;
    }
    if (fallback > kMaxLeds) {
        fallback = kMaxLeds;
    }
    Settings settings("led_config", true);
    int n = (int)settings.GetInt("led_count", (int)fallback);
    if (n <= 0) {
        n = (int)fallback;
    }
    if (n > (int)kMaxLeds) {
        n = (int)kMaxLeds;
    }
    return (uint16_t)n;
}

uint16_t RgbStrip::SetLedCount(uint16_t count) {
    if (count == 0) {
        count = 1;
    }
    if (count > kMaxLeds) {
        count = kMaxLeds;
    }
    if (count == led_count_) {
        return led_count_;
    }

    // 不重建 RMT：物理长度是固定的，RMT 一直按 strip_length_ 发，
    // 只是前 count 颗跑效果、后面每帧写黑。所以这里只改数字。
    // 先把整条清黑，免得缩小颗数后原来亮着的那几颗停在上一次的颜色。
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(300)) == pdTRUE) {
        led_strip_clear(strip_);   // clear 会写满 max_leds，后面自然全黑
        led_strip_refresh(strip_);
        xSemaphoreGive(mutex_);
    }

    led_count_ = count;
    blank_tail_needed_ = true;
    {
        Settings settings("led_config", true);
        settings.SetInt("led_count", (int)count);
    }
    // 重启效果任务，让相位和画面从新颗数重新算。
    StartEffectLoop();
    ESP_LOGI(TAG, "LED count changed to %u (strip %u)", (unsigned)count, (unsigned)strip_length_);
    return led_count_;
}

RgbStrip::~RgbStrip() {
    // 先把效果任务停掉、等它真的退出，再删底层句柄。
    // 顺序反了的话任务会往已释放的 led_strip 里写，直接 panic。
    StopEffect();
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
// 100 = 只走硬上限 100/255；再往下是按百分比缩。
static volatile int g_brightness_pct = 100;

void RgbStrip::SetBrightness(int percent) {
    if (percent < 1) percent = 1;
    if (percent > 100) percent = 100;
    g_brightness_pct = percent;
    // 不在这里重绘：效果任务是 50fps 的，下一帧（≤20ms）就会用新亮度出图。
    // 早先是"重放最后一条 gcode"，但那条路径会重启效果任务，相位从 0 开始，
    // 呼吸正好在谷底时调亮度会看到画面跳一下。
}

int RgbStrip::GetBrightness() {
    return g_brightness_pct;
}

void RgbStrip::PersistBrightness() {
    Settings settings("led_config", true);
    settings.SetInt("bright_lvl", g_brightness_pct);
}

// 所有真正写进灯带缓冲色的地方都要过这一层。
//
// 两层限制，缺一不可：
//   1) 硬上限 kChannelMax = 100（0-255 量程）。任何效果算出 255 也只输出 100，
//      这是"限制死"的那一条，跟用户调不调亮度无关。
//   2) 全局亮度百分比，在硬上限之内再缩。
// 顺序不能反：先按百分比缩再截顶，跟先截顶再缩，结果在低百分比下差很多。
static constexpr int kChannelMax = 100;

static inline StripColor Dim(StripColor c) {
    const int b = g_brightness_pct;
    // 先把 0..255 的量程整体压到 0..kChannelMax，再乘亮度百分比。
    // 早先是"先乘百分比再截顶"，结果百分比一过 39% 三个通道全撞上限，
    // 滑杆后半程完全没有区分度（55% 和 100% 输出一模一样）。
    // 现在 100% 正好等于 kChannelMax，全程单调，上限也没松。
    const int num = kChannelMax * b;
    int r = (int)c.r * num / (255 * 100);
    int g = (int)c.g * num / (255 * 100);
    int bl = (int)c.b * num / (255 * 100);
    return StripColor{(uint8_t)r, (uint8_t)g, (uint8_t)bl};
}

void RgbStrip::PaintFrame(const StripColor* px, uint16_t count) {
    if (strip_ == nullptr || px == nullptr) {
        return;
    }
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        // 拿不到锁就丢这一帧。等下去会把效果任务卡在锁上，画面反而更顿；
        // 下一帧 20ms 后就来，肉眼看不出来。
        return;
    }
    if (count > led_count_) {
        count = led_count_;
    }
    for (uint16_t i = 0; i < count; ++i) {
        const StripColor c = Dim(px[i]);
        led_strip_set_pixel(strip_, i, c.r, c.g, c.b);
    }
    // 尾段补黑：灯带物理长度通常大于"用几颗"，后面这些灯珠收不到新数据
    // 就会一直保持上一次锁存的颜色（上电噪声常常是亮白）。
    // 但它们只在数据流经时才会变，稳态不需要每帧重写 —— 只在置位那一帧补一次，
    // 省掉每帧一百多次 set_pixel 调用。
    if (blank_tail_needed_) {
        for (uint16_t i = led_count_; i < strip_length_; ++i) {
            led_strip_set_pixel(strip_, i, 0, 0, 0);
        }
        blank_tail_needed_ = false;
    }
    led_strip_refresh(strip_);
    xSemaphoreGive(mutex_);
}

void RgbStrip::FillBuffer(StripColor color) {
    if (strip_ == nullptr) {
        return;
    }
    const StripColor c = Dim(color);
    for (uint16_t i = 0; i < led_count_; ++i) {
        led_strip_set_pixel(strip_, i, c.r, c.g, c.b);
    }
    for (uint16_t i = led_count_; i < strip_length_; ++i) {
        led_strip_set_pixel(strip_, i, 0, 0, 0);
    }
    led_strip_refresh(strip_);
}

void RgbStrip::SetAll(StripColor color) {
    StripParams p;
    p.effect = kFxSolid;
    p.colors[0] = color;
    p.color_count = 1;
    ApplyParams(p);
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
    // 只改缓冲区，不在这里 refresh。调用方循环结束后统一调 Refresh() 一次。
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
    StopEffect();
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    // 连发两次清零：WS2812 的复位间隔是 >50µs 低电平，如果上一条数据传输
    // 刚结束、复位窗口还没到，紧跟的这一条会被前面的灯当成自己数据的延续，
    // 结果就是"关了几颗、后面两颗反而亮起来"。
    led_strip_clear(strip_);
    esp_rom_delay_us(100);
    led_strip_clear(strip_);
    last_gcode_ = "0,0,0";
    xSemaphoreGive(mutex_);
}

// ---------------------------------------------------------------- 效果调度

void RgbStrip::StopEffect() {
    // 代号一变，在跑的任务下一轮就会看到不匹配并退出。
    effect_gen_++;
    if (effect_task_ != nullptr) {
        // 按 tick 计时：100Hz 下 pdMS_TO_TICKS(5) = 0，vTaskDelay(0) 不让出 CPU。
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

void RgbStrip::EffectLoop() {
    // 帧缓冲放栈上：64 颗 × 3 字节 = 192 字节，比堆分配省事，
    // 也不会在 PSRAM 里做每帧写（PSRAM 写延迟是内部 RAM 的好几倍）。
    StripColor buf[kMaxLeds];
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(kFrameMs);

    while (true) {
        if (effect_gen_ != effect_my_gen_) {
            break;
        }
        // 每帧重读：Web 上改灯珠数量后不用重启任务也能跟上。
        const uint16_t count = led_count_ > kMaxLeds ? kMaxLeds : led_count_;
        StripParams p;
        taskENTER_CRITICAL(&params_mux_);
        p = params_;
        taskEXIT_CRITICAL(&params_mux_);

        const uint8_t fx = p.effect < kFxCount ? p.effect : (uint8_t)kFxSolid;
        const uint32_t t = NowMs() - effect_start_ms_;
        kEffects[fx].render(p, t, count, buf);
        PaintFrame(buf, count);

        // vTaskDelayUntil 而不是 vTaskDelay：后者是"跑完再等 20ms"，
        // 渲染和刷灯的耗时会被算进去，帧率掉到 45fps 上下还不稳，
        // 快速流动的效果看起来就发涩。
        vTaskDelayUntil(&last, period);
    }

    effect_task_exited_ = true;
    vTaskDelete(nullptr);
}

void RgbStrip::StartEffectLoop() {
    StopEffect();

    taskENTER_CRITICAL(&params_mux_);
    effect_my_gen_ = ++effect_gen_;
    taskEXIT_CRITICAL(&params_mux_);

    effect_start_ms_ = NowMs();
    effect_task_exited_ = false;

    // 优先级 idle+5：比 idle 高，保证 20ms 一到就轮到；
    // 又低于 WiFi 和音频任务，不会去抢它们的时间片。
    BaseType_t ok = xTaskCreate(
        [](void* self) { static_cast<RgbStrip*>(self)->EffectLoop(); },
        "rgb_effect", 3072, this, tskIDLE_PRIORITY + 5, &effect_task_);

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create effect task");
        effect_task_exited_ = true;
        effect_task_ = nullptr;
    }
}

void RgbStrip::ApplyParamsInternal(const StripParams& params) {
    taskENTER_CRITICAL(&params_mux_);
    params_ = params;
    taskEXIT_CRITICAL(&params_mux_);
    last_gcode_ = ToGcode(params);
    blank_tail_needed_ = true;
    StartEffectLoop();
}

void RgbStrip::ApplyParams(const StripParams& params) {
    // 手动应用：之后状态切换不再覆盖它，直到用户点槽位或"恢复随状态联动"。
    manual_override_ = true;
    ApplyParamsInternal(params);
}

StripParams RgbStrip::GetParams() const {
    StripParams p;
    taskENTER_CRITICAL(&params_mux_);
    p = params_;
    taskEXIT_CRITICAL(&params_mux_);
    return p;
}

// ---------------------------------------------------------------- gcode 解析

namespace {

// gcode 第二段是毫秒（v206 语义），内部 speed 是 0-255（WLED 语义）。
// 映射区间取 200ms（最快）到 4000ms（最慢）。
uint8_t MsToSpeed(int ms) {
    if (ms <= 200) return 255;
    if (ms >= 4000) return 0;
    return (uint8_t)(255 - (ms - 200) * 255 / 3800);
}

// 反函数必须和 MsToSpeed 同向：speed 越大 = 周期越短 = 越快。
// 之前写成 200 + speed*3800/255，方向和 MsToSpeed 相反，
// 结果 gcode 存进 NVS 再读回来速度会翻个面（200 变成 55）。
int SpeedToMs(uint8_t speed) {
    return 200 + (int)(255 - speed) * 3800 / 255;
}

std::string Trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

bool ParseInt(const std::string& s, long lo, long hi, long* out) {
    if (s.empty()) return false;
    char* endp = nullptr;
    long v = strtol(s.c_str(), &endp, 10);
    if (endp == s.c_str() || *endp != '\0') return false;
    if (v < lo || v > hi) return false;
    *out = v;
    return true;
}

}  // namespace

bool RgbStrip::ParseToParams(const std::string& gcode, StripParams* out, std::string* error) {
    if (out == nullptr) {
        return false;
    }
    StripParams p;
    p.color_count = 0;

    std::vector<std::string> parts;
    std::string cur;
    for (char ch : gcode) {
        if (ch == ',') {
            parts.push_back(Trim(cur));
            cur.clear();
        } else {
            cur.push_back(ch);
        }
    }
    parts.push_back(Trim(cur));

    // 只写一个数字 = 只给数量。写 0 就是关灯，这是最顺手的写法，
    // 不用凑成 "0,0,0" 三段。
    if (parts.size() == 1) {
        long only = 0;
        if (ParseInt(parts[0], 0, 100, &only) && only == 0) {
            p.effect = kFxSolid;
            p.speed = 128;
            p.intensity = 128;
            p.colors[0] = StripColor{0, 0, 0};
            p.color_count = 1;
            *out = p;
            return true;
        }
    }

    if (parts.size() < 3) {
        if (error) *error = "格式应为 <数量>,<速度ms>,<效果>[,R,G,B...]";
        return false;
    }

    long count = 0;
    if (!ParseInt(parts[0], 0, 100, &count)) {
        if (error) *error = "数量必须是 0-100 的整数";
        return false;
    }
    long speed_ms = 0;
    if (!ParseInt(parts[1], 0, 10000, &speed_ms)) {
        if (error) *error = "速度必须是 0-10000 的毫秒数";
        return false;
    }
    p.speed = MsToSpeed((int)speed_ms);

    const std::string& fx = parts[2];
    if (fx.empty()) {
        if (error) *error = "缺少效果代号";
        return false;
    }

    // 第三段是数字 → 常亮，数字本身是 R，后面按颜色继续读
    size_t color_start = 3;
    size_t cursor = 2;
    if (fx[0] >= '0' && fx[0] <= '9') {
        p.effect = kFxSolid;
        color_start = 2;
        cursor = 2;
    } else {
        const char letter = (char)toupper((unsigned char)fx[0]);
        uint8_t id = FindEffectByLetter(letter);
        if (id >= kFxCount) {
            if (error) *error = std::string("未知效果代号 '") + fx + "'";
            return false;
        }
        p.effect = id;
        // F3 / F12 这种带次数
        if (letter == 'F' && fx.size() > 1) {
            long n = 0;
            if (!ParseInt(fx.substr(1), 0, 99, &n)) {
                if (error) *error = "闪烁次数必须是 0-99 的整数";
                return false;
            }
            p.c2 = (uint8_t)n;
        }
        color_start = 3;
        cursor = 3;
        (void)cursor;
    }

    // 剩下的数字按 RGB 三元组读，最多 6 组
    std::vector<long> nums;
    for (size_t i = color_start; i < parts.size(); ++i) {
        if (parts[i].empty()) continue;
        long v = 0;
        if (!ParseInt(parts[i], 0, 255, &v)) {
            if (error) *error = std::string("颜色分量必须是 0-255 的整数，收到 '") + parts[i] + "'";
            return false;
        }
        nums.push_back(v);
    }
    if (nums.size() % 3 != 0) {
        if (error) *error = "颜色分量必须是 3 的倍数（R,G,B 一组）";
        return false;
    }
    size_t groups = nums.size() / 3;
    if (groups > 6) groups = 6;
    for (size_t i = 0; i < groups; ++i) {
        p.colors[i] = StripColor{(uint8_t)nums[i * 3 + 0], (uint8_t)nums[i * 3 + 1], (uint8_t)nums[i * 3 + 2]};
    }
    p.color_count = (uint8_t)groups;

    const EffectDef& def = kEffects[p.effect];
    if (p.color_count < def.color_min) {
        if (error) {
            *error = std::string("效果「") + def.name + "」至少需要 " +
                     std::to_string(def.color_min) + " 组颜色";
        }
        return false;
    }

    // 数量为 0 等价于全灭（v206 的 "0,0,0"）
    if (count == 0) {
        p.effect = kFxSolid;
        p.colors[0] = StripColor{0, 0, 0};
        p.color_count = 1;
    }

    *out = p;
    return true;
}

std::string RgbStrip::ToGcode(const StripParams& p) {
    char head[32];
    const int ms = SpeedToMs(p.speed);
    if (p.effect == kFxSolid) {
        const StripColor c = p.ColorOf(0);
        snprintf(head, sizeof(head), "7,%d,%u,%u,%u", ms, c.r, c.g, c.b);
        return std::string(head);
    }
    const EffectDef& def = kEffects[p.effect < kFxCount ? p.effect : 0];
    std::string out;
    if (def.letter == 'F' && p.c2 > 0) {
        snprintf(head, sizeof(head), "7,%d,F%u", ms, (unsigned)p.c2);
    } else if (def.letter != 0) {
        snprintf(head, sizeof(head), "7,%d,%c", ms, def.letter);
    } else {
        snprintf(head, sizeof(head), "7,%d,0,0,0", ms);
    }
    out = head;
    for (uint8_t i = 0; i < p.color_count && i < 6; ++i) {
        char c[16];
        snprintf(c, sizeof(c), ",%u,%u,%u", p.ColorOf(i).r, p.ColorOf(i).g, p.ColorOf(i).b);
        out += c;
    }
    return out;
}

bool RgbStrip::ValidateGcode(const std::string& gcode, std::string* error) {
    StripParams p;
    return ParseToParams(gcode, &p, error);
}

bool RgbStrip::ApplyGcode(const std::string& gcode, std::string* error) {
    StripParams p;
    if (!ParseToParams(gcode, &p, error)) {
        return false;
    }
    // 保留原始文本：用户写 "0,0,0" 时不想被规范成 "7,200,0,0,0"
    ApplyParams(p);
    if (p.color_count == 1 && p.effect == kFxSolid && p.colors[0].r == 0 && p.colors[0].g == 0 &&
        p.colors[0].b == 0) {
        last_gcode_ = gcode;
    }
    return true;
}

// ---------------------------------------------------------------- 兼容接口

void RgbStrip::Blink(StripColor color, int times, int interval_ms) {
    StripParams p;
    p.effect = kFxBlink;
    p.speed = MsToSpeed(interval_ms * 2);   // gcode 的 period 是"一亮一灭一圈"
    p.intensity = 128;
    p.c2 = (uint8_t)(times < 0 ? 0 : (times > 99 ? 99 : times));
    p.colors[0] = color;
    p.color_count = 1;
    ApplyParams(p);
}

void RgbStrip::Rainbow(int speed_ms, int /*times*/) {
    StripParams p;
    p.effect = kFxRainbow;
    p.speed = MsToSpeed(speed_ms);
    p.intensity = 192;
    p.color_count = 0;
    ApplyParams(p);
}

// ---------------------------------------------------------------- NVS

void RgbStrip::LoadFromNvs() {
    Settings settings("led_config", true);

    // 亮度独立于槽位版本，不做 cfg_ver 重置，用户调过就一直保留
    int b = settings.GetInt("bright_lvl", 100);
    if (b < 1) b = 1;
    if (b > 100) b = 100;
    g_brightness_pct = b;
    ESP_LOGI(TAG, "Brightness = %d%%", b);

    // 槽位结构变了就整体重置一次。
    // 8：新增「语音律动」并把它设成默认的说话效果，倾听槽位改成灭灯。
    // 旧版本存下来的 listening/speaking 是红色呼吸和蓝色脉冲，语义已变，必须重置。
    constexpr int kCurrentVersion = 8;
    const int stored_version = settings.GetInt("cfg_ver", 0);
    const bool reset_all = (stored_version != kCurrentVersion);

    for (int i = 0; i < kSlotCount; ++i) {
        std::string def = kSlotDefaults[i];
        std::string raw = reset_all ? def : settings.GetString(kSlotKeys[i], def);
        StripParams p;
        std::string err;
        if (!ParseToParams(raw, &p, &err)) {
            ESP_LOGW(TAG, "Slot %s has invalid gcode '%s' (%s), falling back to default",
                     SlotName((Slot)i), raw.c_str(), err.c_str());
            raw = def;
            ParseToParams(raw, &p, &err);
        }
        slot_params_[i] = p;
        ESP_LOGI(TAG, "Slot %s = %s", SlotName((Slot)i), raw.c_str());
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
        settings.SetString(kSlotKeys[i], ToGcode(slot_params_[i]));
    }
    ESP_LOGI(TAG, "LED slots saved to NVS");
}

void RgbStrip::SetSlotParams(Slot slot, const StripParams& params) {
    if (slot < 0 || slot >= kSlotCount) {
        return;
    }
    slot_params_[slot] = params;
}

void RgbStrip::SetSlotGcode(Slot slot, const std::string& gcode) {
    if (slot < 0 || slot >= kSlotCount) {
        return;
    }
    StripParams p;
    std::string err;
    if (!ParseToParams(gcode, &p, &err)) {
        ESP_LOGW(TAG, "SetSlotGcode(%s) rejected: %s", SlotName(slot), err.c_str());
        return;
    }
    slot_params_[slot] = p;
}

StripParams RgbStrip::GetSlotParams(Slot slot) const {
    if (slot < 0 || slot >= kSlotCount) {
        return StripParams{};
    }
    return slot_params_[slot];
}

std::string RgbStrip::GetSlotGcode(Slot slot) const {
    if (slot < 0 || slot >= kSlotCount) {
        return {};
    }
    return ToGcode(slot_params_[slot]);
}

void RgbStrip::ApplySlot(Slot slot) {
    if (slot < 0 || slot >= kSlotCount) {
        return;
    }
    // 切槽位是"回到自动模式"，把手动覆盖清掉。
    manual_override_ = false;
    ApplyParamsInternal(slot_params_[slot]);
}
