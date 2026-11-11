#include "roboeyes_adapter.h"
#include <esp_log.h>
#include <cstring>
#include <esp_timer.h>
#include <esp_system.h>
#include <stdlib.h>
#include <stdint.h>
#include <unordered_map>
#include <vector>
// Provide Arduino-like 'byte' type used by RoboEyes header
using byte = uint8_t;
// forward-declare Arduino-like helpers used inside the header
extern "C" unsigned long millis();
long random(long v);
// Include vendored RoboEyes header (vendored path)
#include "third_party/roboeyes/src/FluxGarage_RoboEyes.h"
#include <algorithm>

// Provide Arduino-like millis() and random() used by RoboEyes header
extern "C" unsigned long millis() {
    return static_cast<unsigned long>(esp_timer_get_time() / 1000ULL);
}

long random(long v) {
    if (v <= 0) return 0;
    return (long)(rand() % (uint32_t)v);
}

// Adafruit-like shim used by RoboEyes (file-scoped so type is visible)
class AdafruitShim {
public:
    AdafruitShim(uint8_t* bitbuf, int w, int h, RoboEyesAdapter* parent, const char* tag) : bitbuf_(bitbuf), w_(w), h_(h), parent_(parent), tag_(tag) {
        stride_ = (w_ + 7) / 8;
    }
    void clearDisplay() {
        if (bitbuf_) {
            memset(bitbuf_, 0x00, stride_ * h_);
            if (parent_ && parent_->verbose_logging()) ESP_LOGI(tag_, "AdafruitShim: clearDisplay called");
            if (parent_) parent_->SetDrewThisFrame(true);
        }
    }
    void display() {
        // no-op; adapter will invalidate LVGL canvas after RoboEyes update
    }
    void fillRoundRect(int x, int y, int w, int h, int r, uint8_t color) {
        bool on = color ? true : false;
        if (!bitbuf_) return;
        int x1 = x;
        int y1 = y;
        int x2 = x + w - 1;
        int y2 = y + h - 1;
        if (x1 < 0) x1 = 0;
        if (y1 < 0) y1 = 0;
        if (x2 >= w_) x2 = w_ - 1;
        if (y2 >= h_) y2 = h_ - 1;
        if (r > w/2) r = w/2;
        if (r > h/2) r = h/2;
        if (parent_ && parent_->verbose_logging()) ESP_LOGI(tag_, "AdafruitShim: fillRoundRect x=%d y=%d w=%d h=%d r=%d color=%d", x, y, w, h, r, color);
        if (parent_) parent_->SetDrewThisFrame(true);
        for (int yy = y1; yy <= y2; ++yy) {
            uint8_t* row = bitbuf_ + yy * stride_;
            for (int xx = x1; xx <= x2; ++xx) {
                bool should_draw = true;

                // Check if pixel is in a corner that should be rounded
                if (xx < x1 + r && yy < y1 + r) {
                    // Top-left corner
                    int dx = (x1 + r) - xx;
                    int dy = (y1 + r) - yy;
                    if (dx*dx + dy*dy > r*r) should_draw = false;
                } else if (xx > x2 - r && yy < y1 + r) {
                    // Top-right corner
                    int dx = xx - (x2 - r);
                    int dy = (y1 + r) - yy;
                    if (dx*dx + dy*dy > r*r) should_draw = false;
                } else if (xx < x1 + r && yy > y2 - r) {
                    // Bottom-left corner
                    int dx = (x1 + r) - xx;
                    int dy = yy - (y2 - r);
                    if (dx*dx + dy*dy > r*r) should_draw = false;
                } else if (xx > x2 - r && yy > y2 - r) {
                    // Bottom-right corner
                    int dx = xx - (x2 - r);
                    int dy = yy - (y2 - r);
                    if (dx*dx + dy*dy > r*r) should_draw = false;
                }
                // Non-corner areas are always drawn

                if (should_draw) {
                    int byte_idx = xx / 8;
                    int bit_idx = 7 - (xx % 8);
                    if (on) row[byte_idx] |= (1 << bit_idx);
                    else row[byte_idx] &= ~(1 << bit_idx);
                }
            }
        }
        // Dump a small region of the affected row for debugging to confirm bits were set
        {
            int sample_row = y1;
            if (sample_row < 0) sample_row = 0;
            if (sample_row >= h_) sample_row = h_ - 1;
            uint8_t* srow = bitbuf_ + sample_row * stride_;
            int byte_start = x1 / 8;
            int max_print = std::min(8, stride_ - byte_start);
            if (max_print > 0 && parent_ && parent_->verbose_logging()) {
                char dbg[128];
                int p = 0;
                p += snprintf(dbg + p, sizeof(dbg) - p, "AdafruitShim: rowdump y=%d:", sample_row);
                for (int i = 0; i < max_print; ++i) p += snprintf(dbg + p, sizeof(dbg) - p, " %02X", srow[byte_start + i]);
                ESP_LOGI(tag_, "%s", dbg);
            }
        }
    }
    void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint8_t color) {
        int minx = std::min({x0, x1, x2});
        int maxx = std::max({x0, x1, x2});
        int miny = std::min({y0, y1, y2});
        int maxy = std::max({y0, y1, y2});
        if (minx < 0) minx = 0;
        if (miny < 0) miny = 0;
        if (maxx >= w_) maxx = w_ - 1;
        if (maxy >= h_) maxy = h_ - 1;
        bool on = color ? true : false;
        auto edge = [](int x0,int y0,int x1,int y1,int x,int y)->bool{
            return (x - x0) * (y1 - y0) - (y - y0) * (x1 - x0) >= 0;
        };
        if (parent_ && parent_->verbose_logging()) ESP_LOGI(tag_, "AdafruitShim: fillTriangle x0=%d y0=%d x1=%d y1=%d x2=%d y2=%d color=%d", x0,y0,x1,y1,x2,y2,color);
        if (parent_) parent_->SetDrewThisFrame(true);
        for (int yy = miny; yy <= maxy; ++yy) {
            uint8_t* row = bitbuf_ + yy * stride_;
            for (int xx = minx; xx <= maxx; ++xx) {
                bool b0 = edge(x0,y0,x1,y1,xx,yy);
                bool b1 = edge(x1,y1,x2,y2,xx,yy);
                bool b2 = edge(x2,y2,x0,y0,xx,yy);
                if ((b0 && b1 && b2) || (!b0 && !b1 && !b2)) {
                    int byte_idx = xx / 8;
                    int bit_idx = 7 - (xx % 8);
                    if (on) row[byte_idx] |= (1 << bit_idx);
                    else row[byte_idx] &= ~(1 << bit_idx);
                }
            }
        }
    }
private:
    uint8_t* bitbuf_;
    int w_;
    int h_;
    int stride_ = 0;
    RoboEyesAdapter* parent_;
    const char* tag_;
};

static const char* TAG = "RoboEyesAdapter";

// Static map to associate timers with RoboEyesAdapter instances
static std::unordered_map<lv_timer_t*, RoboEyesAdapter*> g_timer_map;

void RoboEyesAdapter::RoboEyesTimerCallback(lv_timer_t* timer) {
    auto it = g_timer_map.find(timer);
    if (it != g_timer_map.end()) {
        it->second->Update();
    }
}

RoboEyesAdapter::RoboEyesAdapter() {}

bool RoboEyesAdapter::StopPanelTask(uint32_t timeout_ms) {
    if (panel_task_ == nullptr) {
        return true;
    }
    // 顺序不能反：先置标志，再等。任务在页循环里也检查这个标志，
    // 最坏只多等一页（100ms 的 I2C 超时）。
    panel_task_running_ = false;

    // 用 tick 计数，不要用 vTaskDelay(pdMS_TO_TICKS(5)) 累加毫秒。
    //
    // CONFIG_FREERTOS_HZ = 100，一个 tick 就是 10ms，
    // pdMS_TO_TICKS(5) 算出来是 0，vTaskDelay(0) 立即返回、根本不让出 CPU。
    // 结果就是 400 次循环在一瞬间跑完，2000ms 的等待变成 10ms，
    // 日志里表现为「StopPanelTask: task still alive after 2000 ms」紧跟着
    // 上一行打印，实际只过了十毫秒。任务其实还在正常跑，是我们没等它。
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    while (!panel_task_exited_ && (xTaskGetTickCount() - start) < timeout_ticks) {
        vTaskDelay(1);   // 至少让出 1 tick
    }
    if (!panel_task_exited_) {
        ESP_LOGE(TAG, "StopPanelTask: task still alive after %u ms", (unsigned)timeout_ms);
        return false;
    }
    panel_task_ = nullptr;
    return true;
}

RoboEyesAdapter::~RoboEyesAdapter() {
    // 第一件事必须是停掉刷新任务，并且等它真正退出。
    // 原来的析构直接去 delete 成员，但 roboeyes_panel_task 还在另一核上跑
    // Update() → DrawFrame() → FlushToPanel()，等它跑到 FlushToPanel 里
    // 访问已经析构的 std::vector<uint8_t> flush_raw_ 时，读到的 size 是垃圾值，
    // assign() 抛 length_error → 没有 catch → abort。
    // 实测栈：
    //   Guru Meditation (LoadProhibited), EXCVADDR 0x0000000a
    //   __throw_length_error ← vector::_M_fill_assign ← FlushToPanel ← DrawFrame
    // 触发条件就是反复按 BOOT 切显示模式（每次切回眼睛都会重建/销毁 adapter）。
    // 统一走 StopPanelTask。它失败时说明任务还在跑，这里只保护成员不释放，
    // 真正的保护在调用方：OledDisplay::SetAnimatedEmotionMode(false) 会用
    // release() 把整个对象泄漏掉，不让 delete 落到一个还活着的任务身上。
    if (!StopPanelTask(2000)) {
        ESP_LOGE(TAG, "~RoboEyesAdapter: panel task refused to stop, leaking members");
        return;
    }

    if (bit_buffer_ != nullptr) {
        delete[] bit_buffer_;
        bit_buffer_ = nullptr;
    }

    // 任务已经停了，再关掉 initialized_，兜住任何残留的 Update() 调用
    initialized_ = false;
    // stop any panel timer
    if (panel_timer_ != nullptr) {
        esp_timer_stop(panel_timer_);
        esp_timer_delete(panel_timer_);
        panel_timer_ = nullptr;
    }
    // delete RoboEyes instance and shim if created
    if (eyes_obj_) {
        // eyes_obj_ is a RoboEyes<AdafruitShim>*
        auto eyes_ptr = static_cast<RoboEyes<AdafruitShim>*>(eyes_obj_);
        delete eyes_ptr;
        eyes_obj_ = nullptr;
    }
    if (shim_obj_) {
        auto shim_ptr = static_cast<AdafruitShim*>(shim_obj_);
        delete shim_ptr;
        shim_obj_ = nullptr;
    }
}

bool RoboEyesAdapter::Begin(lv_obj_t* parent, int width, int height, int max_fps,
                           esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel) {
    if (initialized_) return true;
    if (width <= 0 || height <= 0 || parent == nullptr) return false;

    // store optional panel handles for direct flush
    panel_io_ = panel_io;
    panel_ = panel;

    width_ = width;
    height_ = height;
    max_fps_ = max_fps > 0 ? max_fps : 15;

    // allocate internal 1-bit packed buffer (stride bytes per row)
    int stride = (width_ + 7) / 8;
    bit_buffer_ = new uint8_t[stride * height_];
    if (bit_buffer_ == nullptr) {
        ESP_LOGE(TAG, "bit buffer alloc failed");
        return false;
    }
    // clear bit buffer
    memset(bit_buffer_, 0x00, stride * height_);

    // 脏区检测的上一帧缓存：和"待刷页数据"同样大小，初始全 0。
    // first_flush_ = true 保证第一帧无论内容如何都完整刷一遍，
    // 否则刚开机屏幕上是残留的旧画面，而 RoboEyes 第一帧恰好也是全黑时
    // memcmp 会认为"没变化"、一个字节都不发，屏幕就保持脏的。
    last_raw_.assign(static_cast<size_t>(((height_ + 7) / 8)) * width_, 0x00);
    first_flush_ = true;

    // canvas removed: panel_ 直刷路径不需要 LVGL canvas，省 16 KB 和每帧两遍 8192 次循环
    initialized_ = true;
    if (verbose_logging_) ESP_LOGI(TAG, "RoboEyesAdapter initialized %dx%d fps=%d", width_, height_, max_fps_);
    // Initialize RoboEyes with Adafruit-like shim
    // instantiate RoboEyes using the shim (store opaque pointers in header fields)
    AdafruitShim* shim = new AdafruitShim(bit_buffer_, width_, height_, this, TAG);
    shim_obj_ = static_cast<void*>(shim);
    // Set global colors expected by RoboEyes header
    BGCOLOR = 0;
    MAINCOLOR = 1;
    // create RoboEyes instance (template) and store opaque pointer
    auto eyes_ptr = new RoboEyes<AdafruitShim>(*shim);
    eyes_obj_ = static_cast<void*>(eyes_ptr);
    // RoboEyes uses millis()/internal timing; set framerate
    eyes_ptr->setFramerate(max_fps_);
    // 这一句以前漏了：不调 begin() 时 screenWidth/screenHeight 一直是头文件里
    // 写死的 128×64，眼睛按整屏居中画。分屏后只给 48 行，必须传进去，
    // 否则眼睛会画到下半屏的字幕区里去。
    eyes_ptr->begin(width_, height_, max_fps_);
    // Enable idle/autoblink modes so RoboEyes animates continually
    eyes_ptr->setIdleMode(true, 1, 3);     // idle repositioning every ~1s ± variation
    eyes_ptr->setAutoblinker(true, 3, 4);  // autoblink every ~3s ± variation

    // Start LVGL timer for periodic updates
    if (!StartTimer(max_fps_)) {
        ESP_LOGE(TAG, "Failed to start LVGL timer for RoboEyes updates");
        delete eyes_ptr;
        eyes_obj_ = nullptr;
        delete shim;
        shim_obj_ = nullptr;
        return false;
    }

    if (verbose_logging_) ESP_LOGI(TAG, "RoboEyes initialized with timer at %d fps", max_fps_);
    return true;
}

void RoboEyesAdapter::SetMood(const char* mood) {
    mood_ = mood ? mood : "neutral";
}

void RoboEyesAdapter::SetEmotion(const char* emotion) {
    if (!initialized_ || eyes_obj_ == nullptr) return;

    if (emotion == nullptr) emotion = "neutral";

    auto eyes_ptr = static_cast<RoboEyes<AdafruitShim>*>(eyes_obj_);

    // Reset all special modes first - IMPORTANT: Clear all previous animations!
    eyes_ptr->setIdleMode(false);
    eyes_ptr->setCuriosity(false);
    eyes_ptr->setSweat(false);
    eyes_ptr->setHFlicker(false);  // Clear horizontal flicker
    eyes_ptr->setVFlicker(false);  // Clear vertical flicker

    // Exact emoji and emotion mapping based on user's list
    // Handle English verb forms that might come from LLM emotion field
    if (strcmp(emotion, "smile") == 0) {
        // smile -> happy
        eyes_ptr->setMood(HAPPY);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to HAPPY (smile)");
    } else if (strcmp(emotion, "laugh") == 0) {
        // laugh -> laughing
        eyes_ptr->anim_laugh();
        eyes_ptr->setMood(HAPPY);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to LAUGHING (laugh)");
    } else if (strcmp(emotion, "cry") == 0) {
        // cry -> crying
        eyes_ptr->setMood(TIRED);
        eyes_ptr->setSweat(true);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to CRYING (cry)");
    } else if (strcmp(emotion, "wink") == 0) {
        // wink -> winking
        eyes_ptr->anim_laugh();
        eyes_ptr->setMood(HAPPY);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to WINKING (wink)");
    } else if (strcmp(emotion, "😶") == 0 || strcmp(emotion, "neutral") == 0) {
        // 1. 😶 - neutral
        eyes_ptr->setMood(DEFAULT);
        eyes_ptr->setIdleMode(true, 2, 4);  // Moderate idle movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to NEUTRAL (😶)");
    } else if (strcmp(emotion, "🙂") == 0 || strcmp(emotion, "happy") == 0) {
        // 2. 🙂 - happy (animated)
        eyes_ptr->setMood(HAPPY);
        eyes_ptr->setIdleMode(true, 1, 6);  // Gentle happy movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to HAPPY (🙂)");
    } else if (strcmp(emotion, "😆") == 0 || strcmp(emotion, "laughing") == 0) {
        // 3. 😆 - laughing
        eyes_ptr->anim_laugh();
        eyes_ptr->setMood(HAPPY);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to LAUGHING (😆)");
    } else if (strcmp(emotion, "😂") == 0 || strcmp(emotion, "funny") == 0) {
        // 4. 😂 - funny
        eyes_ptr->anim_laugh();
        eyes_ptr->setMood(HAPPY);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to FUNNY (😂)");
    } else if (strcmp(emotion, "😔") == 0 || strcmp(emotion, "sad") == 0) {
        // 5. 😔 - sad (animated with slow movement)
        eyes_ptr->setMood(TIRED);
        eyes_ptr->setIdleMode(true, 1, 8);  // Very slow, sad movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to SAD (😔)");
    } else if (strcmp(emotion, "😠") == 0 || strcmp(emotion, "angry") == 0) {
        // 6. 😠 - angry (animated with aggressive movement)
        eyes_ptr->setMood(ANGRY);
        eyes_ptr->setIdleMode(true, 2, 2);  // Quick, angry movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to ANGRY (😠)");
    } else if (strcmp(emotion, "😭") == 0 || strcmp(emotion, "crying") == 0) {
        // 7. 😭 - crying
        eyes_ptr->setMood(TIRED);
        eyes_ptr->setSweat(true);
        eyes_ptr->setIdleMode(true, 1, 5);  // Slow crying movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to CRYING (😭)");
    } else if (strcmp(emotion, "😍") == 0 || strcmp(emotion, "loving") == 0) {
        // 8. 😍 - loving (animated with heart eyes effect)
        eyes_ptr->setMood(LOVING);
        eyes_ptr->setIdleMode(true, 1, 7);  // Gentle loving movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to LOVING (😍)");
    } else if (strcmp(emotion, "😳") == 0 || strcmp(emotion, "embarrassed") == 0) {
        // 9. 😳 - embarrassed
        eyes_ptr->setIdleMode(true, 2, 2);  // Frequent idle movement for embarrassment
        eyes_ptr->setMood(DEFAULT);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to EMBARRASSED (😳)");
    } else if (strcmp(emotion, "😲") == 0 || strcmp(emotion, "surprised") == 0) {
        // 10. 😲 - surprised (animated with shocked movement)
        eyes_ptr->setMood(SURPRISED);
        eyes_ptr->setIdleMode(true, 2, 1);  // Quick, surprised movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to SURPRISED (😲)");
    } else if (strcmp(emotion, "😱") == 0 || strcmp(emotion, "shocked") == 0) {
        // 11. 😱 - shocked
        eyes_ptr->anim_confused();
        eyes_ptr->setMood(DEFAULT);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to SHOCKED (😱)");
    } else if (strcmp(emotion, "🤔") == 0 || strcmp(emotion, "thinking") == 0) {
        // 12. 🤔 - thinking
        eyes_ptr->setIdleMode(true, 2, 2);  // Looking around while thinking
        eyes_ptr->setMood(DEFAULT);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to THINKING (🤔)");
    } else if (strcmp(emotion, "😉") == 0 || strcmp(emotion, "winking") == 0) {
        // 13. 😉 - winking
        eyes_ptr->anim_laugh();
        eyes_ptr->setMood(HAPPY);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to WINKING (😉)");
    } else if (strcmp(emotion, "😎") == 0 || strcmp(emotion, "cool") == 0) {
        // 14. 😎 - cool (animated with sunglasses vibe)
        eyes_ptr->setMood(HAPPY);
        eyes_ptr->setIdleMode(true, 1, 10);  // Very slow, cool movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to COOL (😎)");
    } else if (strcmp(emotion, "😌") == 0 || strcmp(emotion, "relaxed") == 0) {
        // 15. 😌 - relaxed (animated with calm movement)
        eyes_ptr->setMood(HAPPY);
        eyes_ptr->setIdleMode(true, 1, 12);  // Extremely slow, relaxed movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to RELAXED (😌)");
    } else if (strcmp(emotion, "🤤") == 0 || strcmp(emotion, "delicious") == 0) {
        // 16. 🤤 - delicious (animated with drooling effect)
        eyes_ptr->setMood(HAPPY);
        eyes_ptr->setIdleMode(true, 1, 8);  // Slow, savoring movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to DELICIOUS (🤤)");
    } else if (strcmp(emotion, "😘") == 0 || strcmp(emotion, "kissy") == 0) {
        // 17. 😘 - kissy
        eyes_ptr->setMood(LOVING);
        eyes_ptr->setIdleMode(true, 2, 3);  // Playful kissy movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to KISSY (😘)");
    } else if (strcmp(emotion, "😏") == 0 || strcmp(emotion, "confident") == 0) {
        // 18. 😏 - confident (animated with knowing movement)
        eyes_ptr->setMood(HAPPY);
        eyes_ptr->setIdleMode(true, 1, 9);  // Slow, confident movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to CONFIDENT (😏)");
    } else if (strcmp(emotion, "😴") == 0 || strcmp(emotion, "sleepy") == 0) {
        // 19. 😴 - sleepy
        eyes_ptr->setMood(SLEEPY);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to SLEEPY (😴)");
    } else if (strcmp(emotion, "😜") == 0 || strcmp(emotion, "silly") == 0) {
        // 20. 😜 - silly
        eyes_ptr->anim_laugh();
        eyes_ptr->setMood(HAPPY);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to SILLY (😜)");
    } else if (strcmp(emotion, "🙄") == 0 || strcmp(emotion, "confused") == 0) {
        // 21. 🙄 - confused
        eyes_ptr->setIdleMode(true, 3, 1);  // More frequent movement for confusion
        eyes_ptr->setMood(DEFAULT);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to CONFUSED (🙄)");
    } else if (strcmp(emotion, "wake") == 0) {
        // 22. wake - waking up (animated sleepy to awake transition)
        eyes_ptr->setMood(SLEEPY);
        eyes_ptr->setIdleMode(true, 2, 3);  // Waking up movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to WAKE (sleepy animation)");
    } else if (strcmp(emotion, "listening") == 0) {
        // 23. listening - focused listening (use confident expression)
        eyes_ptr->setMood(HAPPY);
        eyes_ptr->setIdleMode(true, 1, 4);  // Slow, focused movement
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to LISTENING (confident)");
    } else if (strcmp(emotion, "speaking") == 0) {
        // 24. speaking - talking (use wink animation)
        eyes_ptr->anim_laugh();  // wink-like animation
        eyes_ptr->setMood(HAPPY);
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to SPEAKING (wink)");
    } else if (strcmp(emotion, "excited") == 0) {
        // 25. excited - excited (use excited jumping animation)
        eyes_ptr->setMood(HAPPY);
        eyes_ptr->setVFlicker(true, 8);  // Excited jumping up and down
        eyes_ptr->setIdleMode(true, 1, 2);  // Quick random movements
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to EXCITED (jumping)");
    } else if (strcmp(emotion, "twinkle") == 0) {
        // 26. twinkle - magical twinkling effect
        eyes_ptr->setMood(HAPPY);
        eyes_ptr->setHFlicker(true, 3);  // Gentle twinkling left-right
        eyes_ptr->setVFlicker(true, 3);  // Gentle twinkling up-down
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to TWINKLE (magical)");
    } else if (strcmp(emotion, "bounce") == 0) {
        // 27. bounce - bouncy playful animation
        eyes_ptr->setMood(HAPPY);
        eyes_ptr->setVFlicker(true, 12);  // Big bouncy movements
        eyes_ptr->setIdleMode(true, 1, 1);  // Very frequent position changes
        if (verbose_logging_) ESP_LOGI(TAG, "Set eyes to BOUNCE (playful)");
    } else {
        // Default to neutral for unknown emotions
        eyes_ptr->setMood(DEFAULT);
        eyes_ptr->setIdleMode(true, 2, 4);  // Moderate idle movement
        if (verbose_logging_) ESP_LOGI(TAG, "Unknown emotion '%s', set to DEFAULT", emotion);
    }
}


void RoboEyesAdapter::DrawFrame() {
    // canvas removed：这里只做一件事——让 RoboEyes 把当前表情画进 bit_buffer_，
    // 然后按 1-bit 页格式直刷 panel。原先的 RGB565 展开和白点统计在
    // panel_ 直刷路径上永远用不到，已经删除。
    if (!initialized_ || bit_buffer_ == nullptr) return;

    if (!eyes_obj_) return;

    auto eyes_ptr = static_cast<RoboEyes<AdafruitShim>*>(eyes_obj_);
    eyes_ptr->update();

    if (panel_ != nullptr) {
        FlushToPanel();
    }
}
void RoboEyesAdapter::DrawTestPattern() {
    if (!bit_buffer_) return;

    int stride = (width_ + 7) / 8;

    // Clear bit buffer
    memset(bit_buffer_, 0, stride * height_);

    // Draw a simple test pattern: alternating checkerboard
    for (int y = 0; y < height_; ++y) {
        uint8_t* row = bit_buffer_ + y * stride;
        for (int x = 0; x < width_; ++x) {
            bool on = ((x / 8) + (y / 8)) % 2 == 0;
            int byte_idx = x / 8;
            int bit_idx = 7 - (x % 8);
            if (on) row[byte_idx] |= (1 << bit_idx);
            else row[byte_idx] &= ~(1 << bit_idx);
        }
    }

    // canvas removed：测试图案也走 1-bit 直刷，不再展开成 RGB565
    if (panel_ != nullptr) FlushToPanel();

    if (verbose_logging_) ESP_LOGI(TAG, "Drew test checkerboard pattern");
}

void RoboEyesAdapter::Update() {
    if (!initialized_) return;
    DrawFrame();
    // After DrawFrame we ensure canvas buffer is up-to-date (demo path already updated canvas_buffer_)
    // For the RoboEyes path, DrawFrame handles conversion and invalidation.
}

bool RoboEyesAdapter::StartTimer(int fps) {
    if (update_timer_ != nullptr) {
        ESP_LOGW(TAG, "Timer already running");
        return true;
    }

    int interval_ms = 1000 / fps;
    if (interval_ms < 1) interval_ms = 1;

    // If we were given a panel handle (direct writes), use esp_timer instead
    if (panel_ != nullptr) {
        if (panel_task_ != nullptr) {
            ESP_LOGW(TAG, "Panel task already running");
            return true;
        }
        panel_task_running_ = true;
        panel_task_exited_ = false;
        // create a dedicated FreeRTOS task to drive updates in a safe context
        BaseType_t rc = xTaskCreatePinnedToCore(
            [](void* arg) {
                RoboEyesAdapter* self = static_cast<RoboEyesAdapter*>(arg);
                int delay_ms = 1000 / (self->max_fps_ > 0 ? self->max_fps_ : 15);
                if (delay_ms < 1) delay_ms = 1;
                uint32_t loops = 0;
                while (self->panel_task_running_) {
                    self->Update();
                    vTaskDelay(pdMS_TO_TICKS(delay_ms));
                    // 心跳：每秒打一条，用来区分"眼睛静止"和"任务卡死"。
                    // 如果这条日志停了，说明 Update() 或 vTaskDelay 再也没回来。
                    if (++loops % (uint32_t)(self->max_fps_ > 0 ? self->max_fps_ : 15) == 0) {
                        ESP_LOGI(TAG, "task alive, loops=%lu, stack free=%u",
                                 (unsigned long)loops,
                                 (unsigned)uxTaskGetStackHighWaterMark(nullptr));
                    }
                }
                // 必须在 vTaskDelete 之前置位：析构正在等这个标志，
                // 而 vTaskDelete(nullptr) 之后就再也没机会写它了。
                self->panel_task_exited_ = true;
                vTaskDelete(nullptr);
            },
            "roboeyes_panel_task",
            4096,
            this,
            tskIDLE_PRIORITY + 1,
            &panel_task_,
#if CONFIG_SOC_CPU_CORES_NUM > 1
            0
#else
            tskNO_AFFINITY
#endif
        );
        if (rc != pdPASS) {
            ESP_LOGE(TAG, "Failed to create panel task: %d", rc);
            panel_task_ = nullptr;
            panel_task_running_ = false;
            return false;
        }
        if (verbose_logging_) ESP_LOGI(TAG, "Started panel task for updates at ~%dms interval", interval_ms);
        return true;
    } else {
        update_timer_ = lv_timer_create(RoboEyesTimerCallback, interval_ms, this);
        if (update_timer_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create LVGL timer");
            return false;
        }
        g_timer_map[update_timer_] = this;
        if (verbose_logging_) ESP_LOGI(TAG, "Started LVGL timer with %dms interval", interval_ms);
        return true;
    }
}

void RoboEyesAdapter::StopTimer() {
    if (update_timer_ != nullptr) {
        lv_timer_del(update_timer_);
        g_timer_map.erase(update_timer_);
        update_timer_ = nullptr;
        ESP_LOGI(TAG, "Stopped LVGL timer");
    }
    if (panel_timer_ != nullptr) {
        esp_timer_stop(panel_timer_);
        esp_timer_delete(panel_timer_);
        panel_timer_ = nullptr;
        ESP_LOGI(TAG, "Stopped panel esp_timer");
    }
    if (panel_task_ != nullptr) {
        panel_task_running_ = false;
        // wait a short moment for task to exit
        vTaskDelay(pdMS_TO_TICKS(20));
        panel_task_ = nullptr;
        ESP_LOGI(TAG, "Stopped panel task");
    }
}

void RoboEyesAdapter::FlushToPanel() {
    if (!panel_ || !bit_buffer_) return;
    int stride = (width_ + 7) / 8;
    int pages = (height_ + 7) / 8;
    size_t len = static_cast<size_t>(pages) * static_cast<size_t>(width_);

    // 复用成员缓冲，不再每帧分配（见 roboeyes_adapter.h 里 flush_raw_ 的注释）
    if (flush_raw_.size() != len) {
        flush_raw_.assign(len, 0x00);
        last_raw_.assign(len, 0x00);
        first_flush_ = true;
    }
    std::vector<uint8_t>& raw = flush_raw_;

    // For each page (vertical 8-pixel band), for each column, pack bits LSB = top row in page
    for (int p = 0; p < pages; ++p) {
        for (int x = 0; x < width_; ++x) {
            uint8_t b = 0;
            for (int k = 0; k < 8; ++k) {
                int y = p * 8 + k;
                if (y >= height_) continue;
                int byte_idx = (y * stride) + (x / 8);
                int bit_idx = 7 - (x % 8); // match how AdafruitShim sets bits
                bool on = (bit_buffer_[byte_idx] >> bit_idx) & 0x1;
                if (on) b |= (1 << k);
            }
            raw[p * width_ + x] = b;
        }
    }

    // Write per-page to avoid driver/panel issues when sending a full frame at once.
    //
    // === 脏区检测（第 2 步）===
    // RoboEyes 一帧里只有眼睛那一小块在变，上下大部分页是纯黑。整屏 8 页 × 128
    // 字节 = 1024 字节，I2C 400 kHz 每字节 9 bit，一帧 23 ms，30 fps 就是 69% 的
    // 总线占用——同一条线上还挂着 VL6180X，被挤到 87% 超时率。
    // 这里逐页比较，只发从上到下发生变化的那段区间；整帧没变就一个字节都不发。
    int first_page = -1;
    int last_page = -1;

    if (first_flush_) {
        // 第一帧必须完整刷一遍：屏幕上可能残留上一次的画面，而 RoboEyes
        // 第一帧恰好全黑时 memcmp 会判"没变化"，脏画面就留在屏上了。
        first_page = 0;
        last_page = pages - 1;
        first_flush_ = false;
    } else if (last_raw_.size() != len) {
        last_raw_.assign(len, 0x00);
        first_page = 0;
        last_page = pages - 1;
    } else {
        for (int p = 0; p < pages; ++p) {
            const uint8_t* cur = raw.data() + (size_t)p * width_;
            const uint8_t* prev = last_raw_.data() + (size_t)p * width_;
            if (memcmp(cur, prev, width_) != 0) {
                if (first_page < 0) {
                    first_page = p;
                }
                last_page = p;
            }
        }
    }

    if (first_page < 0) {
        // 整帧一个字节都没变（眼睛完全静止），直接返回，总线零占用
        return;
    }

    bool any_timeout = false;
    for (int p = first_page; p <= last_page; ++p) {
        // 每页开始前看一眼停止信号。析构函数是「置 false 再等」，
        // 如果这里不检查，8 页各等 100ms 超时就是 800ms，
        // 正好等于析构的等待上限，谁也等不到谁。
        // 加了这一句之后最坏情况只多等一页（100ms）。
        if (!panel_task_running_) {
            return;
        }
        const uint8_t* page_ptr = raw.data() + (size_t)p * width_;
        int y_start = p * 8;
        int y_end = y_start + 8;
        if (y_end > height_) y_end = height_;
        esp_err_t err = esp_lcd_panel_draw_bitmap(panel_, 0, y_start, width_, y_end, page_ptr);
        if (err == ESP_ERR_TIMEOUT) {
            // panel io 的超时是 100ms（见 i2c_panel_io_timeout.h）。单次超时可能只是
            // 总线瞬时被 LVGL 占着，重试即可；连续 3 次说明 I2C 已经死了，
            // 这时候继续刷只会一直卡在超时里，主动停任务让析构能等到它退出。
            if (++consecutive_timeouts_ >= 3) {
                ESP_LOGE(TAG, "FlushToPanel: %d consecutive I2C timeouts, stopping panel task",
                         consecutive_timeouts_);
                panel_task_running_ = false;
                return;
            }
            ESP_LOGW(TAG, "FlushToPanel draw_bitmap page %d timeout (%d/3)", p, consecutive_timeouts_);
            any_timeout = true;
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "FlushToPanel draw_bitmap page %d failed: %d", p, err);
        } else {
            consecutive_timeouts_ = 0;   // 成功一次就把连击计数清零
            ESP_LOGD(TAG, "FlushToPanel wrote page %d (%d-%d)", p, y_start, y_end - 1);
        }
    }

    // 有页没刷上就别更新 last_raw_：下一帧会重新比较，把这几页补刷一遍。
    if (!any_timeout) {
        last_raw_.swap(raw);
    }

    // 脏区统计：每 100 帧打一次，用来量化脏区检测省了多少总线时间。
    // 8 页全刷 = 23ms/帧，按 30fps 算就是 69% 的 I2C 占用；这里能直接看出实际值。
    static uint32_t stat_frames = 0;
    static uint32_t stat_pages = 0;
    ++stat_frames;
    stat_pages += static_cast<uint32_t>(last_page - first_page + 1);
    if (stat_frames >= 100) {
        float avg_pages = static_cast<float>(stat_pages) / static_cast<float>(stat_frames);
        float ms_per_frame = avg_pages * 128.0f * 9.0f / 400.0f;   // 400 kHz: 每字节 9 bit
        ESP_LOGI(TAG, "Dirty stats: %.1f pages/frame, %.1f ms/frame, %.0f%% bus",
                 avg_pages, ms_per_frame, ms_per_frame * 30.0f / 10.0f);
        stat_frames = 0;
        stat_pages = 0;
    }
}
