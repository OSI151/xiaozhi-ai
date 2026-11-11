#pragma once

#include <lvgl.h>
#include <cstdint>
#include <vector>
#include <esp_lcd_panel_ops.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class RoboEyesAdapter {
public:
    RoboEyesAdapter();
    ~RoboEyesAdapter();

    // Initialize with LVGL parent and target size
    // 停刷新任务并等它真正退出。返回 true 表示安全，可以 delete。
    // 析构函数和 OledDisplay::SetAnimatedEmotionMode 都走这里。
    bool StopPanelTask(uint32_t timeout_ms = 2000);

    bool Begin(lv_obj_t* parent, int width, int height, int max_fps = 15,
               esp_lcd_panel_io_handle_t panel_io = nullptr, esp_lcd_panel_handle_t panel = nullptr);

    // Set mood string (e.g., "happy","sad")
    void SetMood(const char* mood);

    // Update internal animation state (called in LVGL-safe context)
    void Update();

    // Return whether initialized
    bool IsInitialized() const { return initialized_; }

    // Get canvas object

private:
    void DrawFrame();
    static void RoboEyesTimerCallback(lv_timer_t* timer);
    bool StartTimer(int fps);
    void StopTimer();

    // LVGL canvas buffer (RGB565) used for display update
    // Internal 1-bit buffer for RoboEyes drawing (packed bits, stride = (width+7)/8)
    uint8_t* bit_buffer_ = nullptr;
    // 脏区检测：上一帧打包好的页数据（每页 width_ 字节），和"还没刷过第一帧"标志。
    // RoboEyes 一帧只有眼睛在动，大部分页是纯黑，逐页 memcmp 后只发变化的页，
    // I2C 占用能从 69% 降到个位数。
    std::vector<uint8_t> last_raw_;
    bool first_flush_ = true;
    // 连续的 I2C 传输超时次数。panel io 那边的超时是 100ms（见 i2c_panel_io_timeout.h），
    // 连续 3 次说明总线真的挂了，这时候继续刷只会一直卡，主动让任务退出，
    // 好让析构函数能等到它，不至于再走"abandoning destructor"那条路。
    int consecutive_timeouts_ = 0;
    // 复用的打包缓冲。原实现每帧在 FlushToPanel() 里临时 new 一个 1024 字节的
    // std::vector，30 fps 下每秒 30 次 malloc/free，长时间运行会把内部堆碎掉，
    // 表现为眼睛停在某一帧不动、随后音频任务分配失败（喊唤醒词没反应）。
    std::vector<uint8_t> flush_raw_;
    // Optional LVGL indexed 1-bit buffer (palette + bit data) to avoid RGB conversion
    int width_ = 0;
    int height_ = 0;
    int max_fps_ = 15;

    // simple state
    const char* mood_ = "neutral";
    bool initialized_ = false;
    uint32_t frame_counter_ = 0;
    // opaque pointers for vendored RoboEyes objects
    void* eyes_obj_ = nullptr;
    void* shim_obj_ = nullptr;

    // LVGL timer for periodic updates
    lv_timer_t* update_timer_ = nullptr;
    // When flushing directly to panel we use esp_timer to drive updates
    esp_timer_handle_t panel_timer_ = nullptr;
    // Alternative: a dedicated FreeRTOS task for panel updates (used instead of esp_timer)
    TaskHandle_t panel_task_ = nullptr;
    // 必须是 volatile：析构函数在调用者的任务里置 false，panel task 在另一个核上跑。
    // 不加 volatile 时编译器会把这个值缓存在寄存器里，任务永远看不到停止信号，
    // 析构等满超时后进 abandoning 分支、对象被 delete，任务继续访问已释放的
    // eyes_obj_/bit_buffer_，表现为 LoadProhibited + 垃圾 EXCVADDR。
    volatile bool panel_task_running_ = false;
    // 任务真正退出的标志：任务在 vTaskDelete(nullptr) 之前置 true。
    // 析构函数靠它判断"可以安全销毁成员了"，比 eTaskGetState 可靠——
    // 任务自杀后句柄状态在 FreeRTOS 里可能先返回 eInvalid 再被复用，
    // 只看句柄状态会误判成"已经退出"，然后析构继续往下跑，
    // 而任务其实还卡在 AdafruitShim::fillTriangle 里写 bit_buffer_。
    volatile bool panel_task_exited_ = true;

private:
    // Debug tracking for drawing operations
    bool drew_this_frame_ = false;

public:
    void SetDrewThisFrame(bool drew) { drew_this_frame_ = drew; }

    void DrawTestPattern();
    // If panel handles are provided, adapter can flush frames directly to the panel (SSD1306)
    void FlushToPanel();

    // Optional panel handles for direct writes
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    // Control verbose logging (default off)
    bool verbose_logging_ = false;
    void SetVerboseLogging(bool v) { verbose_logging_ = v; }
    bool verbose_logging() const { return verbose_logging_; }

    // Set emotion/mood for the animated eyes
    void SetEmotion(const char* emotion);
};

