#ifndef _RGB_STRIP_H_
#define _RGB_STRIP_H_

#include <driver/gpio.h>
#include <led_strip.h>
#include <atomic>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string>

#include "rgb_effects.h"

// WS2812 灯带驱动 + 效果调度。
//
// 和 main/led/single_led.cc 的区别：SingleLed 只驱动 1 颗、并且挂在 Led 基类上
// 跟着设备状态变颜色；这里驱动整条灯带、独立于 LED 基类，不参与状态联动。
//
// 效果调度只有一个任务（EffectLoop）：固定 50fps，用 vTaskDelayUntil 定周期，
// 每帧先让效果函数算出一整条像素，再一次性写进 RMT。这样做是为了不卡顿：
//  - 不再"每颗灯各拿一次锁、各刷一次"，一帧只拿一次锁、只 refresh 一次；
//  - 相位由绝对时间算，帧间隔抖一下也不会让画面顿住或跳格；
//  - 任务优先级抬到 idle+5，避免被 WiFi/音频任务压到几十毫秒才轮到一次。
class RgbStrip {
public:
    RgbStrip(gpio_num_t gpio, uint16_t led_count, uint16_t strip_length);
    ~RgbStrip();

    // 帧间隔。20ms = 50fps；FREERTOS_HZ=100 时正好 2 个 tick。
    static constexpr uint32_t kFrameMs = 20;

    // 效果驱动的颗数上限。改大了要同时抬 kMaxStripLength。
    static constexpr uint16_t kMaxLeds = 64;
    // 物理长度上限。RMT 按物理长度发数据，每颗约 30us，太长会吃掉太多 CPU 时间。
    static constexpr uint16_t kMaxStripLength = 300;

    uint16_t LedCount() const { return led_count_; }
    // 灯带物理长度。每帧会把 led_count_ 之后的这些颗显式写黑。
    uint16_t StripLength() const { return strip_length_; }

    // 用户手动设过效果就不再让状态切换覆盖它。
    // Web/工具手动应用会置位，切槽位（ApplySlot）会清除。
    bool ManualOverride() const { return manual_override_; }
    void ClearManualOverride() { manual_override_ = false; }

    // 尾段是否需要在下一帧补一次黑。稳态不写，省掉每帧一百多次 set_pixel。
    void MarkTailBlank() { blank_tail_needed_ = true; }

    // 运行时改驱动颗数：只改数字并写 NVS，不重建 RMT（物理长度固定）。
    // 返回改完之后的数量。
    uint16_t SetLedCount(uint16_t count);
    // 从 NVS 读上次保存的数量，没存过就用 fallback。
    static uint16_t LoadLedCount(uint16_t fallback);


    // 全局亮度百分比（1-100）。所有写灯的地方都会乘上它。
    // 存 NVS，重启保留。改完立刻重绘当前画面。
    void SetBrightness(int percent);
    static int GetBrightness();
    // 亮度单独落盘。不和 SaveToNvs 混在一起：拖滑杆会连着改几十次，
    // 每改一次就重写五个槽位没必要，也没那个寿命。
    static void PersistBrightness();

    // 最后一条真正生效的 gcode，用于回答"现在是什么效果"和重绘。
    const std::string& GetCurrentGcode() const { return last_gcode_; }

    // 当前生效的参数（拷贝，供 Web/工具查询）
    StripParams GetParams() const;

    void SetAll(StripColor color);
    void SetOne(int index, StripColor color);
    void Refresh();
    void TurnOff();

    // 一次性效果，保留给 MCP 工具用；内部转成参数效果。
    void Blink(StripColor color, int times, int interval_ms);
    void Rainbow(int speed_ms, int times);

    // 启动参数效果。remember=true 时记进 last_gcode_。
    void ApplyParams(const StripParams& params);

    // 停止正在跑的效果（任务置代号退出）
    void StopEffect();

    // 解析并执行 gcode。
    // 语法：<数量>,<速度ms>,<特效>[,R,G,B...]
    // 特效字母见 rgb_effects.cc 的 kEffects 表；第三段是纯数字时表示常亮颜色。
    bool ApplyGcode(const std::string& gcode, std::string* error);
    static bool ValidateGcode(const std::string& gcode, std::string* error);

    // gcode ⇄ 参数 的互转（Web 导入导出、NVS 旧数据迁移都用它）
    static bool ParseToParams(const std::string& gcode, StripParams* out, std::string* error);
    static std::string ToGcode(const StripParams& params);

    // NVS 槽位。顺序与 v206 的 /api/led/config 字段一致。
    enum Slot {
        kSlotStartupInitial = 0,   // 上电时先亮的效果
        kSlotStartupIdle,          // 上电稳定后的空闲效果
        kSlotLightshow,            // 彩灯秀
        kSlotListening,            // 正在听
        kSlotSpeaking,             // 正在说
        kSlotCount,
    };

    void ApplySlot(Slot slot);
    void SetSlotParams(Slot slot, const StripParams& params);
    void SetSlotGcode(Slot slot, const std::string& gcode);
    StripParams GetSlotParams(Slot slot) const;
    std::string GetSlotGcode(Slot slot) const;

    void LoadFromNvs();
    void SaveToNvs();

    static const char* SlotName(Slot slot);

private:
    gpio_num_t gpio_;
    uint16_t led_count_ = 0;
    uint16_t strip_length_ = 0;
    // 手动设过效果就置位，状态切换不再覆盖。
    std::atomic<bool> manual_override_{false};
    // 尾段需要补黑（上电、改颗数、换效果时置位，补完自动清）。
    volatile bool blank_tail_needed_ = true;
    led_strip_handle_t strip_ = nullptr;

    // 用 FreeRTOS 计数信号量而不是 std::mutex：std::lock_guard 没有超时，
    // 一旦某个任务卡在 led_strip_refresh() 里，其他调用会永久阻塞。
    SemaphoreHandle_t mutex_ = nullptr;

    // 参数与效果任务之间的互斥。参数只有十几个字节，用临界区足够，
    // 不动那把信号量——否则 Web 线程改参数会和正在刷帧的效果任务互等。
    // mutable：GetParams() 是 const，但临界区需要非 const 指针
    mutable portMUX_TYPE params_mux_ = portMUX_INITIALIZER_UNLOCKED;
    StripParams params_;

    std::string last_gcode_;

    // 效果代号：每次启动/停止都 +1。任务启动时记下自己的代号，
    // 发现 self 上的代号变了就退出。比共享 bool 可靠。
    volatile uint32_t effect_gen_ = 0;
    uint32_t effect_my_gen_ = 0;
    TaskHandle_t effect_task_ = nullptr;
    volatile bool effect_task_exited_ = true;

    // 效果相位起点。相位 = 当前毫秒 - 这个值，所以每个效果都从 0 开始，
    // 又不受"第几帧"影响。
    uint32_t effect_start_ms_ = 0;

    StripParams slot_params_[kSlotCount];

    void StartEffectLoop();
    void EffectLoop();
    // 真正落参数的地方：ApplyParams（手动）和 ApplySlot（自动）都走这里。
    void ApplyParamsInternal(const StripParams& params);
    void PaintFrame(const StripColor* px, uint16_t count);
    void FillBuffer(StripColor color);
    static uint32_t NowMs();
};

#endif  // _RGB_STRIP_H_
