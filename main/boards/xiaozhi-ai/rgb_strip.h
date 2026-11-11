#ifndef _RGB_STRIP_H_
#define _RGB_STRIP_H_

#include <driver/gpio.h>
#include <led_strip.h>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string>
#include <vector>

// WS2812 灯带驱动。
//
// 和 main/led/single_led.cc 的区别：SingleLed 只驱动 1 颗、并且挂在 Led 基类上
// 跟着设备状态变颜色；这里驱动整条灯带、独立于 LED 基类，不参与状态联动。
// 板载那颗（GPIO48）继续由 SingleLed 负责，两边各用各的 RMT 通道。
struct StripColor {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

class RgbStrip {
public:
    RgbStrip(gpio_num_t gpio, uint16_t led_count);
    ~RgbStrip();

    // 基础操作
    uint16_t LedCount() const { return led_count_; }

    // 全局亮度百分比（1-100）。所有写灯的地方都会乘上它，
    // 包括 gcode 里带的 RGB 和彩虹/呼吸这类满亮度效果。
    // 存 NVS，重启保留。
    // 非静态：改完亮度要立刻把当前画面重绘一遍，否则变量改了、灯不动，
    // 看起来就是"调了没反应"。重绘靠的是记住最后一条生效的 gcode。
    void SetBrightness(int percent);
    static int GetBrightness();

    // 最后一条真正生效的 gcode。用来回答"现在是什么效果"，
    // 也用于改亮度后原样重绘一遍。
    const std::string& GetCurrentGcode() const { return last_gcode_; }

    void SetAll(StripColor color);
    void SetOne(int index, StripColor color);
    void Refresh();
    void TurnOff();

    // 一次性效果（阻塞到结束或被打断）
    void Blink(StripColor color, int times, int interval_ms);
    void Rainbow(int speed_ms, int times);

    // 停止正在跑的效果（任务置标志退出）
    void StopEffect();

    // 解析并执行 gcode。失败时返回 false 并写 *error。
    // 语法：<LED数量>,<速度ms>,<特效>[,R,G,B]
    //   特效 W = 常亮（后接 R,G,B）
    //        K = 呼吸（后接 R,G,B）
    //        P = 脉冲（后接 R,G,B，缺省用当前色）
    //        R = 彩虹（不带 RGB）
    //        N = 灭灯（等价于 "0,0,0"）
    bool ApplyGcode(const std::string& gcode, std::string* error);

    // 只校验不执行，用于"保存前先检查"
    static bool ValidateGcode(const std::string& gcode, std::string* error);

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
    void SetSlotGcode(Slot slot, const std::string& gcode);
    std::string GetSlotGcode(Slot slot) const;

    void LoadFromNvs();
    void SaveToNvs();

    static const char* SlotName(Slot slot);

private:
    gpio_num_t gpio_;
    uint16_t led_count_ = 0;
    led_strip_handle_t strip_ = nullptr;
    // 用 FreeRTOS 计数信号量而不是 std::mutex：std::lock_guard 没有超时，
    // 一旦某个任务卡在 led_strip_refresh() 里，其他调用会永久阻塞。
    // 这里统一改成最长等 100ms，超时就放弃这一帧，至少不会把整个系统拖死。
    SemaphoreHandle_t mutex_ = nullptr;

    // 最后一条成功执行的 gcode。改亮度、恢复默认时靠它重绘。
    std::string last_gcode_;

    // 效果任务的控制标志。用 volatile 是因为它在另一个任务里被读。
    volatile bool effect_running_ = false;
    // 效果代号：每次启动/停止都 +1。任务在启动时记下自己的代号，
    // 一旦发现 self 上的代号变了就退出。比共享一个 bool 可靠——
    // 旧写法里 StartEffect 会把 bool 重置成 false，而旧任务可能还没
    // 检查到 true，于是两个效果同时跑，出现"呼吸叠彩虹"。
    volatile uint32_t effect_gen_ = 0;
    TaskHandle_t effect_task_ = nullptr;
    volatile bool effect_task_exited_ = true;

    std::string slot_gcode_[kSlotCount];

    void StartEffect(void (*fn)(RgbStrip*), const char* name);
    void FillBuffer(StripColor color);
};

#endif  // _RGB_STRIP_H_
