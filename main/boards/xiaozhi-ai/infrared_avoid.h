#ifndef __INFRARED_AVOID_H__
#define __INFRARED_AVOID_H__

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <functional>

// 反射式红外避障模块（一对红外发射/接收管，三线 VCC/GND/OUT）。
//
// 输出是开关量：前方有遮挡物时电平翻转，没有就停在另一头。触发距离由模块
// 上的电位器决定，固件这边不读距离，只回答"前面有没有东西"。
//
// 极性按「有障碍 → 低电平」写（市面上大多数带 LM393 比较器的模块如此）。
// 实测相反就改 config.h 里的 INFRARED_AVOID_ACTIVE_LOW，这个文件不用动。
//
// 为什么不用 GPIO 中断：模块在临界距离上会抖（地面反光、电机 PWM 串扰），
// 中断会打出一串边沿，每次都要进 ISR 判一遍。20ms 采样 + 连续两次一致就
// 把抖动滤掉了，40ms 的延迟对刹车来说够快——100% 占空比下轮子 40ms 也就
// 走一两厘米。
class InfraredAvoidSensor {
public:
    static constexpr int kPollIntervalMs = 20;
    static constexpr int kStableSamples  = 2;
    static constexpr int kStopWaitMs     = 500;

    bool Begin(gpio_num_t pin, bool active_low = true);
    void Stop();

    bool     IsBlocked()   const { return blocked_.load(std::memory_order_relaxed); }
    int      RawLevel()    const { return raw_level_.load(std::memory_order_relaxed); }
    uint32_t TriggerCount() const { return trigger_count_.load(std::memory_order_relaxed); }
    gpio_num_t Pin()       const { return pin_; }
    bool     Ready()       const { return task_.load(std::memory_order_acquire) != nullptr; }

    // 只在「刚变成有障碍」的那一下调一次。回调跑在采样任务里，不能阻塞、
    // 不能在里面播音频或等信号量。
    // 注意：目前没有任何地方注册这个回调。
    // Application 走的是 AttachObstacleProbe() + 50ms 轮询（CheckObstacleWatchdog），
    // 不是靠回调驱动停车。留着是给「需要障碍出现那一刻立即反应」的场景（比如播音效），
    // 用之前先想清楚：回调跑在采样任务上下文里，不能在里面做耗时操作。
    void SetOnBlocked(std::function<void()> cb) { on_blocked_ = std::move(cb); }

private:
    static void PollTaskEntry(void* arg);
    void PollLoop();

    gpio_num_t pin_       = GPIO_NUM_NC;
    bool       active_low_ = true;
    std::atomic<bool>     blocked_{false};
    std::atomic<int>      raw_level_{1};
    std::atomic<uint32_t> trigger_count_{0};

    // running_ 是让任务自己退出的开关。Stop() 不能直接 vTaskDelete(task_)：
    // S3 是双核，删的那一刻它可能正在另一核上执行 gpio_get_level()，
    // 栈被回收后就是一串野指针。改成一个标志位，任务走到循环顶部看到它
    // 是 false 就自己 vTaskDelete(nullptr)。
    std::atomic<bool>         running_{false};
    std::atomic<TaskHandle_t> task_{nullptr};

    int stable_cnt_       = 0;
    std::function<void()> on_blocked_;
};

#endif  // __INFRARED_AVOID_H__
