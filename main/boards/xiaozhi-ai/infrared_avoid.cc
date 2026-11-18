#include "infrared_avoid.h"

#include <esp_log.h>

#define TAG "InfraredAvoid"

bool InfraredAvoidSensor::Begin(gpio_num_t pin, bool active_low) {
    if (task_.load(std::memory_order_acquire) != nullptr) {
        ESP_LOGW(TAG, "already started on GPIO%d, ignore", (int)pin_);
        return true;
    }

    pin_       = pin;
    active_low_ = active_low;

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << pin_);
    io.mode         = GPIO_MODE_INPUT;
    // 便宜模块的输出常是开漏，没有上拉读到的是浮空值。推挽输出的模块
    // 这边多一个上拉也没有副作用。
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(GPIO%d) failed: %s", (int)pin_, esp_err_to_name(err));
        return false;
    }

    // 上电先按当前电平定一次初值，别默认「没障碍」——真要是对着墙开机，
    // 第一帧就是错的，得等两个采样周期才纠正。
    int level = gpio_get_level(pin_);
    raw_level_.store(level);
    blocked_.store(active_low_ ? (level == 0) : (level == 1));

    running_.store(true, std::memory_order_relaxed);
    TaskHandle_t handle = nullptr;
    if (xTaskCreate(PollTaskEntry, "infrared_avoid", 2560, this,
                    tskIDLE_PRIORITY + 2, &handle) != pdPASS) {
        ESP_LOGE(TAG, "failed to create poll task");
        running_.store(false, std::memory_order_relaxed);
        return false;
    }
    task_.store(handle, std::memory_order_release);

    ESP_LOGI(TAG, "GPIO%d ready: boot level=%d, blocked means %s, now %s",
             (int)pin_, level, active_low_ ? "LOW" : "HIGH",
             blocked_.load() ? "BLOCKED" : "clear");
    return true;
}

void InfraredAvoidSensor::Stop() {
    if (task_.load(std::memory_order_acquire) == nullptr) {
        return;
    }
    // 先让它自己退，别直接删。任务此刻可能正在另一核上跑 gpio_get_level()，
    // 把它的栈强行回收，返回地址就成了野指针。
    running_.store(false, std::memory_order_relaxed);

    for (int waited = 0; waited < kStopWaitMs; waited += kPollIntervalMs) {
        if (task_.load(std::memory_order_acquire) == nullptr) {
            ESP_LOGI(TAG, "poll task stopped");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
    }

    // 等超时了才强删。删之前先摘句柄，免得它和 PollLoop 尾部抢着写。
    TaskHandle_t h = task_.exchange(nullptr, std::memory_order_acq_rel);
    if (h != nullptr) {
        ESP_LOGW(TAG, "poll task did not exit in %d ms, force delete", kStopWaitMs);
        vTaskDelete(h);
    }
}

void InfraredAvoidSensor::PollTaskEntry(void* arg) {
    static_cast<InfraredAvoidSensor*>(arg)->PollLoop();
}

void InfraredAvoidSensor::PollLoop() {
    while (running_.load(std::memory_order_relaxed)) {
        int level = gpio_get_level(pin_);
        raw_level_.store(level);

        bool now = active_low_ ? (level == 0) : (level == 1);
        if (now == blocked_.load()) {
            // 一直在同一状态，把计数清掉，避免"抖三下攒够"。
            stable_cnt_ = 0;
        } else if (++stable_cnt_ >= kStableSamples) {
            blocked_.store(now);
            stable_cnt_ = 0;
            if (now) {
                trigger_count_.fetch_add(1);
                ESP_LOGI(TAG, "obstacle detected (level=%d, total=%u)",
                         level, (unsigned)trigger_count_.load());
                if (on_blocked_) {
                    on_blocked_();
                }
            } else {
                ESP_LOGI(TAG, "obstacle cleared (level=%d)", level);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
    }

    // 自己退出：先摘句柄让 Stop() 的等待循环看见，再删自己。
    // 顺序不能反——先 vTaskDelete(nullptr) 的话，句柄就永远摘不掉了。
    task_.store(nullptr, std::memory_order_release);
    vTaskDelete(nullptr);
}
