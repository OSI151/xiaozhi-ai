#include "vl6180x.h"

#include <cstring>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "settings.h"

static const char* TAG = "VL6180X";

namespace {

// ---------------------------------------------------------------- 寄存器地址
// 公开寄存器
constexpr uint16_t REG_MODEL_ID              = 0x0000;
constexpr uint16_t REG_SYSTEM_FRESH_OUT_OF_RESET = 0x0016;
constexpr uint16_t REG_SYSRANGE_START        = 0x0018;
// ST 数据手册：SYSTEM__INTERRUPT_CLEAR = 0x0015。
// 早先写成 0x001B（那是别的寄存器），清中断从来没生效，
// 导致状态字的 ready 位一直不复位，接着每次读都超时。
constexpr uint16_t REG_SYSTEM_INTERRUPT_CLEAR = 0x0015;
// ready 位在 RESULT__INTERRUPT_STATUS_GPIO（0x004F）的 bit2。
// 别跟 RESULT__RANGE_STATUS（0x004D）搞混：后者装的是量程状态码，
// 位定义完全不同，读它去等 ready 会一直等不到。
constexpr uint16_t REG_RESULT_INTERRUPT_STATUS = 0x004F;
// 量程状态码在这里。bit3~6 是状态码，0 表示正常。
constexpr uint16_t REG_RESULT_RANGE_STATUS   = 0x004D;
constexpr uint16_t REG_RESULT_RANGE_VAL      = 0x0062;

// 预期值
constexpr uint8_t  MODEL_ID_VL6180X          = 0xB4;

// 状态字位定义（RESULT_RANGE_STATUS）
constexpr uint8_t  STATUS_READY              = 0x04;   // bit2，测量完成
constexpr uint8_t  STATUS_CODE_MASK          = 0x78;   // bit3~6，量程状态

// I2C 时序
constexpr uint32_t I2C_TIMEOUT_MS            = 100;
constexpr uint8_t  DEVICE_ADDR               = 0x29;   // VL6180X 固定 7 位地址

// 采样节奏
constexpr int      SAMPLE_PERIOD_MS          = 50;     // 20 Hz
// 收敛时间 49ms 起步，留四倍余量。之前 100ms 就够，但那是地址写错时
// 一直读到无效寄存器，改成正确地址后这里要按真实收敛时间给足。
constexpr int      READY_TIMEOUT_MS          = 200;    // 单次测量的等待上限
constexpr int      INIT_RETRY_AFTER_FAILS    = 3;      // 连续超时几次后重写初始化寄存器
constexpr int      DEGRADE_AFTER_FAILS       = 20;     // 连续失败几次进降级
constexpr int      DEGRADE_RETRY_MS          = 5000;   // 降级后多久重试一次

// 障碍两段阈值（外加迟滞）。
//
// 传感器是斜向地面的，正常站在桌面上读数就在 150mm 上下晃，
// 所以"很远"这个档位没有意义——高于 120mm 一律当正常。
//
//   < 80mm       停车，并且禁前进
//   80 ~ 120mm   限速 40%
//   > 120mm      不干预
//
// 进 blocked 看 80，出 blocked 看 120，中间 80~120 就是迟滞带：
// 已经在 blocked 里时不会被 100mm 的读数解掉，必须退到 120 以外。
// 四档阈值改成了成员变量（th_blocked_mm_ 等），由 Application 的配置推过来。
// 原来的 kBlockedMm / kBlockedExitMm / kNearMm / kNearExitMm 常量已删。

// 桌沿：连续多少次"打空"才算确认。
//
// 传感器斜向地面，正常在桌上读数稳定在基线附近（实测 151mm）。
// 掉下桌沿那一刻激光打不到东西，读数会变成超量程 / 无目标 / 读取超时，
// 所以判据不是距离大小，而是"读没读到东西"。
//
// 2 次 = 100ms。8 条腿的底盘从桌沿翻下去只有几百毫秒，3 次来不及。
constexpr int      kEdgeConfirmCount         = 4;   // 200ms 连续打空

// 初始化寄存器表。这 30 组是 ST 官方推荐值，不能省——
// VL6180X 上电后内部状态机是关闭的，只有写完整套才会进入可用状态。
// 传感器内部卡死时，也是重写这整套能把它拉回来。
struct RegValue {
    uint16_t reg;
    uint8_t  value;
};

const RegValue kInitTable[] = {
    {0x0207, 0x01}, {0x0208, 0x01}, {0x0096, 0x00}, {0x0097, 0xfd},
    {0x00e3, 0x00}, {0x00e4, 0x04}, {0x00e5, 0x02}, {0x00e6, 0x01},
    {0x00e7, 0x03}, {0x00f5, 0x02}, {0x00d9, 0x05}, {0x00db, 0xce},
    {0x00dc, 0x03}, {0x00dd, 0xf8}, {0x009f, 0x00}, {0x00a3, 0x3c},
    {0x00b7, 0x00}, {0x00bb, 0x3c}, {0x00b2, 0x09}, {0x00ca, 0x09},
    {0x0198, 0x01}, {0x01b0, 0x17}, {0x01ad, 0x00}, {0x00ff, 0x05},
    {0x0100, 0x05}, {0x0199, 0x05}, {0x01a6, 0x1b}, {0x01ac, 0x3e},
    {0x01a7, 0x1f}, {0x0030, 0x00},
};

// 应用层配置：采样平均、收敛时间、增益、测距偏置
// 实测修正：早先这张表里的地址是凭印象写的，几个都不对——
//   0x0011 不是中断配置（那是 0x0014）
//   0x010a 是读出平均采样周期，不是最大收敛时间（0x001C 才是）
//   0x0031 是两次测量的间隔，不是量程偏置
// 表现是初始化全过、但每次读状态字都超时。按 ST 数据手册重写：
const RegValue kConfigTable[] = {
    {0x0014, 0x04},   // SYSTEM__INTERRUPT_CONFIG_GPIO：新样本时置位
    {0x001c, 0x31},   // SYSRANGE__MAX_CONVERGENCE_TIME = 49 ms
    {0x0031, 0x09},   // SYSRANGE__INTERMEASUREMENT_PERIOD = 9 ms
    {0x003f, 0x46},   // SYSALS__ANALOGUE_GAIN = 1.0
    {0x0040, 0x63},   // SYSALS__INTEGRATION_PERIOD
    {0x002e, 0x01},   // SYSRANGE__VHV_RECALIBRATE
};

int64_t NowMs() {
    return esp_timer_get_time() / 1000;
}

}  // namespace

// 声明在头文件里，实现在这里——age_ms 需要 esp_timer，头文件不想拖这个依赖。
int SensorSnapshot::age_ms() const {
    if (stamp_ms == 0) {
        return 0x7fffffff;   // 从没采到过
    }
    return static_cast<int>(NowMs() - stamp_ms);
}

Vl6180xSensor::~Vl6180xSensor() {
    running_ = false;
    // 等任务自己退出。不等的话它会继续往已释放的成员里写。
    //
    // 按 tick 计时，不要用 vTaskDelay(pdMS_TO_TICKS(5)) 累加毫秒：
    // CONFIG_FREERTOS_HZ = 100 时 pdMS_TO_TICKS(5) = 0，
    // vTaskDelay(0) 立即返回，等于忙等，一圈瞬间跑完根本没等。
    TickType_t start = xTaskGetTickCount();
    while (!task_exited_ && (xTaskGetTickCount() - start) < pdMS_TO_TICKS(2000)) {
        vTaskDelay(1);
    }
    if (!task_exited_) {
        // 任务还在跑，绝不能删名字。宁可泄漏也不制造 use-after-free。
        ESP_LOGE(TAG, "sample task did not exit in time, leaking sensor object");
        return;
    }
    if (dev_ != nullptr && bus_ != nullptr) {
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
    }
}

// ------------------------------------------------------------------ I2C 基础
bool Vl6180xSensor::WriteReg(uint16_t reg, uint8_t value) {
    uint8_t buf[3] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xff),
        value,
    };
    esp_err_t err = i2c_master_transmit(dev_, buf, sizeof(buf), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write reg 0x%04x failed: %s", reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

bool Vl6180xSensor::ReadRegs(uint16_t reg, uint8_t* buf, size_t len) {
    uint8_t addr[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xff),
    };
    esp_err_t err = i2c_master_transmit_receive(dev_, addr, sizeof(addr), buf, len,
                                                I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "read reg 0x%04x failed: %s", reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

bool Vl6180xSensor::ReadReg(uint16_t reg, uint8_t* value) {
    return ReadRegs(reg, value, 1);
}

// ------------------------------------------------------------------ 初始化
bool Vl6180xSensor::InitRegisters() {
    // 先确认芯片在
    uint8_t model_id = 0;
    if (!ReadReg(REG_MODEL_ID, &model_id)) {
        ESP_LOGE(TAG, "cannot read MODEL_ID");
        return false;
    }
    if (model_id != MODEL_ID_VL6180X) {
        ESP_LOGE(TAG, "MODEL_ID = 0x%02x, expected 0x%02x (VL6180X)", model_id, MODEL_ID_VL6180X);
        return false;
    }

    // 芯片要求：远离 reset 后必须先写 0x01 到 SYSTEM_FRESH_OUT_OF_RESET
    uint8_t fresh = 0;
    if (ReadReg(REG_SYSTEM_FRESH_OUT_OF_RESET, &fresh) && fresh == 0x01) {
        if (!WriteReg(REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00)) {
            return false;
        }
    }

    for (const auto& rv : kInitTable) {
        if (!WriteReg(rv.reg, rv.value)) {
            ESP_LOGE(TAG, "init table failed at reg 0x%04x", rv.reg);
            return false;
        }
    }
    for (const auto& rv : kConfigTable) {
        if (!WriteReg(rv.reg, rv.value)) {
            ESP_LOGE(TAG, "config table failed at reg 0x%04x", rv.reg);
            return false;
        }
    }
    return true;
}

void Vl6180xSensor::SetThresholds(int blocked_mm, int blocked_exit_mm,
                                  int near_mm, int near_exit_mm) {
    // 顺序不合法就整组丢弃，保持原值。
    // 只写一半的话会得到「停车 150 / 解除 100」这种自相矛盾的组合。
    if (!(blocked_mm < blocked_exit_mm && blocked_exit_mm < near_mm &&
          near_mm < near_exit_mm)) {
        ESP_LOGE(TAG, "SetThresholds: invalid order %d/%d/%d/%d, keeping previous",
                 blocked_mm, blocked_exit_mm, near_mm, near_exit_mm);
        return;
    }
    th_blocked_mm_      = blocked_mm;
    th_blocked_exit_mm_ = blocked_exit_mm;
    th_near_mm_         = near_mm;
    th_near_exit_mm_    = near_exit_mm;
    ESP_LOGI(TAG, "thresholds set: blocked<%d exit>%d near<%d exit>%d",
             blocked_mm, blocked_exit_mm, near_mm, near_exit_mm);
}

bool Vl6180xSensor::Begin(i2c_master_bus_handle_t bus) {
    bus_ = bus;

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = DEVICE_ADDR;
    dev_cfg.scl_speed_hz = 400000;

    esp_err_t err = i2c_master_bus_add_device(bus_, &dev_cfg, &dev_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add device failed: %s", esp_err_to_name(err));
        dev_ = nullptr;
        return false;
    }

    if (!InitRegisters()) {
        ESP_LOGE(TAG, "initialization failed");
        return false;
    }

    sensor_ok_ = true;
    fail_count_ = 0;
    ESP_LOGI(TAG, "VL6180X ready on sensor bus");
    return true;
}

// ------------------------------------------------------------------ 单次测量
bool Vl6180xSensor::ReadOnce(int* distance_mm, int* status) {
    // 启动单次测量
    if (!WriteReg(REG_SYSRANGE_START, 0x01)) {
        *status = 3;
        return false;
    }

    // 轮询 STATUS 的 bit2。VL6180X 单次测量典型 20~30ms，
    // 100ms 上限对最坏情况留了三倍余量。
    // 按 tick 计时。原来用 vTaskDelay(pdMS_TO_TICKS(5)) + waited += 5，
    // 但 100Hz 下 5ms 换算成 0 个 tick，延时立即返回 —— 整个轮询变成
    // 忙着读 I2C，ReadOnce 一发起就把总线占满，这才是 I2C 超时的来源之一。
    // 现在每次固定让出 1 tick（10ms），100ms 上限就是 10 次。
    TickType_t start = xTaskGetTickCount();
    uint8_t st = 0;
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(READY_TIMEOUT_MS)) {
        vTaskDelay(1);
        if (!ReadReg(REG_RESULT_INTERRUPT_STATUS, &st)) {
            *status = 3;
            return false;
        }
        if (st & STATUS_READY) {
            break;
        }
    }
    if ((st & STATUS_READY) == 0) {
        // 超时。清中断、把测量停掉，避免下一次读到的还是这一次的残值。
        WriteReg(REG_SYSTEM_INTERRUPT_CLEAR, 0x07);
        WriteReg(REG_SYSRANGE_START, 0x00);
        *status = 4;
        return false;
    }

    uint8_t range = 0;
    if (!ReadRegs(REG_RESULT_RANGE_VAL, &range, 1)) {
        *status = 3;
        return false;
    }
    WriteReg(REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

    *distance_mm = static_cast<int>(range);

    // 量程状态码在 RESULT__RANGE_STATUS（0x004D）的 bit3~6，
    // 跟上面等 ready 的那个寄存器不是同一个，得单独读。
    uint8_t range_status = 0;
    uint8_t code = 0;
    if (ReadReg(REG_RESULT_RANGE_STATUS, &range_status)) {
        code = static_cast<uint8_t>((range_status & STATUS_CODE_MASK) >> 3);
    }
    if (code != 0) {
        // 常见：1 = 超出量程（太远或太近）、2 = 目标过弱、11 = 环境光过强
        *status = (code == 1) ? 1 : 2;
        return true;
    }
    *status = 0;
    return true;
}

// ------------------------------------------------------------------ 采样任务
void Vl6180xSensor::TaskEntry(void* arg) {
    static_cast<Vl6180xSensor*>(arg)->TaskLoop();
}

void Vl6180xSensor::TaskLoop() {
    int16_t degrade_retry_ms = 0;

    while (running_) {
        if (!sensor_ok_) {
            // 降级态：每 5 秒试一次能不能拉回来
            if (degrade_retry_ms <= 0) {
                ESP_LOGW(TAG, "sensor degraded, retrying initialization");
                if (InitRegisters()) {
                    ESP_LOGI(TAG, "sensor recovered");
                    sensor_ok_ = true;
                    fail_count_ = 0;
                }
                degrade_retry_ms = DEGRADE_RETRY_MS;
            }
            degrade_retry_ms -= SAMPLE_PERIOD_MS;
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        int distance = 0;
        int status = 0;
        bool ok = ReadOnce(&distance, &status);

        SensorSnapshot snap = {};
        snap.stamp_ms = NowMs();
        snap.status = status;
        snap.distance_mm = (status == 0 || status == 1 || status == 2) ? distance : 0;
        snap.baseline_mm = baseline_mm_;
        snap.sensor_ok = sensor_ok_;
        snap.blocked = false;

        if (ok) {
            fail_count_ = 0;
        } else {
            ++fail_count_;

            // 连续 3 次超时 → 重写整套初始化寄存器。
            // 这一步不是可选的：VL6180X 内部状态机卡死时，
            // 重新发 START 没用，只有把 30 个寄存器重写一遍才能恢复。
            if (fail_count_ % INIT_RETRY_AFTER_FAILS == 0 && fail_count_ < DEGRADE_AFTER_FAILS) {
                ESP_LOGW(TAG, "%d consecutive failures, re-initializing", fail_count_);
                if (!InitRegisters()) {
                    ESP_LOGE(TAG, "re-initialization failed");
                }
            }
            if (fail_count_ >= DEGRADE_AFTER_FAILS) {
                sensor_ok_ = false;
                ESP_LOGE(TAG, "%d consecutive failures, sensor marked unavailable", fail_count_);
                degrade_retry_ms = DEGRADE_RETRY_MS;
            }
        }

        // ---- 判定层 ----
        //
        // 障碍三档，每档都有自己的进出线（迟滞）。
        // 用同一条线进出的话，读数卡在阈值上会每 50ms 翻一次状态。
        ObstacleState next = obstacle_;
        if (ok) {
            if (status == 1) {
                // 超量程 = 前方 200mm 内没有东西，等价于"很远"。
                // 不处理的话，障碍被拿走后状态会永远卡在 blocked 出不来。
                next = ObstacleState::kClear;
            } else if (status == 0) {
                // 两段各自带迟滞。不用同一条线进出，否则读数在阈值附近
                // 抖动时状态每 50ms 翻一次——日志里那种 0->1->0->1 连刷
                // 就是这么来的（120mm 正卡在限速线上）。
                if (obstacle_ == ObstacleState::kBlocked) {
                    // 已经在停车档：只有退到 100mm 以外才解除
                    next = (distance > th_blocked_exit_mm_) ? ObstacleState::kNear
                                                       : ObstacleState::kBlocked;
                    if (distance > th_near_exit_mm_) {
                        next = ObstacleState::kClear;
                    }
                } else if (obstacle_ == ObstacleState::kNear) {
                    if (distance < th_blocked_mm_) {
                        next = ObstacleState::kBlocked;
                    } else if (distance > th_near_exit_mm_) {
                        next = ObstacleState::kClear;
                    }
                } else {
                    // kClear
                    if (distance < th_blocked_mm_) {
                        next = ObstacleState::kBlocked;
                    } else if (distance < th_near_mm_) {
                        next = ObstacleState::kNear;
                    }
                }

            }
        }

        if (next != obstacle_) {
            int old_state = (int)obstacle_;
            ESP_LOGW(TAG, "obstacle %d -> %d (distance %d mm)",
                     old_state, (int)next, distance);
            obstacle_ = next;
            if (obstacle_cb_) {
                obstacle_cb_(old_state, (int)next);
            }
        }

        // 桌沿：读不到有效值（超时 / 超量程 / 无目标）都算候选。
        // 没校准过基线就不判，免得把"正常没东西"当成桌沿。
        if (ok && status == 0) {
            last_valid_ms_ = snap.stamp_ms;
            last_valid_mm_ = distance;
        }

        // 桌沿候选 = 持续打空。
        //
        // 第一版只数"连续几次 status 非 0"，结果在桌上就误报——VL6180X
        // 偶发单帧 no_target 很常见（日志里 100ms 前还有有效读数就判了）。
        //
        // 加第二个条件：距上次有效读数超过 300ms。真掉桌沿时是持续读不到，
        // 时间会一直涨；偶发单帧的话下一次采样就刷新了 last_valid_ms_，
        // 时间归零，判不出来。
        //
        // 这里不看 distance：打空的时候 distance 是无效值。
        bool edge_candidate = false;
        if (last_valid_ms_ > 0) {
            int64_t lost_ms = snap.stamp_ms - last_valid_ms_;
            edge_candidate = (!ok || status == 1 || status == 2) && lost_ms > 300;
        }
        if (baseline_mm_ > 0) {
            EdgeState prev_edge = edge_;
            if (edge_candidate) {
                if (edge_confirm_ < kEdgeConfirmCount) {
                    ++edge_confirm_;
                }
                if (edge_confirm_ >= kEdgeConfirmCount) {
                    edge_ = EdgeState::kEdge;
                }
            } else {
                edge_confirm_ = 0;
                edge_ = EdgeState::kSafe;
            }

            if (edge_ != prev_edge) {
                if (edge_ == EdgeState::kEdge) {
                    ESP_LOGW(TAG, "edge detected (status %d, last valid %d mm ago)",
                             status, (int)(snap.stamp_ms - last_valid_ms_));
                } else if (prev_edge == EdgeState::kEdge) {
                    ESP_LOGI(TAG, "back on the table (distance %d mm)", distance);
                }
                if (edge_cb_) {
                    edge_cb_(edge_ == EdgeState::kEdge);
                }
            }
        }

        snap.fail_count = fail_count_;
        snap.sensor_ok = sensor_ok_;
        snap.obstacle = (int)obstacle_;
        snap.edge = (int)edge_;
        snap.edge_confirm = edge_confirm_;
        snap.blocked = (obstacle_ == ObstacleState::kBlocked);

        portENTER_CRITICAL(&lock_);
        cache_ = snap;
        portEXIT_CRITICAL(&lock_);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }

    task_exited_ = true;
    vTaskDelete(nullptr);
}

void Vl6180xSensor::Start() {
    if (task_ != nullptr) {
        return;
    }
    LoadBaseline();
    running_ = true;
    task_exited_ = false;
    // 3072 足够：ReadOnce 里最多一层调用，没有大局部数组。
    // 优先级 +2 高于 idle，但不挡音频和显示任务。
    BaseType_t ret = xTaskCreatePinnedToCore(TaskEntry, "vl6180x", 3072, this,
                                             tskIDLE_PRIORITY + 2, &task_, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create sample task");
        running_ = false;
        task_exited_ = true;
        task_ = nullptr;
        return;
    }
    ESP_LOGI(TAG, "sample task started (%d ms period)", SAMPLE_PERIOD_MS);
}

SensorSnapshot Vl6180xSensor::Snapshot() const {
    portENTER_CRITICAL(&lock_);
    SensorSnapshot copy = cache_;
    portEXIT_CRITICAL(&lock_);
    // stamp_ms 是最后一次成功刷新的时刻，age 由调用方按需算
    return copy;
}

// ------------------------------------------------------------------ 校准
bool Vl6180xSensor::Calibrate(int* baseline_mm_out) {
    if (!sensor_ok_) {
        ESP_LOGE(TAG, "calibrate: sensor not available");
        return false;
    }

    // 采样期间停掉后台任务，避免它和校准抢总线
    running_ = false;
    for (int i = 0; i < 100 && !task_exited_; ++i) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    int samples[20];
    int got = 0;
    for (int i = 0; i < 20; ++i) {
        int d = 0;
        int s = 0;
        if (ReadOnce(&d, &s) && s == 0) {
            samples[got++] = d;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }

    // 恢复后台任务
    running_ = true;
    task_exited_ = false;
    xTaskCreatePinnedToCore(TaskEntry, "vl6180x", 3072, this, tskIDLE_PRIORITY + 2, &task_, 1);

    if (got < 10) {
        ESP_LOGE(TAG, "calibrate: only %d/20 valid samples", got);
        return false;
    }

    // 排序取中位数。用插入排序，20 个元素不值得引入 qsort。
    for (int i = 1; i < got; ++i) {
        int key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            --j;
        }
        samples[j + 1] = key;
    }
    int median = samples[got / 2];

    // 极差太大说明读数不稳定（手在传感器前晃、桌面反光、震动），拒绝写入。
    int spread = samples[got - 1] - samples[0];
    if (spread > median / 2 && spread > 30) {
        ESP_LOGE(TAG, "calibrate: unstable readings (spread %d mm on median %d mm)", spread, median);
        return false;
    }

    baseline_mm_ = median;
    SaveBaseline(median);
    if (baseline_mm_out != nullptr) {
        *baseline_mm_out = median;
    }
    ESP_LOGI(TAG, "calibrate: baseline = %d mm (%d samples, spread %d mm)", median, got, spread);
    return true;
}

// ------------------------------------------------------------------ NVS
void Vl6180xSensor::LoadBaseline() {
    // 存在 NVS 的 sensor 命名空间下，键 baseline。读不到就是 0，
    // 表示"没校准过"，不阻断启动。
    Settings settings("sensor", false);
    baseline_mm_ = settings.GetInt("baseline", 0);
    ESP_LOGI(TAG, "baseline from NVS: %d mm", baseline_mm_);
}

void Vl6180xSensor::SaveBaseline(int baseline_mm) {
    Settings settings("sensor", true);
    settings.SetInt("baseline", baseline_mm);
}
