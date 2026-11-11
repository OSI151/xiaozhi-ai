#pragma once

#include <atomic>
#include <functional>
#include <cstdint>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// VL6180X 激光测距（模块丝印 TOF050C-VL6180X）。
//
// 这个类自己持有一个后台任务，按固定周期刷新缓存；工具和 Web 只读缓存，
// 不去碰 I2C。这么做是为了解决 v206 的病：「偶尔读到，后来怎么也读不到」——
// 那边是调用方直接对着总线轮询，一旦卡在某个不返回的测量里，
// 后面每次调用都读到同一个死值。
//
// 自愈路径：
//   单次超时      → 清中断、重发 START
//   连续 3 次超时 → 完整重写初始化寄存器（内部状态机卡死时只有这个能拉回来）
//   连续 20 次失败 → sensor_ok_ = false，进降级；之后每 5 秒重试一次 init()
// 障碍判定的三档状态。带迟滞，避免读数在阈值附近抖动时来回切。
//
//   距离 < 50mm   → kBlocked  停车、禁止前进
//   50 ~ 80mm     → kNear     限速 40%
//   > 100mm       → kClear    不干预
//
// 进 kBlocked 只需 < 50mm，但退出必须 > 100mm，中间的 50~100mm 是迟滞带。
// 这样物体在 50mm 上下抖动时电机状态不会每帧翻转。
enum class ObstacleState {
    kClear = 0,
    kNear = 1,
    kBlocked = 2,
};

// 桌沿判定。VL6180X 量程只有 200mm，桌面边缘垂下去读数会突然跳到
// out_of_range 或者一个很大的值，跟"真的没东西"很难直接区分，
// 所以用连续 N 次确认来过滤。
enum class EdgeState {
    kUnknown = 0,   // 还没校准基线，判断不了
    kSafe = 1,      // 在桌面上
    kEdge = 2,      // 已确认在桌沿
};

struct SensorSnapshot {
    int     distance_mm;   // 距离，毫米；status 非 0 时无意义
    int     status;        // 0=ok 1=out_of_range 2=no_target 3=i2c_error 4=not_ready
    int64_t stamp_ms;      // 采样时刻（esp_timer 毫秒）
    int     baseline_mm;   // 校准出来的桌沿基线，0 表示没校准过
    bool    blocked;       // 安全闸门是否触发
    bool    sensor_ok;     // 传感器当前是否可用
    int     fail_count;    // 连续失败次数
    int     obstacle;      // ObstacleState：0=clear 1=near 2=blocked
    int     edge;          // EdgeState：0=unknown 1=safe 2=edge
    int     edge_confirm;  // 桌沿候选已连续确认几次（0..3）

    // 数据新鲜度。LLM 看到这个值大于 1000 就该说「传感器没有数据」，
    // 不要把几秒前的读数当成现在的距离。
    int age_ms() const;
};

class Vl6180xSensor {
public:
    Vl6180xSensor() = default;
    ~Vl6180xSensor();

    // 探测 + 初始化。失败返回 false，调用方据此决定要不要继续起任务。
    bool Begin(i2c_master_bus_handle_t bus);

    // 起后台采样任务（20 Hz）。Begin() 成功后才调用。
    void Start();

    // 读缓存，不阻塞、不碰 I2C。
    SensorSnapshot Snapshot() const;

    // 校准：连续采 20 次取中位数作为桌沿基线，写入 NVS。
    // 采样期间会先停电机，避免震动干扰。阻塞约 1 秒。
    bool Calibrate(int* baseline_mm_out);

    bool IsOk() const { return sensor_ok_; }

    // 四档阈值由外部（Application 的配置）推过来，不再写成常量。
    // 两个迟滞带，必须满足 blocked < blocked_exit < near < near_exit。
    // 调用方负责保证顺序；这里做兜底：不合法就整组丢弃，保持原值。
    void SetThresholds(int blocked_mm, int blocked_exit_mm,
                       int near_mm, int near_exit_mm);

    // 障碍状态变化回调。参数是 (旧状态, 新状态)，取值见 ObstacleState。
    //
    // 只在"跨越那一刻"调一次，中间在迟滞带里抖动不会重复触发。
    // 传前后两个状态而不是一个布尔，是因为调用方要区分三种跨越：
    //   -> kBlocked       进入，要停车 + 响提示音
    //   kBlocked -> 其它  离开，响解除音
    //   其它 -> 其它      只是过了一档，不响
    // 只给布尔的话，near -> clear 也会被当成"离开"，提示音就白响了。
    //
    // 回调在采样任务上下文里执行，实现方不要在里面做耗时操作——
    // 播报音效这类事情要转投到主循环去。
    void SetObstacleCallback(std::function<void(int old_state, int new_state)> cb) {
        obstacle_cb_ = std::move(cb);
    }

    // 桌沿状态变化回调。参数 true = 刚确认在桌沿，false = 刚回到安全区。
    //
    // 判据是「斜向激光打空」：超量程、无目标、读取超时，三种都算。
    // 正常在桌面上读数稳定在基线附近，这三种不会同时出现，
    // 所以不需要看具体距离，只看 status 就够了。
    //
    // 确认次数只有 2 次（100ms）。8 条腿的底盘从桌沿翻下去也就几百毫秒，
    // 3 次太慢来不及——但 2 次又依赖"偶发噪声不会连续两次"这个前提，
    // 真机上如果误报多，就得改成同时看距离变化。
    void SetEdgeCallback(std::function<void(bool at_edge)> cb) {
        edge_cb_ = std::move(cb);
    }

private:
    static void TaskEntry(void* arg);
    void TaskLoop();

    bool WriteReg(uint16_t reg, uint8_t value);
    bool ReadReg(uint16_t reg, uint8_t* value);
    bool ReadRegs(uint16_t reg, uint8_t* buf, size_t len);
    bool InitRegisters();
    bool ReadOnce(int* distance_mm, int* status);

    void LoadBaseline();
    void SaveBaseline(int baseline_mm);

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_ = nullptr;

    TaskHandle_t task_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<bool> task_exited_{true};

    // 缓存。整块用一个互斥包住，读侧拿到的是同一时刻的一致快照。
    mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
    SensorSnapshot cache_ = {};

    std::atomic<bool> sensor_ok_{false};
    int baseline_mm_ = 0;
    int fail_count_ = 0;

    // 判定层状态。只在采样任务里改，所以不加锁；
    // 对外通过 Snapshot() 的临界区一起拷出去。
    ObstacleState obstacle_ = ObstacleState::kClear;
    EdgeState     edge_ = EdgeState::kUnknown;
    int           edge_confirm_ = 0;
    int64_t       last_valid_ms_ = 0;
    int           last_valid_mm_ = 0;
    // 四档阈值，默认值与原来那几个常量一致
    int th_blocked_mm_      = 80;
    int th_blocked_exit_mm_ = 100;
    int th_near_mm_         = 120;
    int th_near_exit_mm_    = 140;

    std::function<void(int, int)> obstacle_cb_;
    std::function<void(bool)> edge_cb_;
};
