#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <string>
#include <mutex>
#include <deque>
#include <memory>
#include <atomic>
#include <queue>

#include "protocol.h"
#include "ota.h"
#include "audio_service.h"
#include "device_state.h"
#include "device_state_machine.h"
#include "web_server/web_server.h"
#include "peripherals/vl6180x.h"

// Main event bits
#define MAIN_EVENT_SCHEDULE             (1 << 0)
#define MAIN_EVENT_SEND_AUDIO           (1 << 1)
#define MAIN_EVENT_WAKE_WORD_DETECTED   (1 << 2)
#define MAIN_EVENT_VAD_CHANGE           (1 << 3)
#define MAIN_EVENT_ERROR                (1 << 4)
#define MAIN_EVENT_ACTIVATION_DONE      (1 << 5)
#define MAIN_EVENT_CLOCK_TICK           (1 << 6)
#define MAIN_EVENT_NETWORK_CONNECTED    (1 << 7)
#define MAIN_EVENT_NETWORK_DISCONNECTED (1 << 8)
#define MAIN_EVENT_TOGGLE_CHAT          (1 << 9)
#define MAIN_EVENT_START_LISTENING      (1 << 10)
#define MAIN_EVENT_STOP_LISTENING       (1 << 11)
#define MAIN_EVENT_STATE_CHANGED        (1 << 12)


enum AecMode {
    kAecOff,
    kAecOnDeviceSide,
    kAecOnServerSide,
};

// 第 2 步：屏幕显示模式
// kDisplayModeDefault —— 官方原样：状态栏 + 表情图标 + 聊天字幕。只在开机
//                        （starting / activating）阶段出现，用户切不到它。
// kDisplayModeEyeOnly —— RoboEyes 独占整屏画动画眼睛（会 lvgl_port_stop）
// kDisplayModeSleep   —— 整屏大字时钟，设备待机/睡眠时的默认样子
//
// 注意 kDisplayModeDefault 必须留 0：Initialize() 里第一次渲染用的就是它，
// 开机那 9.6 秒要显示"正在配网""激活中"这些状态文本。
enum DisplayMode {
    kDisplayModeDefault = 0,
    kDisplayModeEyeOnly,
    kDisplayModeSleep,
    // 音乐界面。不进 NVS：它是临时状态，下次开机不该直接停在歌名上。
    // 退出时回到进音乐之前那一屏，靠 display_mode_before_music_ 记住。
    kDisplayModeMusic,
};

// 第 10 步：机器人四模式。
// kRobotExplore   —— 随机大动作，不需要传感器
// kRobotFree      —— 原地做表情，不驱动轮子，不需要传感器
// kRobotFollow    —— 跟着人走，>10cm 追 / 5-10cm 保持 / <5cm 后退
// kRobotEdgePatrol—— 直行、判到桌沿就退 15cm 转 90°，继续
//
// follow 和 edge_patrol 靠距离驱动，传感器不可用时 start() 直接拒绝，
// 不让它瞎跑。explore 和 free 不碰传感器，任何情况下都能跑。
enum RobotMode {
    kRobotNone = 0,
    kRobotExplore,
    kRobotFree,
    kRobotFollow,
    kRobotEdgePatrol,
};

class Application {
public:
    static Application& GetInstance() {
        static Application instance;
        return instance;
    }
    // Delete copy constructor and assignment operator
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    /**
     * Initialize the application
     * This sets up display, audio, network callbacks, etc.
     * Network connection starts asynchronously.
     */
    void Initialize();

    /**
     * Run the main event loop
     * This function runs in the main task and never returns.
     * It handles all events including network, state changes, and user interactions.
     */
    void Run();

    DeviceState GetDeviceState() const { return state_machine_.GetState(); }
    bool IsVoiceDetected() const { return audio_service_.IsVoiceDetected(); }
    
    /**
     * Request state transition
     * Returns true if transition was successful
     */
    bool SetDeviceState(DeviceState state);

    /**
     * Schedule a callback to be executed in the main task
     */
    void Schedule(std::function<void()>&& callback);

    /**
     * Alert with status, message, emotion and optional sound
     */
    void Alert(const char* status, const char* message, const char* emotion = "", const std::string_view& sound = "");
    void DismissAlert();

    void AbortSpeaking(AbortReason reason);

    /**
     * Toggle chat state (event-based, thread-safe)
     * Sends MAIN_EVENT_TOGGLE_CHAT to be handled in Run()
     */
    void ToggleChatState();

    /**
     * Start listening (event-based, thread-safe)
     * Sends MAIN_EVENT_START_LISTENING to be handled in Run()
     */
    void StartListening();

    /**
     * Stop listening (event-based, thread-safe)
     * Sends MAIN_EVENT_STOP_LISTENING to be handled in Run()
     */
    void StopListening();

    void Reboot();
    void WakeWordInvoke(const std::string& wake_word);
    bool UpgradeFirmware(const std::string& url, const std::string& version = "");
    bool CanEnterSleepMode();
    void SendMcpMessage(const std::string& payload);
    void SetAecMode(AecMode mode);
    AecMode GetAecMode() const { return aec_mode_; }
    void PlaySound(const std::string_view& sound);
    AudioService& GetAudioService() { return audio_service_; }

    // 第 10 步：机器人模式
    // 返回值是错误说明，空串表示启动成功。这样工具层能直接把原因念给用户听。
    std::string StartRobotMode(int mode);
    void StopRobotMode();
    int GetRobotMode() const { return robot_mode_.load(); }
    static const char* RobotModeName(int mode);

    // 第 2 步：屏幕显示模式
    DisplayMode GetDisplayMode() const { return display_mode_; }

    // 7b：静音状态。-1 表示没静音，>=0 表示静音前的音量。
    int GetVolumeBeforeMute() const { return volume_before_mute_; }
    void SetVolumeBeforeMute(int v) { volume_before_mute_ = v; }
    void SetDisplayMode(DisplayMode mode);
    // 从 NVS 恢复上次停在哪一屏（激活完成后调用）
    void RestoreDisplayMode();
    // 测距。传感器由板卡创建并 Attach 进来，Application 只持有裸指针，
    // 生命周期由板卡管理（板卡对象活到进程结束）。
    // 板卡把传感器交过来。这里顺便挂上障碍跨越回调，
    // 让 5cm / 10cm 两个提示音在跨阈值时响。
    void AttachSensor(Vl6180xSensor* sensor);
    SensorSnapshot GetSensorSnapshot() const;
    bool CalibrateSensor(int* baseline_mm_out);

    void ToggleDisplayMode();

    // 给串口终端用的手动驱动入口。SetRealtimeMotorCommand 本身是 private，
    // 但终端要能在不烧固件的前提下单独验电机，所以开这两个薄包装，
    // 内部继续走 from_web=true（绕开传感器闸门，和 Web 遥控器同一条路）。
    void DebugMotorCommand(int direction, int speed);
    void DebugMotorStop();

    // === 第 6 步：显示状态机（睡眠 / 唤醒）===
    // Sleep  = 整屏大字时钟，60 秒没交互自动进
    // Eyes   = RoboEyes 眼睛 + 底部字幕（第 7 步把字幕开出来）
    static constexpr int kIdleToSleepSeconds = 60;
    void EnterSleep();          // 主动进睡眠：切时钟 + 降频
    void WakeUp();              // 从睡眠唤醒：切眼睛 + 恢复性能
    void ResetIdleTimer();      // 有交互就重置 60 秒倒计时
    bool IsBusyWithTask();      // 正在听/说/连/升级——这些状态下不许睡

    // === 电机控制（第 1 步，来自 2.1.0 qebabe-xiaoche）===

    // 各动作的默认时长，全部可通过 NVS "motor_config" 命名空间改写
    struct MotorActionConfig {
        int forward_duration_ms = 5000;
        int backward_duration_ms = 5000;
        int left_turn_duration_ms = 600;
        int right_turn_duration_ms = 600;
        int spin_duration_ms = 2500;
        int wiggle_duration_ms = 600;
        int dance_duration_ms = 1500;
        int quick_forward_duration_ms = 5000;
        int quick_backward_duration_ms = 5000;
        int default_speed_percent = 100;
        // 跟随模式的两个距离阈值。电池掉电压时电机带不动，会把速度调高，
        // 阈值跟着要能动，不然一直是 10cm 前进 / 5cm 后退那一套。
        int follow_far_mm = 100;    // 超过这个距离才追
        int follow_near_mm = 50;    // 小于这个距离往后退

        // 机器人模式的速度。电池电压掉下来时同样占空比的扭矩会变小，
        // 表现为「该动的时候不动」，把这两个值往上提比调距离更直接。
        int follow_speed = 55;      // 跟随：追进和后退都用它
        int patrol_speed = 55;      // 巡检：直行、后退、转向都用它

        // 测距四档迟滞阈值（毫米）。两个迟滞带，必须满足
        // 停车 < 停车解除 < 减速 < 减速解除，否则判定会自相矛盾。
        int th_blocked_mm = 80;       // 近于这个，停车并响 near5cm
        int th_blocked_exit_mm = 100; // 远于这个，解除停车
        int th_near_mm = 120;         // 近于这个，限速
        int th_near_exit_mm = 140;    // 远于这个，解除限速
    };

    void LoadMotorActionConfig();
    void SaveMotorActionConfig();
    const MotorActionConfig& GetMotorActionConfig() const { return motor_action_config_; }
    void SetMotorActionConfig(const MotorActionConfig& config);

    // direction: 0=停 1=右 2=后 3=左 4=前
    void TriggerMotorEmotion(int emotion_type);
    void HandleMotorActionWithDuration(int direction, int speed, int duration_ms, int priority = 1);
    void QueueMotorAction(int direction, int speed, int duration_ms, const std::string& description);
    void ExecuteMotorActionQueue();

    // === 网页遥控（第 3 步）===
    // direction: 0=停 1=右 2=后 3=左 4=前；speed: 0-100
    // 摇杆推着走用的是实时控制（直接改 PWM 占空比，不入队），松手时 speed=0 停。
    void HandleWebMotorControl(int direction, int speed);
    void StartWebServer();
    
    /**
     * Reset protocol resources (thread-safe)
     * Can be called from any task to release resources allocated after network connected
     * This includes closing audio channel, resetting protocol and ota objects
     */
    void ResetProtocol();

private:
    Application();
    ~Application();

    std::mutex mutex_;
    std::deque<std::function<void()>> main_tasks_;
    std::unique_ptr<Protocol> protocol_;
    EventGroupHandle_t event_group_ = nullptr;
    esp_timer_handle_t clock_timer_handle_ = nullptr;
    DeviceStateMachine state_machine_;
    ListeningMode listening_mode_ = kListeningModeAutoStop;

    // TTS 兜底：服务端偶尔不发 tts stop，设备会永远停在 speaking。
    // 记「播放队列开始空」的时刻，空满 3 秒就强制回聆听。
    int64_t tts_idle_since_ms_ = 0;

    // 只有收到 % play_music 之后，下一条《歌名》才算歌名。
    // 否则服务端回答"有这几首：《A》《B》"时也会被误判成进音乐界面。
    bool awaiting_music_title_ = false;
    AecMode aec_mode_ = kAecOff;
    std::string last_error_message_;
    AudioService audio_service_;
    std::unique_ptr<Ota> ota_;

    bool has_server_time_ = false;
    bool aborted_ = false;
    bool assets_version_checked_ = false;
    bool play_popup_on_listening_ = false;  // Flag to play popup sound after state changes to listening
    int clock_ticks_ = 0;
    TaskHandle_t activation_task_handle_ = nullptr;

    // === 网页遥控（第 3 步）===
    // 只在 HandleActivationDoneEvent 之后创建并启动。
    std::unique_ptr<WebServer> web_server_;

    // === 电机控制（第 1 步）===
    MotorActionConfig motor_action_config_;

    // 第 2 步：显示模式。初始值保持 Default，让 Initialize() 里那次
    // SetDisplayMode(kDisplayModeEyeOnly) 能真正走进函数体（2.1.0 里
    // 先赋值再调用，导致 if (display_mode_ != mode) 判假、整段跳过）。
    DisplayMode display_mode_ = kDisplayModeDefault;
    // 进音乐之前停在哪一屏。退出音乐时原样恢复，不擅自改用户的界面选择。
    DisplayMode display_mode_before_music_ = kDisplayModeDefault;
    // 静音前的音量。-1 表示当前没静音。不能拿 0 当静音标记：
    // audio_codec.cc 看到音量 <= 0 会警告并强制改成 10。
    int volume_before_mute_ = -1;
    Vl6180xSensor* sensor_ = nullptr;
    // 上一句推断出来的语气。相同就不重复 SetEmotion——
    // SetEmotion 内部会先关掉 idle/curiosity/flicker 再重设，
    // 每句都调一次会把正在跑的小动画打断，看起来反而更僵。
    std::string last_sentence_tone_;

    // 第 6 步：60 秒无交互进时钟的定时器。
    // 用 esp_timer 而不是 FreeRTOS 定时器，因为回调里要调 esp_pm_configure
    // 和 display 方法，esp_timer 的任务优先级比 IDLE 高，不会被饿死。
    esp_timer_handle_t idle_timer_ = nullptr;
    static void IdleTimerCallback(void* arg);

    // 实时控制标志：为 true 时表示有动作正在跑，新动作要按优先级比较
    std::atomic<bool> realtime_control_active_{false};
    // 最近一次电机指令是不是 Web 遥控器发的。
    // 传感器触发保护性停车时要看这个：Web 是手动模式，不被自动停。
    std::atomic<bool> last_command_from_web_{false};
    std::atomic<int> current_motor_priority_{0};
    std::atomic<int64_t> last_realtime_command_ms_{0};

    std::mutex motor_gpio_init_mutex_;
    bool motor_gpio_initialized_ = false;
    bool motor_pwm_initialized_ = false;

    int pwm_freq_hz_ = 20000;
    int pwm_resolution_bits_ = 10;
    int pwm_ramp_ms_ = 50;

    // from_web=true 时跳过传感器安全闸门。
    // Web 遥控器是手动模式，要能强行开，否则传感器一坏连遥控也废了。
    void SetRealtimeMotorCommand(int direction, int speed, bool from_web = false);

    // 传感器不可用时，前进/后退已经从哪一刻开始算。0 表示没在走。
    // 降级策略是"前后最多 1 秒"，实时控制没有 duration，只能自己计时。
    int64_t degraded_forward_start_ms_ = 0;

    // 上一次真正下发到电机的实时指令。
    //
    // 实时控制是"持续指令"，调用方每 100~150ms 会重复下发同一个方向和速度。
    // 而 SetRealtimeMotorCommand 每次都要先把四路 PWM 缓降到 0 再缓升到目标值，
    // 高频重设会让电机一直在 0 和目标值之间震荡，表现就是"给了指令但轮子不动"。
    // 记住上一次的值，没变化就直接返回。
    int last_realtime_direction_ = -1;
    int last_realtime_speed_ = -1;
    void StopRealtimeMotorControl();
    void InitMotorPwm();

    // 第 10 步：机器人模式
    // 一个任务吃下四个子模式，靠 robot_mode_ 分派。不给每种模式单开任务——
    // 同一时刻只有一个模式在跑，多开任务只会多占栈。
    std::atomic<int> robot_mode_{kRobotNone};
    std::atomic<bool> robot_task_running_{false};
    // 语音唤醒/说话时置 true，模式任务看到就停手（停音效、停大幅动作），
    // 但不退出模式；聊完自动继续。
    std::atomic<bool> robot_paused_{false};
    // 暂停期间机器人可能被挪动，恢复时要先原地校准一次。
    std::atomic<bool> robot_need_recalibrate_{false};

    // explore 上一个方向码，用来禁止连续走反方向。
    // 纯随机时「前→后→前」这种组合出现得不算少，看着像原地晃。
    int last_explore_dir_ = 0;
    // 跟随模式的目标丢失扫描。VL6180X 是单点测距，只知道"正前方多远"，
    // 目标侧移出视场后读不到任何有效距离，只能原地转着找。
    int follow_lost_count_ = 0;
    int64_t follow_search_start_ms_ = 0;
    TaskHandle_t robot_task_ = nullptr;
    static void RobotModeTaskEntry(void* arg);
    void RobotModeLoop();
    void RunExploreStep();
    void RunFreeStep();
    void RunFollowStep();
    void RunEdgePatrolStep();
    // 从对话回到模式时先做一次原地动作校准位置：
    // 唤醒期间机器人可能被人挪过，直接接着跑会撞墙。
    void RunRecalibrateStep();


    // Event handlers
    void HandleStateChangedEvent();
    void HandleToggleChatEvent();
    void HandleStartListeningEvent();
    void HandleStopListeningEvent();
    void HandleNetworkConnectedEvent();
    void HandleNetworkDisconnectedEvent();
    void HandleActivationDoneEvent();
    void HandleWakeWordDetectedEvent();
    void ContinueOpenAudioChannel(ListeningMode mode);
    void ContinueWakeWordInvoke(const std::string& wake_word);

    // Activation task (runs in background)
    void ActivationTask();

    // Helper methods
    void CheckAssetsVersion();
    void CheckNewVersion();
    void InitializeProtocol();
    void ShowActivationCode(const std::string& code, const std::string& message);
    void SetListeningMode(ListeningMode mode);
    ListeningMode GetDefaultListeningMode() const;
    
    // State change handler called by state machine
    void OnStateChanged(DeviceState old_state, DeviceState new_state);
};


class TaskPriorityReset {
public:
    TaskPriorityReset(BaseType_t priority) {
        original_priority_ = uxTaskPriorityGet(NULL);
        vTaskPrioritySet(NULL, priority);
    }
    ~TaskPriorityReset() {
        vTaskPrioritySet(NULL, original_priority_);
    }

private:
    BaseType_t original_priority_;
};

#endif // _APPLICATION_H_
