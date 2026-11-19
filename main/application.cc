#include "application.h"
#include "board.h"
#include "display.h"
#include "system_info.h"
#include "audio_codec.h"
#include "mqtt_protocol.h"
#include "websocket_protocol.h"
#include "assets/lang_config.h"
#include "mcp_server.h"
#include "assets.h"
#include "settings.h"

#include <cstring>
#include <esp_log.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <arpa/inet.h>
#include <esp_random.h>   // 第 10 步 explore 模式随机挑方向/速度/时长
#include <font_awesome.h>

#define TAG "Application"

// === 电机控制（第 1 步，来自 2.1.0 qebabe-xiaoche）===
#include <driver/ledc.h>
#include <tuple>
#include <wifi_manager.h>   // WifiManager::GetInstance().GetIpAddress()，Web 控制显示访问地址用
#include "console/console.h"   // 串口服务终端（UART0，和日志共用）

// 板卡引脚。CMakeLists 的 INCLUDE_DIRS 不含 boards/<BOARD_TYPE>，所以 application.cc
// 拿不到板卡 config.h，只能在这里兜底。这几个值必须和
// boards/deepseeking-robot/config.h 里的 MOTOR_*_GPIO 保持一致。
#ifndef MOTOR_LF_GPIO
#define MOTOR_LF_GPIO GPIO_NUM_12   // 左前，接 DRV8833 AIN1
#define MOTOR_LB_GPIO GPIO_NUM_13   // 左后，接 DRV8833 AIN2
#define MOTOR_RF_GPIO GPIO_NUM_14   // 右前，接 DRV8833 BIN1
#define MOTOR_RB_GPIO GPIO_NUM_21   // 右后，接 DRV8833 BIN2
#endif

// 动作队列：所有电机动作串行执行，避免并发抢 PWM
struct MotorAction {
    int direction;
    int speed;
    int duration_ms;
    std::string description;
};

static std::queue<MotorAction> motor_action_queue_;
static std::mutex motor_queue_mutex_;
static volatile bool motor_executor_running_ = false;



// 从一句话的文本推断"语气"，只覆盖语气层面：
//   笑 → laughing、疑问 → thinking、惊叹 → surprised、反问 → confused
// 情感层面（难过、生气、亲昵）不动，那部分交给服务端 llm 消息里的 emotion，
// 因为关键词法判断情绪不可靠，判断语气却够用。
//
// 注意：只认明确的信号，认不出来返回 nullptr，调用方就不切表情，
// 让当前表情自然延续下去，避免每句都强行变脸。
static const char* EmotionFromSentence(const char* text) {
    if (text == nullptr || text[0] == '\0') {
        return nullptr;
    }

    // 笑。放最前面：一句话里带"哈哈"通常整句都是笑的语气。
    if (strstr(text, "哈哈") != nullptr || strstr(text, "嘿嘿") != nullptr ||
        strstr(text, "嘻嘻") != nullptr || strstr(text, "笑死") != nullptr) {
        return "laughing";
    }
    // 疑问词在前半句 → 困惑
    if (strstr(text, "为什么") != nullptr || strstr(text, "怎么会") != nullptr ||
        strstr(text, "咦") != nullptr) {
        return "confused";
    }
    // 惊叹
    if (strstr(text, "太好了") != nullptr || strstr(text, "真的吗") != nullptr ||
        strstr(text, "哇") != nullptr || strstr(text, "天哪") != nullptr) {
        return "surprised";
    }

    // 句尾标点。UTF-8 里全角标点占 3 字节，问号感叹号各查一次尾部。
    size_t len = strlen(text);
    if (len >= 3) {
        const char* tail = text + len - 3;
        if (strcmp(tail, "？") == 0) {
            return "thinking";
        }
        if (strcmp(tail, "！") == 0) {
            return "happy";
        }
    }
    if (len >= 1 && text[len - 1] == '?') {
        return "thinking";
    }
    if (len >= 1 && text[len - 1] == '!') {
        return "happy";
    }
    return nullptr;
}

Application::Application() {
    event_group_ = xEventGroupCreate();

#if CONFIG_USE_DEVICE_AEC && CONFIG_USE_SERVER_AEC
#error "CONFIG_USE_DEVICE_AEC and CONFIG_USE_SERVER_AEC cannot be enabled at the same time"
#elif CONFIG_USE_DEVICE_AEC
    aec_mode_ = kAecOnDeviceSide;
#elif CONFIG_USE_SERVER_AEC
    aec_mode_ = kAecOnServerSide;
#else
    aec_mode_ = kAecOff;
#endif

    esp_timer_create_args_t clock_timer_args = {
        .callback = [](void* arg) {
            Application* app = (Application*)arg;
            xEventGroupSetBits(app->event_group_, MAIN_EVENT_CLOCK_TICK);
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "clock_timer",
        .skip_unhandled_events = true
    };
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
}

Application::~Application() {
    if (clock_timer_handle_ != nullptr) {
        esp_timer_stop(clock_timer_handle_);
        esp_timer_delete(clock_timer_handle_);
    }
    vEventGroupDelete(event_group_);
}

bool Application::SetDeviceState(DeviceState state) {
    return state_machine_.TransitionTo(state);
}

void Application::Initialize() {
    auto& board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

    // Setup the display
    auto display = board.GetDisplay();
    display->SetupUI();
    // Print board name/version info
    display->SetChatMessage("system", SystemInfo::GetUserAgent().c_str());

    // 第 2 步：这里**不**立刻切眼睛模式。
    // 开机头几秒要留给小智自己的信息显示（版本号、激活状态、WiFi 连接过程），
    // 切到眼睛模式会 lvgl_port_stop() 把整个 LVGL 停掉，那些字就看不见了。
    // 真正的切换点在 HandleStateChangedEvent() 的 kDeviceStateIdle 分支——
    // 也就是激活完成、真正进入待机之后。

    // Setup the audio service
    auto codec = board.GetAudioCodec();
    audio_service_.Initialize(codec);
    audio_service_.Start();

    AudioServiceCallbacks callbacks;
    callbacks.on_send_queue_available = [this]() {
        xEventGroupSetBits(event_group_, MAIN_EVENT_SEND_AUDIO);
    };
    callbacks.on_wake_word_detected = [this](const std::string& wake_word) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_WAKE_WORD_DETECTED);
    };
    callbacks.on_vad_change = [this](bool speaking) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_VAD_CHANGE);
    };
    audio_service_.SetCallbacks(callbacks);

    // 开机音效：从 v206 固件里提取的 0008F191（11.7 KB，16 kHz 单声道 Opus，
    // 与官方 exclamation/low_battery 等同一规格）。用一次性任务异步播放，
    // 因为 PlaySound() 内部是同步逐包入队，直接在 Initialize() 里调会阻塞好几秒。
    xTaskCreate([](void* arg) {
        vTaskDelay(pdMS_TO_TICKS(800));   // 等 codec 与 I2S 通道稳定
        static_cast<AudioService*>(arg)->PlaySound(Lang::Sounds::OGG_STARTUP);
        vTaskDelete(nullptr);
    }, "startup_sound", 6144, &audio_service_, tskIDLE_PRIORITY + 1, nullptr);

    // Add state change listeners
    state_machine_.AddStateChangeListener([this](DeviceState old_state, DeviceState new_state) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_STATE_CHANGED);
    });

    // Start the clock timer to update the status bar
    esp_timer_start_periodic(clock_timer_handle_, 1000000);

    // Add MCP common tools (only once during initialization)
    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddCommonTools();
    mcp_server.AddUserOnlyTools();

    // 第 1 步：从 NVS 读电机动作时长（没有就用默认值）
    LoadMotorActionConfig();

    // 第 6 步：60 秒无交互进时钟。放在这里而不是构造函数里，
    // 是因为回调要用到 this 且此时 Board::GetInstance() 已经可用。
    if (idle_timer_ == nullptr) {
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = &Application::IdleTimerCallback;
        timer_args.arg      = this;
        timer_args.name     = "idle_timer";
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &idle_timer_));
        ESP_LOGI(TAG, "Idle timer created (%d s)", (int)kIdleToSleepSeconds);
    }

        // 串口服务终端。挂在 UART0（和日志同一个口），自带行编辑和历史。
        // 启动一次就够，重复调用会多注册一遍命令，所以和 idle timer 一起判空。
        static bool console_started = false;
        if (!console_started) {
            console_started = true;
            console_start();
        }


    // Set network event callback for UI updates and network state handling
    board.SetNetworkEventCallback([this](NetworkEvent event, const std::string& data) {
        auto display = Board::GetInstance().GetDisplay();
        
        switch (event) {
            case NetworkEvent::Scanning:
                display->ShowNotification(Lang::Strings::SCANNING_WIFI, 30000);
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_DISCONNECTED);
                break;
            case NetworkEvent::Connecting: {
                if (data.empty()) {
                    // Cellular network - registering without carrier info yet
                    display->SetStatus(Lang::Strings::REGISTERING_NETWORK);
                } else {
                    // WiFi or cellular with carrier info
                    std::string msg = Lang::Strings::CONNECT_TO;
                    msg += data;
                    msg += "...";
                    display->ShowNotification(msg.c_str(), 30000);
                }
                break;
            }
            case NetworkEvent::Connected: {
                std::string msg = Lang::Strings::CONNECTED_TO;
                msg += data;
                display->ShowNotification(msg.c_str(), 30000);
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_CONNECTED);
                break;
            }
            case NetworkEvent::Disconnected:
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_DISCONNECTED);
                break;
            case NetworkEvent::WifiConfigModeEnter:
                // WiFi config mode enter is handled by WifiBoard internally
                break;
            case NetworkEvent::WifiConfigModeExit:
                // WiFi config mode exit is handled by WifiBoard internally
                break;
            // Cellular modem specific events
            case NetworkEvent::ModemDetecting:
                display->SetStatus(Lang::Strings::DETECTING_MODULE);
                break;
            case NetworkEvent::ModemErrorNoSim:
                Alert(Lang::Strings::ERROR, Lang::Strings::PIN_ERROR, "triangle_exclamation", Lang::Sounds::OGG_ERR_PIN);
                break;
            case NetworkEvent::ModemErrorRegDenied:
                Alert(Lang::Strings::ERROR, Lang::Strings::REG_ERROR, "triangle_exclamation", Lang::Sounds::OGG_ERR_REG);
                break;
            case NetworkEvent::ModemErrorInitFailed:
                Alert(Lang::Strings::ERROR, Lang::Strings::MODEM_INIT_ERROR, "triangle_exclamation", Lang::Sounds::OGG_EXCLAMATION);
                break;
            case NetworkEvent::ModemErrorTimeout:
                display->SetStatus(Lang::Strings::REGISTERING_NETWORK);
                break;
        }
    });

    // Start network asynchronously
    board.StartNetwork();

    // Update the status bar immediately to show the network state
    display->UpdateStatusBar(true);
}

void Application::Run() {
    // Set the priority of the main task to 10
    vTaskPrioritySet(nullptr, 10);

    const EventBits_t ALL_EVENTS = 
        MAIN_EVENT_SCHEDULE |
        MAIN_EVENT_SEND_AUDIO |
        MAIN_EVENT_WAKE_WORD_DETECTED |
        MAIN_EVENT_VAD_CHANGE |
        MAIN_EVENT_CLOCK_TICK |
        MAIN_EVENT_ERROR |
        MAIN_EVENT_NETWORK_CONNECTED |
        MAIN_EVENT_NETWORK_DISCONNECTED |
        MAIN_EVENT_TOGGLE_CHAT |
        MAIN_EVENT_START_LISTENING |
        MAIN_EVENT_STOP_LISTENING |
        MAIN_EVENT_ACTIVATION_DONE |
        MAIN_EVENT_STATE_CHANGED;

    while (true) {
        auto bits = xEventGroupWaitBits(event_group_, ALL_EVENTS, pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & MAIN_EVENT_ERROR) {
            SetDeviceState(kDeviceStateIdle);
            Alert(Lang::Strings::ERROR, last_error_message_.c_str(), "circle_xmark", Lang::Sounds::OGG_EXCLAMATION);
        }

        if (bits & MAIN_EVENT_NETWORK_CONNECTED) {
            HandleNetworkConnectedEvent();
        }

        if (bits & MAIN_EVENT_NETWORK_DISCONNECTED) {
            HandleNetworkDisconnectedEvent();
        }

        if (bits & MAIN_EVENT_ACTIVATION_DONE) {
            HandleActivationDoneEvent();
        }

        if (bits & MAIN_EVENT_STATE_CHANGED) {
            HandleStateChangedEvent();
        }

        if (bits & MAIN_EVENT_TOGGLE_CHAT) {
            HandleToggleChatEvent();
        }

        if (bits & MAIN_EVENT_START_LISTENING) {
            HandleStartListeningEvent();
        }

        if (bits & MAIN_EVENT_STOP_LISTENING) {
            HandleStopListeningEvent();
        }

        if (bits & MAIN_EVENT_SEND_AUDIO) {
            while (auto packet = audio_service_.PopPacketFromSendQueue()) {
                if (protocol_ && !protocol_->SendAudio(std::move(packet))) {
                    break;
                }
            }
        }

        if (bits & MAIN_EVENT_WAKE_WORD_DETECTED) {
            HandleWakeWordDetectedEvent();
        }

        if (bits & MAIN_EVENT_VAD_CHANGE) {
            if (GetDeviceState() == kDeviceStateListening) {
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            }
        }

        if (bits & MAIN_EVENT_SCHEDULE) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }

        if (bits & MAIN_EVENT_CLOCK_TICK) {
            clock_ticks_++;
            auto display = Board::GetInstance().GetDisplay();
            // 第 6 步：睡眠态每秒只刷时钟文本；其它态还是刷状态栏。
            if (display_mode_ == kDisplayModeSleep) {
                display->ShowClock();
            } else {
                display->UpdateStatusBar();
            }

            // TTS 兜底：服务端偶尔不发 tts stop，设备会永远停在 speaking，
            // 表现为「一直在说话、听不见唤醒词」。
            //
            // 判据（2026-11-18 改）：看「距最后一个 TTS 音频帧多久」，
            // 不再看「队列空多久」。服务端换句、跑 MCP 工具时队列本来就是空的，
            // 按队列判会在 3 秒时误切走；一旦切走，OnIncomingAudio 的状态门禁
            // 会把后续音频整段丢掉 —— 用户听到的就是「话说一半突然没了」。
            //
            // 两级：队列空 6 秒记一条日志（只诊断，不动状态）；
            //       距最后一个音频帧满 15 秒才判定卡死，回 idle 而不是 listening，
            //       因为回 listening 会下发 start listening，等于主动打断服务端。
            {
                constexpr int64_t kIdleLogMs = 6000;
                constexpr int64_t kStuckMs = 15000;

                if (GetDeviceState() == kDeviceStateSpeaking) {
                    int64_t now_ms = esp_timer_get_time() / 1000;
                    if (audio_service_.IsIdle()) {
                        if (tts_idle_since_ms_ == 0) {
                            tts_idle_since_ms_ = now_ms;
                        }
                        if (!tts_idle_logged_ && now_ms - tts_idle_since_ms_ >= kIdleLogMs) {
                            int64_t last_audio = last_tts_audio_ms_.load();
                            int64_t since_audio = (last_audio == 0) ? -1 : (now_ms - last_audio);
                            ESP_LOGW(TAG, "tts: queue empty %d ms, last audio %d ms ago",
                                     (int)(now_ms - tts_idle_since_ms_), (int)since_audio);
                            tts_idle_logged_ = true;
                        }

                        int64_t last_audio = last_tts_audio_ms_.load();
                        if (last_audio != 0 && now_ms - last_audio >= kStuckMs) {
                            ESP_LOGW(TAG, "tts: no audio frame for %d ms, forcing idle",
                                     (int)(now_ms - last_audio));
                            tts_idle_since_ms_ = 0;
                            tts_idle_logged_ = false;
                            last_tts_audio_ms_.store(0);
                            SetDeviceState(kDeviceStateIdle);
                        }
                    } else {
                        tts_idle_since_ms_ = 0;
                        tts_idle_logged_ = false;
                    }
                } else {
                    tts_idle_since_ms_ = 0;
                    tts_idle_logged_ = false;
                }
            }

            // Print debug info every 10 seconds
            if (clock_ticks_ % 10 == 0) {
                SystemInfo::PrintHeapStats();
            }
        }
    }
}

void Application::HandleNetworkConnectedEvent() {
    ESP_LOGI(TAG, "Network connected");
    auto state = GetDeviceState();

    if (state == kDeviceStateStarting || state == kDeviceStateWifiConfiguring) {
        // Network is ready, start activation
        SetDeviceState(kDeviceStateActivating);
        if (activation_task_handle_ != nullptr) {
            ESP_LOGW(TAG, "Activation task already running");
            return;
        }

        xTaskCreate([](void* arg) {
            Application* app = static_cast<Application*>(arg);
            app->ActivationTask();
            app->activation_task_handle_ = nullptr;
            vTaskDelete(NULL);
        }, "activation", 4096 * 2, this, 2, &activation_task_handle_);
    }

    // Update the status bar immediately to show the network state
    auto display = Board::GetInstance().GetDisplay();
    display->UpdateStatusBar(true);
}

void Application::HandleNetworkDisconnectedEvent() {
    // Close current conversation when network disconnected
    auto state = GetDeviceState();
    if (state == kDeviceStateConnecting || state == kDeviceStateListening || state == kDeviceStateSpeaking) {
        ESP_LOGI(TAG, "Closing audio channel due to network disconnection");
        protocol_->CloseAudioChannel();
    }

    // Update the status bar immediately to show the network state
    auto display = Board::GetInstance().GetDisplay();
    display->UpdateStatusBar(true);
}

void Application::HandleActivationDoneEvent() {
    ESP_LOGI(TAG, "Activation done");

    SystemInfo::PrintHeapStats();
    SetDeviceState(kDeviceStateIdle);

    has_server_time_ = ota_->HasServerTime();

    auto display = Board::GetInstance().GetDisplay();
    std::string message = std::string(Lang::Strings::VERSION) + ota_->GetCurrentVersion();
    display->ShowNotification(message.c_str());
    display->SetChatMessage("system", "");

    // Release OTA object after activation is complete
    ota_.reset();
    auto& board = Board::GetInstance();
    board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);

    Schedule([this]() {
        // 接入平台完成的提示音。原来播的是官方的 success.ogg（两声短促的「嘀」），
        // 换成从 v206 固件里提取的 00098F8D（4.4 KB，16 kHz 单声道 Opus，
        // 与 startup.ogg 同一规格），也就是探索模式那组机器人音效里的一个。
        audio_service_.PlaySound(Lang::Sounds::OGG_READY);
    });

    StartWebServer();

    // 恢复上次停在哪一屏（眼睛或官方界面）。这一步必须在激活完成之后，
    // 早于它会停掉 LVGL，开机过程的状态字就看不见了。
    RestoreDisplayMode();
}

void Application::StartWebServer() {
    // 网页遥控只在激活完成、拿到正常网络之后才起。
    // 起在 Initialize() 里的话，配网阶段 httpd 会跟 WiFi 热点抢资源，
    // 而且那时 80 端口还没被 DHCP 分配好。
    if (web_server_) {
        return;   // 已经起过（激活只会走一次，这里防御性判断）
    }
    web_server_ = std::make_unique<WebServer>();

    web_server_->SetMotorControlCallback([this](int direction, int speed) {
        HandleWebMotorControl(direction, speed);
    });

    web_server_->SetEmotionCallback([](const char* emotion) {
        Board::GetInstance().GetDisplay()->SetEmotion(emotion);
    });

    web_server_->SetMotorActionConfigCallback(
        [this]() -> WebServer::MotorActionConfig {
            const auto& config = GetMotorActionConfig();
            WebServer::MotorActionConfig ws;
            ws.forward_duration_ms        = config.forward_duration_ms;
            ws.backward_duration_ms       = config.backward_duration_ms;
            ws.left_turn_duration_ms      = config.left_turn_duration_ms;
            ws.right_turn_duration_ms     = config.right_turn_duration_ms;
            ws.spin_duration_ms           = config.spin_duration_ms;
            ws.wiggle_duration_ms         = config.wiggle_duration_ms;
            ws.dance_duration_ms          = config.dance_duration_ms;
            ws.quick_forward_duration_ms  = config.quick_forward_duration_ms;
            ws.quick_backward_duration_ms = config.quick_backward_duration_ms;
            ws.default_speed_percent      = config.default_speed_percent;
            ws.follow_far_mm              = config.follow_far_mm;
            ws.follow_near_mm             = config.follow_near_mm;
            ws.follow_speed               = config.follow_speed;
            ws.patrol_speed               = config.patrol_speed;
            ws.th_blocked_mm              = config.th_blocked_mm;
            ws.th_blocked_exit_mm         = config.th_blocked_exit_mm;
            ws.th_near_mm                 = config.th_near_mm;
            ws.th_near_exit_mm            = config.th_near_exit_mm;
            return ws;
        },
        [this](const WebServer::MotorActionConfig& ws) {
            MotorActionConfig config;
            config.forward_duration_ms        = ws.forward_duration_ms;
            config.backward_duration_ms       = ws.backward_duration_ms;
            config.left_turn_duration_ms      = ws.left_turn_duration_ms;
            config.right_turn_duration_ms     = ws.right_turn_duration_ms;
            config.spin_duration_ms           = ws.spin_duration_ms;
            config.wiggle_duration_ms         = ws.wiggle_duration_ms;
            config.dance_duration_ms          = ws.dance_duration_ms;
            config.quick_forward_duration_ms  = ws.quick_forward_duration_ms;
            config.quick_backward_duration_ms = ws.quick_backward_duration_ms;
            config.default_speed_percent      = ws.default_speed_percent;
            config.follow_far_mm              = ws.follow_far_mm;
            config.follow_near_mm             = ws.follow_near_mm;
            config.follow_speed               = ws.follow_speed;
            config.patrol_speed               = ws.patrol_speed;
            config.th_blocked_mm              = ws.th_blocked_mm;
            config.th_blocked_exit_mm         = ws.th_blocked_exit_mm;
            config.th_near_mm                 = ws.th_near_mm;
            config.th_near_exit_mm            = ws.th_near_exit_mm;
            SetMotorActionConfig(config);

            // 阈值同时推给传感器，判定在采样任务里做，不是每次读快照再算
            if (sensor_ != nullptr) {
                sensor_->SetThresholds(config.th_blocked_mm,
                                       config.th_blocked_exit_mm,
                                       config.th_near_mm,
                                       config.th_near_exit_mm);
            }
        });

    // 测距回调。Web 页面一秒轮询 /api/sensor，用来一边推摇杆一边看距离。
    web_server_->SetSensorCallbacks(
        [this]() { return GetSensorSnapshot(); },
        [this](int* baseline) { return CalibrateSensor(baseline); });

    if (web_server_->Start(80)) {
        ESP_LOGI(TAG, "Web server started on port 80");
        auto& board = Board::GetInstance();
        // 把访问地址显示出来，省得去路由器里翻 IP。
        // NetworkInterface 本身没有 GetIpAddress，IP 在 WifiManager 里。
        std::string ip = WifiManager::GetInstance().GetIpAddress();
        std::string msg = ip.empty() ? std::string("Web 控制已启用")
                                     : (std::string("http://") + ip);
        board.GetDisplay()->ShowNotification(msg.c_str(), 3000);
    } else {
        ESP_LOGE(TAG, "Failed to start web server");
        Board::GetInstance().GetDisplay()->ShowNotification("Web控制启动失败", 3000);
    }
}

void Application::ActivationTask() {
    // Create OTA object for activation process
    ota_ = std::make_unique<Ota>();

    // Check for new assets version
    CheckAssetsVersion();

    // Check for new firmware version
    CheckNewVersion();

    // Initialize the protocol
    InitializeProtocol();

    // Signal completion to main loop
    xEventGroupSetBits(event_group_, MAIN_EVENT_ACTIVATION_DONE);
}

void Application::CheckAssetsVersion() {
    // Only allow CheckAssetsVersion to be called once
    if (assets_version_checked_) {
        return;
    }
    assets_version_checked_ = true;

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto& assets = Assets::GetInstance();

    if (!assets.partition_valid()) {
        ESP_LOGW(TAG, "Assets partition is disabled for board %s", BOARD_NAME);
        return;
    }
    
    Settings settings("assets", true);
    // Check if there is a new assets need to be downloaded
    std::string download_url = settings.GetString("download_url");

    if (!download_url.empty()) {
        settings.EraseKey("download_url");

        char message[256];
        snprintf(message, sizeof(message), Lang::Strings::FOUND_NEW_ASSETS, download_url.c_str());
        Alert(Lang::Strings::LOADING_ASSETS, message, "cloud_arrow_down", Lang::Sounds::OGG_UPGRADE);
        
        // Wait for the audio service to be idle for 3 seconds
        vTaskDelay(pdMS_TO_TICKS(3000));
        SetDeviceState(kDeviceStateUpgrading);
        board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
        display->SetChatMessage("system", Lang::Strings::PLEASE_WAIT);

        bool success = assets.Download(download_url, [this, display](int progress, size_t speed) -> void {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
            Schedule([display, message = std::string(buffer)]() {
                display->SetChatMessage("system", message.c_str());
            });
        });

        board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (!success) {
            Alert(Lang::Strings::ERROR, Lang::Strings::DOWNLOAD_ASSETS_FAILED, "circle_xmark", Lang::Sounds::OGG_EXCLAMATION);
            vTaskDelay(pdMS_TO_TICKS(2000));
            SetDeviceState(kDeviceStateActivating);
            return;
        }
    }

    // Apply assets
    assets.Apply();
    display->SetChatMessage("system", "");
    display->SetEmotion("microchip_ai");
}

void Application::CheckNewVersion() {
    const int MAX_RETRY = 10;
    int retry_count = 0;
    int retry_delay = 10; // Initial retry delay in seconds

    auto& board = Board::GetInstance();
    while (true) {
        auto display = board.GetDisplay();
        display->SetStatus(Lang::Strings::CHECKING_NEW_VERSION);

        esp_err_t err = ota_->CheckVersion();
        if (err != ESP_OK) {
            retry_count++;
            if (retry_count >= MAX_RETRY) {
                ESP_LOGE(TAG, "Too many retries, exit version check");
                return;
            }

            char error_message[128];
            snprintf(error_message, sizeof(error_message), "code=%d, url=%s", err, ota_->GetCheckVersionUrl().c_str());
            char buffer[256];
            snprintf(buffer, sizeof(buffer), Lang::Strings::CHECK_NEW_VERSION_FAILED, retry_delay, error_message);
            Alert(Lang::Strings::ERROR, buffer, "cloud_slash", Lang::Sounds::OGG_EXCLAMATION);

            ESP_LOGW(TAG, "Check new version failed, retry in %d seconds (%d/%d)", retry_delay, retry_count, MAX_RETRY);
            for (int i = 0; i < retry_delay; i++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (GetDeviceState() == kDeviceStateIdle) {
                    break;
                }
            }
            retry_delay *= 2; // Double the retry delay
            continue;
        }
        retry_count = 0;
        retry_delay = 10; // Reset retry delay

        if (ota_->HasNewVersion()) {
            if (UpgradeFirmware(ota_->GetFirmwareUrl(), ota_->GetFirmwareVersion())) {
                return; // This line will never be reached after reboot
            }
            // If upgrade failed, continue to normal operation
        }

        // No new version, mark the current version as valid
        ota_->MarkCurrentVersionValid();
        if (!ota_->HasActivationCode() && !ota_->HasActivationChallenge()) {
            // Exit the loop if done checking new version
            break;
        }

        display->SetStatus(Lang::Strings::ACTIVATION);
        // Activation code is shown to the user and waiting for the user to input
        if (ota_->HasActivationCode()) {
            ShowActivationCode(ota_->GetActivationCode(), ota_->GetActivationMessage());
        }

        // This will block the loop until the activation is done or timeout
        for (int i = 0; i < 10; ++i) {
            ESP_LOGI(TAG, "Activating... %d/%d", i + 1, 10);
            esp_err_t err = ota_->Activate();
            if (err == ESP_OK) {
                break;
            } else if (err == ESP_ERR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(3000));
            } else {
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            if (GetDeviceState() == kDeviceStateIdle) {
                break;
            }
        }
    }
}

void Application::InitializeProtocol() {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto codec = board.GetAudioCodec();

    display->SetStatus(Lang::Strings::LOADING_PROTOCOL);

    if (ota_->HasMqttConfig()) {
        protocol_ = std::make_unique<MqttProtocol>();
    } else if (ota_->HasWebsocketConfig()) {
        protocol_ = std::make_unique<WebsocketProtocol>();
    } else {
        ESP_LOGW(TAG, "No protocol specified in the OTA config, using MQTT");
        protocol_ = std::make_unique<MqttProtocol>();
    }

    protocol_->OnConnected([this]() {
        DismissAlert();
    });

    protocol_->OnNetworkError([this](const std::string& message) {
        last_error_message_ = message;
        xEventGroupSetBits(event_group_, MAIN_EVENT_ERROR);
    });
    
    protocol_->OnIncomingAudio([this](std::unique_ptr<AudioStreamPacket> packet) {
        if (GetDeviceState() == kDeviceStateSpeaking) {
            // 只在真正收下这一帧时更新，卡死判定靠它。
            last_tts_audio_ms_.store(esp_timer_get_time() / 1000);
            audio_service_.PushPacketToDecodeQueue(std::move(packet));
        }
    });
    
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG, "Server sample rate %d does not match device output sample rate %d, resampling may cause distortion",
                protocol_->server_sample_rate(), codec->output_sample_rate());
        }
    });
    
    protocol_->OnAudioChannelClosed([this, &board]() {
        board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
        Schedule([this]() {
            auto display = Board::GetInstance().GetDisplay();
            display->SetChatMessage("system", "");
            SetDeviceState(kDeviceStateIdle);
        });
    });
    
    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        // Parse JSON data
        auto type = cJSON_GetObjectItem(root, "type");
        if (strcmp(type->valuestring, "tts") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            if (strcmp(state->valuestring, "start") == 0) {
                // 诊断：确认服务端到底有没有发 start/stop。
                // 原来这两个分支一句日志都没有，卡在 speaking 时无从判断是
                // 服务端没发 stop，还是发了但我们这边没切过去。
                ESP_LOGI(TAG, "tts start (device state: %d)", (int)GetDeviceState());
                Schedule([this]() {
                    aborted_ = false;
                    // 新一段 TTS 开始，以 start 的时刻作为计时起点：
                    // 服务端若一个音频帧都没发（半路断了），15 秒后同样会兜底。
                    last_tts_audio_ms_.store(esp_timer_get_time() / 1000);
                    SetDeviceState(kDeviceStateSpeaking);
                });
            } else if (strcmp(state->valuestring, "stop") == 0) {
                ESP_LOGI(TAG, "tts stop (device state: %d)", (int)GetDeviceState());
                Schedule([this]() {
                    // 放完音乐要退回原来的界面。SetDisplayMode 内部会顺手
                    // 调 SetMusicMode(false) 把音乐那几个 label 藏掉。
                    awaiting_music_title_ = false;
                    if (display_mode_ == kDisplayModeMusic) {
                        SetDisplayMode(display_mode_before_music_);
                    }
                    if (GetDeviceState() == kDeviceStateSpeaking) {
                        if (listening_mode_ == kListeningModeManualStop) {
                            SetDeviceState(kDeviceStateIdle);
                        } else {
                            SetDeviceState(kDeviceStateListening);
                        }
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (cJSON_IsString(text)) {
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    std::string t = text->valuestring;

                    // 服务端会把工具调用（self.motor.emotion 之类）的签名也塞进
                    // sentence_start 的 text 里，原样显示就成了屏幕上的「% self.motor」。
                    // 它不是小智说的话，直接丢掉：不显示、不朗读、不参与语气联动。
                    if (!t.empty() && t[0] == '%') {
                        // 这一段在 lambda 里，不是 switch，所以只能用 return
                        return;
                    }

                    // === 7b：音乐界面识别 ===
                    // 固件里没有音乐服务，service 端的 play_music 只是把 opus 流
                    // 当普通 TTS 推下来。但工具调用和歌名会以 sentence_start 的
                    // 文本形式出现，顺序固定：% play_music... → 《歌名》 → 歌词。
                    // 认这个格式是 hack，官方一改就失效，失效也只是退回普通字幕。
                    if (t.rfind("% play_music", 0) == 0) {
                        awaiting_music_title_ = true;
                        Schedule([this, display]() {
                            if (display_mode_ != kDisplayModeMusic) {
                                display_mode_before_music_ = display_mode_;
                                SetDisplayMode(kDisplayModeMusic);
                            }
                            display->SetMusicMode(true, nullptr, nullptr);
                        });
                        return;
                    }
                    // 《 是 3 字节（E3 80 8A）。char 有符号，t[0] == '《' 永远不成立，
                    // 必须按字节比较；substr 的起点和长度也都按字节算。
                    // 必须紧跟 % play_music 才算歌名。单独出现《...》不算——
                    // 服务端列歌单时也用这个格式，那样会把一次普通回答变成音乐界面。
                    if (awaiting_music_title_ && t.size() >= 6 && t.compare(0, 3, "《") == 0) {
                        size_t end = t.find("》");
                        if (end != std::string::npos && end > 3) {
                            std::string title = t.substr(3, end - 3);
                            awaiting_music_title_ = false;
                            Schedule([this, display, title]() {
                                if (display_mode_ != kDisplayModeMusic) {
                                    display_mode_before_music_ = display_mode_;
                                    SetDisplayMode(kDisplayModeMusic);
                                }
                                display->SetMusicMode(true, title.c_str(), nullptr);
                            });
                            return;
                        }
                    }
                    // 已经在音乐界面里，剩下的文本才当歌词。
                    // 但 '%' 开头的是工具调用回显（% self.motor.emotion 之类），
                    // 不是歌词，扔了——否则屏幕上会滚过一串 self.motor.emotior。
                    if (display_mode_ == kDisplayModeMusic) {
                        if (t.empty() || t[0] == '%') {
                            return;
                        }
                        Schedule([display, message = std::string(t)]() {
                            display->SetMusicMode(true, nullptr, message.c_str());
                        });
                        return;
                    }

                    Schedule([display, message = std::string(t)]() {
                        display->SetChatMessage("assistant", message.c_str());
                    });
                    // 语气联动：服务端一句只给一个 emotion（在 llm 消息里），
                    // 整段话从头到尾一个表情。这里按每句话的文本再切一次，
                    // 让眼睛跟着句子走。只做"语气"层面（笑/疑问/惊叹），
                    // 情感层面（难过/生气/亲昵）仍交给服务端的 emotion，
                    // 因为那个判断比关键词准。
                    const char* tone = EmotionFromSentence(text->valuestring);
                    ESP_LOGI(TAG, "sentence tone: %s", (tone != nullptr) ? tone : "(none)");
                    if (tone != nullptr && last_sentence_tone_ != tone) {
                        last_sentence_tone_ = tone;
                        Schedule([display, e = std::string(tone)]() {
                            display->SetEmotion(e.c_str());
                        });
                    }
                }
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            auto text = cJSON_GetObjectItem(root, "text");
            if (cJSON_IsString(text)) {
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                Schedule([display, message = std::string(text->valuestring)]() {
                    display->SetChatMessage("user", message.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(emotion)) {
                Schedule([display, emotion_str = std::string(emotion->valuestring)]() {
                    display->SetEmotion(emotion_str.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "mcp") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            if (cJSON_IsObject(payload)) {
                McpServer::GetInstance().ParseMessage(payload);
            }
        } else if (strcmp(type->valuestring, "system") == 0) {
            auto command = cJSON_GetObjectItem(root, "command");
            if (cJSON_IsString(command)) {
                ESP_LOGI(TAG, "System command: %s", command->valuestring);
                if (strcmp(command->valuestring, "reboot") == 0) {
                    // Do a reboot if user requests a OTA update
                    Schedule([this]() {
                        Reboot();
                    });
                } else {
                    ESP_LOGW(TAG, "Unknown system command: %s", command->valuestring);
                }
            }
        } else if (strcmp(type->valuestring, "alert") == 0) {
            auto status = cJSON_GetObjectItem(root, "status");
            auto message = cJSON_GetObjectItem(root, "message");
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(status) && cJSON_IsString(message) && cJSON_IsString(emotion)) {
                Alert(status->valuestring, message->valuestring, emotion->valuestring, Lang::Sounds::OGG_VIBRATION);
            } else {
                ESP_LOGW(TAG, "Alert command requires status, message and emotion");
            }
#if CONFIG_RECEIVE_CUSTOM_MESSAGE
        } else if (strcmp(type->valuestring, "custom") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            ESP_LOGI(TAG, "Received custom message: %s", cJSON_PrintUnformatted(root));
            if (cJSON_IsObject(payload)) {
                Schedule([this, display, payload_str = std::string(cJSON_PrintUnformatted(payload))]() {
                    display->SetChatMessage("system", payload_str.c_str());
                });
            } else {
                ESP_LOGW(TAG, "Invalid custom message format: missing payload");
            }
#endif
        } else {
            ESP_LOGW(TAG, "Unknown message type: %s", type->valuestring);
        }
    });
    
    protocol_->Start();
}

void Application::ShowActivationCode(const std::string& code, const std::string& message) {
    struct digit_sound {
        char digit;
        const std::string_view& sound;
    };
    static const std::array<digit_sound, 10> digit_sounds{{
        digit_sound{'0', Lang::Sounds::OGG_0},
        digit_sound{'1', Lang::Sounds::OGG_1}, 
        digit_sound{'2', Lang::Sounds::OGG_2},
        digit_sound{'3', Lang::Sounds::OGG_3},
        digit_sound{'4', Lang::Sounds::OGG_4},
        digit_sound{'5', Lang::Sounds::OGG_5},
        digit_sound{'6', Lang::Sounds::OGG_6},
        digit_sound{'7', Lang::Sounds::OGG_7},
        digit_sound{'8', Lang::Sounds::OGG_8},
        digit_sound{'9', Lang::Sounds::OGG_9}
    }};

    // This sentence uses 9KB of SRAM, so we need to wait for it to finish
    Alert(Lang::Strings::ACTIVATION, message.c_str(), "link", Lang::Sounds::OGG_ACTIVATION);

    for (const auto& digit : code) {
        auto it = std::find_if(digit_sounds.begin(), digit_sounds.end(),
            [digit](const digit_sound& ds) { return ds.digit == digit; });
        if (it != digit_sounds.end()) {
            audio_service_.PlaySound(it->sound);
        }
    }
}

void Application::Alert(const char* status, const char* message, const char* emotion, const std::string_view& sound) {
    ESP_LOGW(TAG, "Alert [%s] %s: %s", emotion, status, message);
    auto display = Board::GetInstance().GetDisplay();
    display->SetStatus(status);
    display->SetEmotion(emotion);
    display->SetChatMessage("system", message);
    if (!sound.empty()) {
        audio_service_.PlaySound(sound);
    }
}

void Application::DismissAlert() {
    if (GetDeviceState() == kDeviceStateIdle) {
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::STANDBY);
        display->SetEmotion("neutral");
        display->SetChatMessage("system", "");
    }
}

void Application::ToggleChatState() {
    xEventGroupSetBits(event_group_, MAIN_EVENT_TOGGLE_CHAT);
}

void Application::StartListening() {
    xEventGroupSetBits(event_group_, MAIN_EVENT_START_LISTENING);
}

void Application::StopListening() {
    xEventGroupSetBits(event_group_, MAIN_EVENT_STOP_LISTENING);
}

void Application::HandleToggleChatEvent() {
    auto state = GetDeviceState();
    
    if (state == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (state == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    } else if (state == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (state == kDeviceStateIdle) {
        ListeningMode mode = GetDefaultListeningMode();
        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // Schedule to let the state change be processed first (UI update)
            Schedule([this, mode]() {
                ContinueOpenAudioChannel(mode);
            });
            return;
        }
        SetListeningMode(mode);
    } else if (state == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonNone);
    } else if (state == kDeviceStateListening) {
        protocol_->CloseAudioChannel();
    }
}

void Application::ContinueOpenAudioChannel(ListeningMode mode) {
    // Check state again in case it was changed during scheduling
    if (GetDeviceState() != kDeviceStateConnecting) {
        return;
    }

    if (!protocol_->IsAudioChannelOpened()) {
        if (!protocol_->OpenAudioChannel()) {
            return;
        }
    }

    SetListeningMode(mode);
}

void Application::HandleStartListeningEvent() {
    auto state = GetDeviceState();
    
    if (state == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (state == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }
    
    if (state == kDeviceStateIdle) {
        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // Schedule to let the state change be processed first (UI update)
            Schedule([this]() {
                ContinueOpenAudioChannel(kListeningModeManualStop);
            });
            return;
        }
        SetListeningMode(kListeningModeManualStop);
    } else if (state == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonNone);
        SetListeningMode(kListeningModeManualStop);
    }
}

void Application::HandleStopListeningEvent() {
    auto state = GetDeviceState();
    
    if (state == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    } else if (state == kDeviceStateListening) {
        if (protocol_) {
            protocol_->SendStopListening();
        }
        SetDeviceState(kDeviceStateIdle);
    }
}

void Application::HandleWakeWordDetectedEvent() {
    if (!protocol_) {
        return;
    }

    auto state = GetDeviceState();
    auto wake_word = audio_service_.GetLastWakeWord();
    ESP_LOGI(TAG, "Wake word detected: %s (state: %d)", wake_word.c_str(), (int)state);

    // 新一轮对话，语气记录归零，这样第一句就能按文本重新设表情。
    last_sentence_tone_.clear();

    // 人喊了一声，算一次互动：机器人模式的 20 分钟时限从这里重新数。
    TouchRobotInteraction();

    // 机器人模式运行中被唤醒：先停手（停音效、停大幅动作），模式本身不退出。
    // 循环那边看到 robot_paused_ 就每 100ms 检查一次，不会继续驱动电机。
    if (robot_mode_.load() != kRobotNone) {
        robot_paused_.store(true);
        robot_need_recalibrate_.store(true);
        ESP_LOGI(TAG, "robot mode paused for conversation");
    }

    // 第 6 步：人来了，重置 60 秒倒计时。睡眠态下顺便切成眼睛。
    if (display_mode_ == kDisplayModeSleep) {
        WakeUp();
    } else {
        ResetIdleTimer();
    }

    if (state == kDeviceStateIdle) {
        audio_service_.EncodeWakeWord();
        auto wake_word = audio_service_.GetLastWakeWord();

        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // Schedule to let the state change be processed first (UI update),
            // then continue with OpenAudioChannel which may block for ~1 second
            Schedule([this, wake_word]() {
                ContinueWakeWordInvoke(wake_word);
            });
            return;
        }
        // Channel already opened, continue directly
        ContinueWakeWordInvoke(wake_word);
    } else if (state == kDeviceStateSpeaking || state == kDeviceStateListening) {
        AbortSpeaking(kAbortReasonWakeWordDetected);
        // Clear send queue to avoid sending residues to server
        while (audio_service_.PopPacketFromSendQueue());

        if (state == kDeviceStateListening) {
            protocol_->SendStartListening(GetDefaultListeningMode());
            audio_service_.ResetDecoder();
            // 唤醒音效已关闭：这里的 popup 会插进 TTS 解码队列，打断正在说的话。
            // 倾听状态靠屏幕上眼睛的变化来反馈，不需要声音。
            // Re-enable wake word detection as it was stopped by the detection itself
            audio_service_.EnableWakeWordDetection(true);
        } else {
            // Play popup sound and start listening again
            play_popup_on_listening_ = true;
            SetListeningMode(GetDefaultListeningMode());
        }
    } else if (state == kDeviceStateActivating) {
        // Restart the activation check if the wake word is detected during activation
        SetDeviceState(kDeviceStateIdle);
    }
}

void Application::ContinueWakeWordInvoke(const std::string& wake_word) {
    // Check state again in case it was changed during scheduling
    if (GetDeviceState() != kDeviceStateConnecting) {
        return;
    }

    if (!protocol_->IsAudioChannelOpened()) {
        if (!protocol_->OpenAudioChannel()) {
            audio_service_.EnableWakeWordDetection(true);
            return;
        }
    }

    ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
#if CONFIG_SEND_WAKE_WORD_DATA
    // Encode and send the wake word data to the server
    while (auto packet = audio_service_.PopWakeWordPacket()) {
        protocol_->SendAudio(std::move(packet));
    }
    // Set the chat state to wake word detected
    protocol_->SendWakeWordDetected(wake_word);

    // Set flag to play popup sound after state changes to listening
    play_popup_on_listening_ = true;
    SetListeningMode(GetDefaultListeningMode());
#else
    // Set flag to play popup sound after state changes to listening
    // (PlaySound here would be cleared by ResetDecoder in EnableVoiceProcessing)
    play_popup_on_listening_ = true;
    SetListeningMode(GetDefaultListeningMode());
#endif
}

// 第 2 步：显示模式切换
// 从 NVS 恢复上次的界面。放在激活完成之后调用：开机头 9.6 秒要留给
// 小智自己的版本号/连接状态，这时候切眼睛会把 LVGL 停掉，那些字就没了。
void Application::RestoreDisplayMode() {
    Settings settings("display", false);
    int saved = settings.GetInt("mode", static_cast<int>(kDisplayModeDefault));
    if (saved == static_cast<int>(kDisplayModeSleep)) {
        // 上次停在睡眠（时钟）。sleep 本身不落盘，走到这里只可能是旧版本留下的值。
        // 现在会进睡眠的就两种来路：官方界面 60 秒、机器人模式超时，
        // 两种醒来都该回官方界面，不再是眼睛。
        saved = static_cast<int>(kDisplayModeDefault);
    }
    if (saved != static_cast<int>(kDisplayModeDefault) &&
        saved != static_cast<int>(kDisplayModeEyeOnly)) {
        saved = static_cast<int>(kDisplayModeDefault);
    }
    ESP_LOGI(TAG, "Restoring display mode from NVS: %d", saved);
    SetDisplayMode(static_cast<DisplayMode>(saved));
}

void Application::SetDisplayMode(DisplayMode mode) {
    if (display_mode_ == mode) {
        return;
    }

    // 省电信号：只有睡眠（大时钟）才降频，其它任何状态立刻回到 PERFORMANCE。
    // 机器人模式下也会走这里，所以模式运行时不降频是自动成立的。
    Board::GetInstance().SetPowerSaveLevel(
        mode == kDisplayModeSleep ? PowerSaveLevel::LOW_POWER
                                  : PowerSaveLevel::PERFORMANCE);
    display_mode_ = mode;

    // 记住用户最后停在哪一屏。只存 Default 与 EyeOnly：
    // 睡眠是临时状态，重启后不该直接黑着脸等 60 秒。
    if (mode != kDisplayModeSleep) {
        Settings settings("display", true);
        settings.SetInt("mode", static_cast<int>(mode));
    }

    const char* mode_name = (mode == kDisplayModeDefault)  ? "Default"
                          : (mode == kDisplayModeEyeOnly) ? "Eye Only"
                          : (mode == kDisplayModeMusic)   ? "Music"
                                                          : "Sleep";
    ESP_LOGI(TAG, "Display mode changed to: %s", mode_name);

    auto display = Board::GetInstance().GetDisplay();

    // 离开音乐界面时统一退出。放在 switch 之前，三个分支都不用各写一遍。
    if (mode != kDisplayModeMusic && display->IsMusicMode()) {
        display->SetMusicMode(false, nullptr, nullptr);
    }

    switch (mode) {
        case kDisplayModeEyeOnly:
            display->HideClock();
            display->SetStatus("");
            display->SetChatMessage("system", "");
            display->SetAnimatedEmotionMode(true);
            break;

        case kDisplayModeSleep:
            // 整屏时钟：先关眼睛（会 lvgl_port_resume），再让 Display 画时钟
            display->SetAnimatedEmotionMode(false);
            display->ShowClock();
            break;

        case kDisplayModeMusic:
            // 音乐界面走 LVGL，必须先把眼睛停掉（它会 lvgl_port_stop）。
            // 中间那一下会闪，这是架构决定的：眼睛直刷 panel，和 LVGL 不能同屏。
            display->SetAnimatedEmotionMode(false);
            display->HideClock();
            break;

        case kDisplayModeDefault:
        default:
            display->SetAnimatedEmotionMode(false);
            display->HideClock();
            display->SetStatus(Lang::Strings::STANDBY);
            display->SetEmotion("neutral");
            break;
    }
}

void Application::AttachObstacleProbe(std::function<bool()> probe) {
    obstacle_probe_ = std::move(probe);

    if (!obstacle_probe_ || obstacle_watch_started_) {
        return;
    }
    obstacle_watch_started_ = true;

    // 50ms 一轮。实时控制是持续状态，下发了就不再经过 SetRealtimeMotorCommand，
    // 所以「已经开出去、障碍才出现」这种情况只能靠这个看门任务兜住。
    // 50ms 在 100% 占空比下轮子走两三厘米，来不及撞上。
    xTaskCreate([](void* arg) {
        auto* app = static_cast<Application*>(arg);
        while (true) {
            app->CheckObstacleWatchdog();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }, "obstacle_watch", 3072, this, tskIDLE_PRIORITY + 2, NULL);

    ESP_LOGI(TAG, "obstacle probe attached, watchdog started");
}

void Application::CheckObstacleWatchdog() {
    if (!obstacle_probe_ || !realtime_control_active_.load()) {
        return;
    }
    // 只拦后退。前进和转向都不拦——后面撞了没处躲，前面撞了还能退。
    // 传感器装在后头，管的就是这一头。
    if (last_realtime_direction_ != 2) {
        return;
    }
    // Web 遥控豁免，跟 SetRealtimeMotorCommand 那道闸门保持一致。
    // 不一致的后果是：浏览器按住后退，闸门放行、50ms 后被这里停掉，
    // 表现成「Web 遥控只能退半秒」。豁免的理由和闸门那边一样——
    // 传感器坏了要还能手动把车开走。
    if (last_command_from_web_.load()) {
        return;
    }
    if (!obstacle_probe_()) {
        return;
    }
    ESP_LOGW(TAG, "obstacle appeared while reversing, stopping");
    StopRealtimeMotorControl();
}

void Application::AttachSensor(Vl6180xSensor* sensor) {
    sensor_ = sensor;
    if (sensor_ == nullptr) {
        return;
    }

    // 障碍跨阈值时响一声。
    //
    // 回调是在采样任务里跑的，绝对不能在那里直接播音频——AudioService
    // 排队播放会阻塞，采样周期（50ms）立刻崩掉。所以这里只做投递。
    //
    // 正在说话时跳过：否则你正聊到一半，它凑近点就开始哔哔。
    // 桌沿：激光打空那一刻立即停车，并且响提示音。
    //
    // 这里比障碍回调更急——障碍是"前面有东西"，桌沿是"脚下没东西"，
    // 晚 100ms 就掉下去了。所以不判断是不是 Web 遥控：手动模式掉桌也一样。
    sensor_->SetEdgeCallback([this](bool at_edge) {
        if (!at_edge) {
            return;
        }
        Schedule([this]() {
            if (realtime_control_active_.load()) {
                ESP_LOGW(TAG, "edge detected while moving, emergency stop");
                StopRealtimeMotorControl();
            }
            if (GetDeviceState() == kDeviceStateSpeaking) {
                ESP_LOGI(TAG, "edge alert skipped while speaking");
                return;
            }
            ESP_LOGI(TAG, "edge detected, playing edge_alert");
            audio_service_.PlaySound(Lang::Sounds::OGG_EDGE_ALERT);
        });
    });

    sensor_->SetObstacleCallback([this](int old_state, int new_state) {
        // 只有和 kBlocked(=2) 相关的跨越才有意义：
        //   进入 blocked  → 停车 + near5cm
        //   离开 blocked  → far10cm
        //   near <-> clear → 什么都不做
        bool entered = (old_state != 2 && new_state == 2);
        bool left    = (old_state == 2 && new_state != 2);
        if (!entered && !left) {
            return;
        }
        bool blocked = entered;
        Schedule([this, blocked, entered]() {
            // 保护性停车：闸门只在"下发指令那一刻"检查，指令一旦下发
            // 电机就一直在转，途中撞上障碍没人复查。所以这里补一次——
            // 进入 blocked 且电机正在动，立即停。
            //
            // Web 遥控器豁免：手动模式要能顶着障碍开，不然传感器一灵敏
            // 就变成遥控器不受控了。
            if (blocked && realtime_control_active_.load() && !last_command_from_web_.load()) {
                ESP_LOGW(TAG, "obstacle blocked while moving, emergency stop");
                StopRealtimeMotorControl();
            }

            if (GetDeviceState() == kDeviceStateSpeaking) {
                ESP_LOGI(TAG, "obstacle sound skipped while speaking");
                return;
            }
            ESP_LOGI(TAG, "obstacle %s, playing %s", entered ? "entered" : "left",
                     entered ? "near5cm" : "far10cm");
            audio_service_.PlaySound(blocked ? Lang::Sounds::OGG_NEAR5CM
                                             : Lang::Sounds::OGG_FAR10CM);
        });
    });
}

SensorSnapshot Application::GetSensorSnapshot() const {
    if (sensor_ == nullptr) {
        SensorSnapshot empty = {};
        empty.status = 4;          // not_ready
        empty.sensor_ok = false;
        return empty;
    }
    return sensor_->Snapshot();
}

bool Application::CalibrateSensor(int* baseline_mm_out) {
    if (sensor_ == nullptr) {
        return false;
    }
    // 采样期间会震动判断，先把电机停掉
    StopRealtimeMotorControl();
    return sensor_->Calibrate(baseline_mm_out);
}

void Application::DebugMotorCommand(int direction, int speed) {
    SetRealtimeMotorCommand(direction, speed, true);
}

void Application::DebugMotorStop() {
    StopRealtimeMotorControl();
}

void Application::ToggleDisplayMode() {
    // 长按 BOOT 用：眼睛 <-> 官方界面（顶部状态栏 + 中间字幕）。
    // 日常的睡眠/唤醒不走这里，走 EnterSleep()/WakeUp()。
    SetDisplayMode(display_mode_ == kDisplayModeDefault
                   ? kDisplayModeEyeOnly
                   : kDisplayModeDefault);
}

// === 第 6 步：显示状态机 ===

void Application::IdleTimerCallback(void* arg) {
    auto* self = static_cast<Application*>(arg);
    if (self == nullptr) {
        return;
    }
    // 唯一检查点：CanEnterSleepMode() 里已经排除了"正在对话""音频没空闲"
    // 这几种情况，机器人模式那条等第 10 步再加。
    if (!self->CanEnterSleepMode()) {
        ESP_LOGI(TAG, "idle timer fired but CanEnterSleepMode()==false, skip");
        return;
    }
    // 关键：这个回调跑在 esp_timer 任务里，而 EnterSleep() 内部要析构
    // RoboEyesAdapter（等最多 800ms）、调 lvgl_port_resume()、再拿 LVGL 锁刷新时钟。
    // 一旦其中任何一步卡住，整个 esp_timer 任务停摆，所有定时器失效，WDT 直接报
    // "CPU 0: esp_timer"。实测就是这么卡死的。
    // 所以这里只投递事件，真正的切换交给 Application::Run() 主循环去做。
    ESP_LOGI(TAG, "idle timer fired, dispatching to main loop");
    self->Schedule([self]() { self->EnterSleep(); });
}

void Application::ResetIdleTimer() {
    if (idle_timer_ == nullptr) {
        ESP_LOGW(TAG, "idle timer not created yet");
        return;
    }
    esp_err_t e1 = esp_timer_stop(idle_timer_);
    esp_err_t e2 = esp_timer_start_once(idle_timer_, (uint64_t)kIdleToSleepSeconds * 1000000ULL);
    ESP_LOGI(TAG, "idle timer reset: stop=%d start=%d (%d s)", (int)e1, (int)e2, kIdleToSleepSeconds);
}

void Application::EnterSleep() {
    if (display_mode_ == kDisplayModeSleep) {
        return;
    }
    ESP_LOGI(TAG, "Entering sleep (clock screen)");
    // 先记住睡前停在哪一屏，醒来原样回去。
    // 音乐界面不算一屏（它本来就是临时状态），统一当官方界面。
    display_mode_before_sleep_ = (display_mode_ == kDisplayModeEyeOnly)
                                     ? kDisplayModeEyeOnly
                                     : kDisplayModeDefault;
    // 进睡眠前先把电机停掉。短按 BOOT 从"正在前进"直接掉到 idle 时，
    // 如果不停，轮子会一直转到 duration 结束才停，看起来像按键没反应。
    StopRealtimeMotorControl();
    // 降频已收口到 SetDisplayMode，这里不再重复调用。
    // 只降 CPU 频率，不发 esp_light_sleep——WakeNet 要继续跑才能被喊醒。
    SetDisplayMode(kDisplayModeSleep);
}

void Application::WakeUp() {
    // 醒来回到睡前那一屏，而不是固定某一屏：
    //   睡前是眼睛（机器人模式）→ 回眼睛
    //   睡前是官方界面           → 回官方界面
    // 以前这里写死 kDisplayModeEyeOnly，官方界面在时钟里被喊醒之后
    // 就再也回不去官方界面，只能长按 BOOT 手动切。
    const DisplayMode target = display_mode_before_sleep_;
    if (display_mode_ == target) {
        ResetIdleTimer();   // 已经在目标界面，只刷新倒计时
        return;
    }
    ESP_LOGI(TAG, "Waking up (%s screen)",
             target == kDisplayModeEyeOnly ? "eye" : "default");
    // 升频已收口到 SetDisplayMode，这里不再重复调用。
    SetDisplayMode(target);
    ResetIdleTimer();
}

void Application::TouchRobotInteraction() {
    last_robot_interaction_ms_.store(esp_timer_get_time() / 1000);
}

bool Application::IsBusyWithTask() {
    auto s = GetDeviceState();
    return s == kDeviceStateConnecting
        || s == kDeviceStateListening
        || s == kDeviceStateSpeaking
        || s == kDeviceStateUpgrading
        || s == kDeviceStateActivating;
}

void Application::HandleStateChangedEvent() {
    DeviceState new_state = state_machine_.GetState();
    clock_ticks_ = 0;

    // 先转给板卡：灯带的槽位切换要跟着状态走，别被后面那些分支的早退挡掉。
    if (led_state_listener_) {
        led_state_listener_(static_cast<int>(new_state));
    }

    // 听说状态也算互动，否则「一直聊、但每句都由服务端先说话」这种情况下
    // 20 分钟时限会照数不误，把正在对话的机器人踢出模式。
    if (new_state == kDeviceStateListening || new_state == kDeviceStateSpeaking) {
        TouchRobotInteraction();
    }

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    led->OnStateChanged();
    
    switch (new_state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            // 进 idle 不主动睡，全部交给 60 秒倒计时：
            //   开机激活完成 → 眼睛或官方界面，60 秒不动才进大时钟
            //   说完话回 idle → 同样先停在当前界面，60 秒后才进时钟
            // 以前这里是直接 EnterSleep()，说完话屏幕会立刻跳回时钟，
            // 眼睛只剩听说时那一瞬间能看见。
                        ResetIdleTimer();

            // 对话结束回 idle：机器人模式如果还在，恢复跑，并先把 paused 清掉。
            // 位置校准由 RobotModeLoop 检测「从暂停恢复」时自己做。
            if (robot_mode_.load() != kRobotNone && robot_paused_.load()) {
                robot_paused_.store(false);
                ESP_LOGI(TAG, "robot mode resumed after conversation");
            }

            // 眼睛模式下 LVGL 已经停了，SetStatus/ClearChatMessages 都不走 LVGL 那条；
            // 眼睛模式下 LVGL 已经停掉，SetStatus/ClearChatMessages 动的是 LVGL 对象，
            // 虽然不会崩但没意义，只把表情推给 RoboEyes 就够了。
            if (display_mode_ == kDisplayModeEyeOnly) {
                display->SetEmotion("neutral");
            } else {
                display->SetStatus(Lang::Strings::STANDBY);
                display->ClearChatMessages();  // Clear messages first
                display->SetEmotion("neutral"); // Then set emotion (wechat mode checks child count)
            }
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(true);
            break;
        case kDeviceStateConnecting:
            if (display_mode_ == kDisplayModeEyeOnly) {
                display->SetEmotion("neutral");
            } else {
                display->SetStatus(Lang::Strings::CONNECTING);
                display->SetEmotion("neutral");
                display->SetChatMessage("system", "");
            }
            break;
        case kDeviceStateListening:
            ResetIdleTimer();   // 第 6 步：正在听不算闲置
            if (display_mode_ == kDisplayModeEyeOnly) {
                display->SetEmotion("neutral");
            } else {
                display->SetStatus(Lang::Strings::LISTENING);
                display->SetEmotion("neutral");
            }

            // Make sure the audio processor is running
            if (play_popup_on_listening_ || !audio_service_.IsAudioProcessorRunning()) {
                // For auto mode, wait for playback queue to be empty before enabling voice processing
                // This prevents audio truncation when STOP arrives late due to network jitter
                if (listening_mode_ == kListeningModeAutoStop) {
                    audio_service_.WaitForPlaybackQueueEmpty();
                }
                
                // Send the start listening command
                protocol_->SendStartListening(listening_mode_);
                audio_service_.EnableVoiceProcessing(true);
            }

#ifdef CONFIG_WAKE_WORD_DETECTION_IN_LISTENING
            // Enable wake word detection in listening mode (configured via Kconfig)
            audio_service_.EnableWakeWordDetection(audio_service_.IsAfeWakeWord());
#else
            // Disable wake word detection in listening mode
            audio_service_.EnableWakeWordDetection(false);
#endif
            
            // 唤醒音效已关闭：popup 与 TTS 共用同一个解码队列，先入先播，
            // 插进去就会打断或延迟小智说话。倾听状态靠屏幕眼睛变化反馈。
            // 标志仍要清掉，上面 play_popup_on_listening_ 还承担
            // "是否需要重新下发 start listening" 的语义。
            if (play_popup_on_listening_) {
                play_popup_on_listening_ = false;
            }
            break;
        case kDeviceStateSpeaking:
            ResetIdleTimer();   // 第 6 步：正在说话不算闲置
            if (display_mode_ != kDisplayModeEyeOnly) {
                display->SetStatus(Lang::Strings::SPEAKING);
            }

            if (listening_mode_ != kListeningModeRealtime) {
                audio_service_.EnableVoiceProcessing(false);
                // Only AFE wake word can be detected in speaking mode
                audio_service_.EnableWakeWordDetection(audio_service_.IsAfeWakeWord());
            }
            audio_service_.ResetDecoder();
            break;
        case kDeviceStateWifiConfiguring:
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(false);
            break;
        default:
            // Do nothing
            break;
    }
}

void Application::Schedule(std::function<void()>&& callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, MAIN_EVENT_SCHEDULE);
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    if (protocol_) {
        protocol_->SendAbortSpeaking(reason);
    }
}

void Application::SetListeningMode(ListeningMode mode) {
    listening_mode_ = mode;
    SetDeviceState(kDeviceStateListening);
}

ListeningMode Application::GetDefaultListeningMode() const {
    return aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime;
}

void Application::Reboot() {
    ESP_LOGI(TAG, "Rebooting...");
    // Disconnect the audio channel
    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        protocol_->CloseAudioChannel();
    }
    protocol_.reset();
    audio_service_.Stop();

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

bool Application::UpgradeFirmware(const std::string& url, const std::string& version) {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();

    std::string upgrade_url = url;
    std::string version_info = version.empty() ? "(Manual upgrade)" : version;

    // Close audio channel if it's open
    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        ESP_LOGI(TAG, "Closing audio channel before firmware upgrade");
        protocol_->CloseAudioChannel();
    }
    ESP_LOGI(TAG, "Starting firmware upgrade from URL: %s", upgrade_url.c_str());

    Alert(Lang::Strings::OTA_UPGRADE, Lang::Strings::UPGRADING, "download", Lang::Sounds::OGG_UPGRADE);
    vTaskDelay(pdMS_TO_TICKS(3000));

    SetDeviceState(kDeviceStateUpgrading);

    std::string message = std::string(Lang::Strings::NEW_VERSION) + version_info;
    display->SetChatMessage("system", message.c_str());

    board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
    audio_service_.Stop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    bool upgrade_success = Ota::Upgrade(upgrade_url, [this, display](int progress, size_t speed) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
        Schedule([display, message = std::string(buffer)]() {
            display->SetChatMessage("system", message.c_str());
        });
    });

    if (!upgrade_success) {
        // Upgrade failed, restart audio service and continue running
        ESP_LOGE(TAG, "Firmware upgrade failed, restarting audio service and continuing operation...");
        audio_service_.Start(); // Restart audio service
        board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER); // Restore power save level
        Alert(Lang::Strings::ERROR, Lang::Strings::UPGRADE_FAILED, "circle_xmark", Lang::Sounds::OGG_EXCLAMATION);
        vTaskDelay(pdMS_TO_TICKS(3000));
        return false;
    } else {
        // Upgrade success, reboot immediately
        ESP_LOGI(TAG, "Firmware upgrade successful, rebooting...");
        display->SetChatMessage("system", "Upgrade successful, rebooting...");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Brief pause to show message
        Reboot();
        return true;
    }
}

void Application::WakeWordInvoke(const std::string& wake_word) {
    if (!protocol_) {
        return;
    }

    auto state = GetDeviceState();
    
    if (state == kDeviceStateIdle) {
        audio_service_.EncodeWakeWord();

        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // Schedule to let the state change be processed first (UI update)
            Schedule([this, wake_word]() {
                ContinueWakeWordInvoke(wake_word);
            });
            return;
        }
        // Channel already opened, continue directly
        ContinueWakeWordInvoke(wake_word);
    } else if (state == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (state == kDeviceStateListening) {   
        Schedule([this]() {
            if (protocol_) {
                protocol_->CloseAudioChannel();
            }
        });
    }
}

bool Application::CanEnterSleepMode() {
    if (GetDeviceState() != kDeviceStateIdle) {
        return false;
    }

    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        return false;
    }

    if (!audio_service_.IsIdle()) {
        return false;
    }

    // 机器人模式运行期间不休眠。模式自己会一直驱动电机，进大时钟等于
    // 把屏幕切成时钟但机器人还在跑，看起来像失控。
    if (robot_mode_.load() != kRobotNone) {
        return false;
    }

    // Now it is safe to enter sleep mode
    return true;
}

void Application::SendMcpMessage(const std::string& payload) {
    // Always schedule to run in main task for thread safety
    Schedule([this, payload = std::move(payload)]() {
        if (protocol_) {
            protocol_->SendMcpMessage(payload);
        }
    });
}

void Application::SetAecMode(AecMode mode) {
    aec_mode_ = mode;
    Schedule([this]() {
        auto& board = Board::GetInstance();
        auto display = board.GetDisplay();
        switch (aec_mode_) {
        case kAecOff:
            audio_service_.EnableDeviceAec(false);
            display->ShowNotification(Lang::Strings::RTC_MODE_OFF);
            break;
        case kAecOnServerSide:
            audio_service_.EnableDeviceAec(false);
            display->ShowNotification(Lang::Strings::RTC_MODE_ON);
            break;
        case kAecOnDeviceSide:
            audio_service_.EnableDeviceAec(true);
            display->ShowNotification(Lang::Strings::RTC_MODE_ON);
            break;
        }

        // If the AEC mode is changed, close the audio channel
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::PlaySound(const std::string_view& sound) {
    audio_service_.PlaySound(sound);
}

void Application::ResetProtocol() {
    Schedule([this]() {
        // Close audio channel if opened
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
        // Reset protocol
        protocol_.reset();
    });
}


// ============================================================================
// 电机控制实现（第 1 步，来自 2.1.0 qebabe-xiaoche，DRV8833 双 H 桥）
// direction: 0=停 1=右 2=后 3=左 4=前
// priority:  0=低（空闲微动作） 1=中（情绪/语音反馈） 2=高（MCP 语音指令）
// ============================================================================

void Application::InitMotorPwm() {
    if (motor_pwm_initialized_) return;

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution = (ledc_timer_bit_t)pwm_resolution_bits_;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.freq_hz = (uint32_t)pwm_freq_hz_;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    if (ledc_timer_config(&ledc_timer) != ESP_OK) {
        ESP_LOGE(TAG, "InitMotorPwm: ledc_timer_config failed");
        return;
    }

    const gpio_num_t pins[4] = {MOTOR_LF_GPIO, MOTOR_LB_GPIO, MOTOR_RF_GPIO, MOTOR_RB_GPIO};
    for (int i = 0; i < 4; ++i) {
        ledc_channel_config_t ch = {};
        ch.gpio_num = pins[i];
        ch.speed_mode = LEDC_LOW_SPEED_MODE;
        ch.channel = (ledc_channel_t)i;
        ch.timer_sel = LEDC_TIMER_0;
        ch.duty = 0;
        ch.hpoint = 0;
        if (ledc_channel_config(&ch) != ESP_OK) {
            ESP_LOGE(TAG, "InitMotorPwm: ledc_channel_config ch%d failed", i);
        }
    }

    // fade 服务只需要装一次。2.1.0 在 main.cc 里也装了一次，日志里会打
    // "fade function already installed"，这里只保留这一处。
    if (ledc_fade_func_install(0) != ESP_OK) {
        ESP_LOGW(TAG, "InitMotorPwm: ledc_fade_func_install failed or already installed");
    }

    motor_pwm_initialized_ = true;
    ESP_LOGI(TAG, "InitMotorPwm: initialized (freq=%dHz, bits=%d, ramp=%dms)",
             pwm_freq_hz_, pwm_resolution_bits_, pwm_ramp_ms_);
}

bool Application::SetRealtimeMotorCommand(int direction, int speed, bool from_web) {
    ESP_LOGI(TAG, "SetRealtimeMotorCommand: direction=%d speed=%d%s", direction, speed,
             from_web ? " (web)" : "");

    // Web 遥控是人在开它，算互动。机器人模式自己驱动的调用（from_web=false）不算，
    // 否则它一直在走，20 分钟时限永远数不到。
    if (from_web) {
        TouchRobotInteraction();
    }

    // ---- 开关量避障（红外对管）：最先拦，也最便宜 ----
    //
    // 只拦后退：前进是脱离动作，转向是绕行动作，这两个方向被挡住就无路可走。
    // 传感器装在后头，管的是倒车这一头。
    // Web 遥控豁免，理由和下面传感器那道闸一样——传感器坏了要还能手动开走。
    if (!from_web && obstacle_probe_ && direction == 2 && obstacle_probe_()) {
        ESP_LOGW(TAG, "SetRealtimeMotorCommand: obstacle behind, refuse backward");
        motor_refusal_streak_.fetch_add(1);
        return false;
    }

    // ---- 安全闸门 ----
    //
    // 优先级：传感器 > BOOT > 机器人模式 > 语音。
    // 这里是唯一入口，所有电机指令都要过这道关。
    //
    // Web 遥控器（from_web=true）完全豁免：手动模式要能强行开，
    // 否则传感器一坏就等于遥控器也废了。
    if (!from_web && sensor_ != nullptr) {
        SensorSnapshot snap = sensor_->Snapshot();

        // 传感器不可用时按降级策略走：只能左右转，前进后退限 1 秒、速度压一半。
        if (!snap.sensor_ok || snap.age_ms() > 1000) {
            bool is_turn = (direction == 1 || direction == 3);
            if (is_turn) {
                // 左右转不受限，这是降级态里唯一能自由动的方向。
                degraded_forward_start_ms_ = 0;
            } else {
                // 前后最多走 1 秒。实时控制是持续指令，没有 duration，
                // 所以自己记一个起始时刻，超时就拒绝。
                int64_t now = esp_timer_get_time() / 1000;
                if (degraded_forward_start_ms_ == 0) {
                    degraded_forward_start_ms_ = now;
                } else if (now - degraded_forward_start_ms_ > 1000) {
                    ESP_LOGW(TAG, "sensor unavailable: forward/backward exceeded 1 s, stopping");
                    degraded_forward_start_ms_ = 0;
                    StopRealtimeMotorControl();
                    motor_refusal_streak_.fetch_add(1);
                    return false;
                }
                if (speed > 50) {
                    speed = 50;
                }
                ESP_LOGW(TAG, "sensor unavailable: forward/backward limited to 50%% and 1 s");
            }
        } else if (snap.obstacle == 2) {
            // 已确认障碍：禁止继续前进，其余方向放行（要能退开）。
            if (direction == 4) {
                ESP_LOGW(TAG, "obstacle blocked (%d mm): forward refused", snap.distance_mm);
                motor_refusal_streak_.fetch_add(1);
                return false;
            }
        } else if (snap.obstacle == 1) {
            // 接近区：限速 40%。
            int limited = (int)(speed * 0.7f);
            // 低于 50 的占空比带不动这颗 N20，限速不能压到那个区间里去，
            // 否则靠近障碍时看起来不是变慢而是停死。
            if (limited < 50) {
                limited = 10;
            }
            ESP_LOGW(TAG, "obstacle near (%d mm): speed %d -> %d", snap.distance_mm, speed, limited);
            speed = limited;
        }
    }

    // 同方向同速度的重复下发直接忽略。
    //
    // 机器人模式每 100~150ms 调一次，而下面的 PWM 写入是"先缓降到 0 再缓升"，
    // 每次都要 50ms 降 + 50ms 升，等于电机永远走不到目标转速。
    // 这里只更新活跃时间戳，让超时看门狗照常工作。
    if (realtime_control_active_.load() && motor_pwm_initialized_ &&
        direction == last_realtime_direction_ && speed == last_realtime_speed_) {
        last_realtime_command_ms_.store(esp_timer_get_time() / 1000);
        return true;
    }
    motor_refusal_streak_.store(0);
    last_realtime_direction_ = direction;
    last_realtime_speed_ = speed;

    realtime_control_active_.store(true);
    last_command_from_web_.store(from_web);
    last_realtime_command_ms_.store(esp_timer_get_time() / 1000);

    // 第一次调用时才初始化 GPIO 和 PWM，避免开机就占 LEDC 资源
    if (!motor_pwm_initialized_) {
        std::lock_guard<std::mutex> lock(motor_gpio_init_mutex_);
        if (!motor_gpio_initialized_) {
            gpio_config_t io_conf = {};
            io_conf.intr_type = GPIO_INTR_DISABLE;
            io_conf.mode = GPIO_MODE_OUTPUT;
            io_conf.pin_bit_mask = (1ULL << MOTOR_LF_GPIO) | (1ULL << MOTOR_LB_GPIO) |
                                   (1ULL << MOTOR_RF_GPIO) | (1ULL << MOTOR_RB_GPIO);
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            if (gpio_config(&io_conf) == ESP_OK) {
                motor_gpio_initialized_ = true;
            } else {
                ESP_LOGE(TAG, "SetRealtimeMotorCommand: 电机 GPIO 初始化失败");
                return false;
            }
        }
        InitMotorPwm();
    }

    if (!motor_pwm_initialized_) {
        ESP_LOGE(TAG, "SetRealtimeMotorCommand: PWM 未初始化，忽略本次动作");
        return false;
    }

    uint32_t max_duty = (1u << pwm_resolution_bits_) - 1u;
    uint32_t duty = (uint32_t)(speed * (int)max_duty) / 100u;

    // 先把四路都缓降到 0，再按方向拉起对应两路，避免换向瞬间出现短路脉冲
    for (int i = 0; i < 4; ++i) {
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, 0, pwm_ramp_ms_);
        ledc_fade_start(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, LEDC_FADE_NO_WAIT);
    }

    // ch0=LF ch1=LB ch2=RF ch3=RB
    int a = -1, b = -1;
    switch (direction) {
        case 1: a = 0; b = 3; break;   // 右转：左轮前进 + 右轮后退
        case 2: a = 1; b = 3; break;   // 后退：两轮都后退
        case 3: a = 1; b = 2; break;   // 左转：左轮后退 + 右轮前进
        case 4: a = 0; b = 2; break;   // 前进：两轮都前进
        default: break;                // 0 或其他：保持全 0
    }
    if (a >= 0) {
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, (ledc_channel_t)a, duty, pwm_ramp_ms_);
        ledc_fade_start(LEDC_LOW_SPEED_MODE, (ledc_channel_t)a, LEDC_FADE_NO_WAIT);
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, (ledc_channel_t)b, duty, pwm_ramp_ms_);
        ledc_fade_start(LEDC_LOW_SPEED_MODE, (ledc_channel_t)b, LEDC_FADE_NO_WAIT);
    }
    return true;
}

void Application::StopRealtimeMotorControl() {
    ESP_LOGI(TAG, "StopRealtimeMotorControl");
    realtime_control_active_.store(false);
    current_motor_priority_.store(0);
    last_realtime_command_ms_.store(0);
    // 停车即清僵局计数：停了以后残留的拒单次数到下一次进模式会误触发脱困。
    motor_refusal_streak_.store(0);
    // 停车后清掉去重记录，否则下次下发同一方向会被误判成"没变化"而跳过。
    last_realtime_direction_ = -1;
    last_realtime_speed_ = -1;

    if (motor_pwm_initialized_) {
        for (int i = 0; i < 4; ++i) {
            ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, 0, pwm_ramp_ms_);
            ledc_fade_start(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, LEDC_FADE_NO_WAIT);
        }
    } else if (motor_gpio_initialized_) {
        gpio_set_level(MOTOR_LF_GPIO, 0);
        gpio_set_level(MOTOR_LB_GPIO, 0);
        gpio_set_level(MOTOR_RF_GPIO, 0);
        gpio_set_level(MOTOR_RB_GPIO, 0);
    }
}

void Application::HandleMotorActionWithDuration(int direction, int speed, int duration_ms, int priority) {
    ESP_LOGI(TAG, "Motor action: direction=%d, speed=%d, duration=%dms, priority=%d",
             direction, speed, duration_ms, priority);

    if (realtime_control_active_.load()) {
        int current_priority = current_motor_priority_.load();
        if (priority >= current_priority) {
            ESP_LOGW(TAG, "Higher/equal priority (new:%d >= current:%d), canceling previous",
                     priority, current_priority);
            StopRealtimeMotorControl();
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            ESP_LOGW(TAG, "Lower priority (new:%d < current:%d), queuing instead",
                     priority, current_priority);
            std::string desc = "Queued (dir=" + std::to_string(direction) +
                               ", speed=" + std::to_string(speed) +
                               ", dur=" + std::to_string(duration_ms) +
                               "ms, pri=" + std::to_string(priority) + ")";
            QueueMotorAction(direction, speed, duration_ms, desc);
            return;
        }
    }

    current_motor_priority_.store(priority);
    SetRealtimeMotorCommand(direction, speed);

    if (duration_ms > 0) {
        // 每个动作配一个一次性 stop 任务。2.1.0 就是这么做的：后到的任务会先把
        // 电机停一次，靠 StopRealtimeMotorControl 幂等来兜底，换向时不会卡死。
        auto stop_task = [](void* params) -> void {
            auto* p = static_cast<std::tuple<int, Application*>*>(params);
            auto [duration, app] = *p;
            delete p;
            vTaskDelay(pdMS_TO_TICKS(duration));
            app->StopRealtimeMotorControl();
            vTaskDelete(NULL);
        };
        auto* params = new std::tuple<int, Application*>(duration_ms, this);
        xTaskCreate(stop_task, "motor_stop", 2048, params, tskIDLE_PRIORITY + 1, NULL);
    }
}

void Application::QueueMotorAction(int direction, int speed, int duration_ms, const std::string& description) {
    ESP_LOGI(TAG, "Queue motor action: %s (direction=%d, speed=%d, duration=%dms)",
             description.c_str(), direction, speed, duration_ms);

    {
        std::lock_guard<std::mutex> lock(motor_queue_mutex_);
        motor_action_queue_.push({direction, speed, duration_ms, description});
    }

    if (!motor_executor_running_) {
        motor_executor_running_ = true;
        xTaskCreate([](void* param) {
            static_cast<Application*>(param)->ExecuteMotorActionQueue();
        }, "motor_executor", 4096, this, tskIDLE_PRIORITY + 1, NULL);
    }
}

void Application::ExecuteMotorActionQueue() {
    ESP_LOGI(TAG, "Motor action queue executor started");

    while (true) {
        MotorAction action;
        bool has_action = false;

        {
            std::lock_guard<std::mutex> lock(motor_queue_mutex_);
            if (!motor_action_queue_.empty()) {
                action = motor_action_queue_.front();
                motor_action_queue_.pop();
                has_action = true;
            }
        }

        if (has_action) {
            ESP_LOGI(TAG, "Executing queued motor action: %s", action.description.c_str());
            SetRealtimeMotorCommand(action.direction, action.speed);
            if (action.duration_ms > 0) {
                vTaskDelay(pdMS_TO_TICKS(action.duration_ms));
            }
            StopRealtimeMotorControl();
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void Application::TriggerMotorEmotion(int emotion_type) {
    // 情绪微动作：全部走队列，保证串行
    switch (emotion_type) {
        case 1:   // 唤醒：短促前冲
            QueueMotorAction(4, motor_action_config_.default_speed_percent, 200, "Emotion forward twitch");
            break;
        case 2:   // 说话：短促后撤
            QueueMotorAction(2, motor_action_config_.default_speed_percent, 150, "Emotion backward twitch");
            break;
        case 3:   // 顽皮/笑：左右快摆
            QueueMotorAction(3, motor_action_config_.default_speed_percent,
                             motor_action_config_.wiggle_duration_ms / 6, "Emotion left wiggle");
            QueueMotorAction(1, motor_action_config_.default_speed_percent,
                             motor_action_config_.wiggle_duration_ms / 6, "Emotion right wiggle");
            break;
        case 4:   // 喜欢/自信：轻点头
            QueueMotorAction(4, motor_action_config_.default_speed_percent,
                             motor_action_config_.forward_duration_ms / 6, "Emotion nod forward");
            break;
        case 5:   // 困惑/思考：轻微左右倾斜
            QueueMotorAction(3, motor_action_config_.default_speed_percent / 2,
                             motor_action_config_.left_turn_duration_ms / 4, "Emotion confused left");
            QueueMotorAction(1, motor_action_config_.default_speed_percent / 2,
                             motor_action_config_.right_turn_duration_ms / 4, "Emotion confused right");
            break;
        case 6:   // 惊讶/生气：后撤再前冲
            QueueMotorAction(2, motor_action_config_.default_speed_percent,
                             motor_action_config_.backward_duration_ms / 6, "Emotion shocked back");
            QueueMotorAction(4, motor_action_config_.default_speed_percent,
                             motor_action_config_.forward_duration_ms / 6, "Emotion shocked forward");
            break;
        case 7:   // 唤醒反馈：前后平衡
            QueueMotorAction(4, 40, 200, "Wake balance forward");
            QueueMotorAction(2, 40, 200, "Wake balance backward");
            break;
        case 8:   // 开始说话：前倾
            QueueMotorAction(4, 50, 250, "Start speaking forward");
            break;
        case 9:   // 结束说话：后撤
            QueueMotorAction(2, 45, 220, "End speaking backward");
            break;
        default:
            ESP_LOGW(TAG, "未知的情感动作类型: %d", emotion_type);
            break;
    }
}

void Application::LoadMotorActionConfig() {
    Settings settings("motor_config", true);

    motor_action_config_.forward_duration_ms       = settings.GetInt("forward_ms", 5000);
    motor_action_config_.backward_duration_ms      = settings.GetInt("backward_ms", 5000);
    motor_action_config_.left_turn_duration_ms     = settings.GetInt("left_turn_ms", 600);
    motor_action_config_.right_turn_duration_ms    = settings.GetInt("right_turn_ms", 600);
    motor_action_config_.spin_duration_ms          = settings.GetInt("spin_ms", 2500);
    motor_action_config_.wiggle_duration_ms        = settings.GetInt("wiggle_ms", 600);
    motor_action_config_.dance_duration_ms         = settings.GetInt("dance_ms", 1500);
    motor_action_config_.quick_forward_duration_ms = settings.GetInt("quick_fwd_ms", 5000);
    motor_action_config_.quick_backward_duration_ms= settings.GetInt("quick_bwd_ms", 5000);
    motor_action_config_.default_speed_percent     = settings.GetInt("def_speed_pct", 100);
    motor_action_config_.follow_far_mm    = settings.GetInt("follow_far_mm",  100);
    motor_action_config_.follow_near_mm   = settings.GetInt("follow_near_mm", 50);
    motor_action_config_.follow_speed     = settings.GetInt("follow_speed",   55);
    motor_action_config_.patrol_speed     = settings.GetInt("patrol_speed",   55);
    motor_action_config_.th_blocked_mm      = settings.GetInt("th_blocked",      80);
    motor_action_config_.th_blocked_exit_mm = settings.GetInt("th_blocked_exit", 100);
    motor_action_config_.th_near_mm         = settings.GetInt("th_near",         120);
    motor_action_config_.th_near_exit_mm    = settings.GetInt("th_near_exit",    140);

    ESP_LOGI(TAG, "电机配置: 前进=%d 后退=%d 左转=%d 右转=%d 转圈=%d 摆动=%d 跳舞=%d 快进=%d 快退=%d 默认速度=%d%%",
             motor_action_config_.forward_duration_ms,
             motor_action_config_.backward_duration_ms,
             motor_action_config_.left_turn_duration_ms,
             motor_action_config_.right_turn_duration_ms,
             motor_action_config_.spin_duration_ms,
             motor_action_config_.wiggle_duration_ms,
             motor_action_config_.dance_duration_ms,
             motor_action_config_.quick_forward_duration_ms,
             motor_action_config_.quick_backward_duration_ms,
             motor_action_config_.default_speed_percent);
}

void Application::SaveMotorActionConfig() {
    Settings settings("motor_config", true);

    settings.SetInt("forward_ms", motor_action_config_.forward_duration_ms);
    settings.SetInt("backward_ms", motor_action_config_.backward_duration_ms);
    settings.SetInt("left_turn_ms", motor_action_config_.left_turn_duration_ms);
    settings.SetInt("right_turn_ms", motor_action_config_.right_turn_duration_ms);
    settings.SetInt("spin_ms", motor_action_config_.spin_duration_ms);
    settings.SetInt("wiggle_ms", motor_action_config_.wiggle_duration_ms);
    settings.SetInt("dance_ms", motor_action_config_.dance_duration_ms);
    settings.SetInt("quick_fwd_ms", motor_action_config_.quick_forward_duration_ms);
    settings.SetInt("quick_bwd_ms", motor_action_config_.quick_backward_duration_ms);
    settings.SetInt("def_speed_pct", motor_action_config_.default_speed_percent);
    settings.SetInt("follow_far_mm",  motor_action_config_.follow_far_mm);
    settings.SetInt("follow_near_mm", motor_action_config_.follow_near_mm);
    settings.SetInt("follow_speed",   motor_action_config_.follow_speed);
    settings.SetInt("patrol_speed",   motor_action_config_.patrol_speed);
    settings.SetInt("th_blocked",      motor_action_config_.th_blocked_mm);
    settings.SetInt("th_blocked_exit", motor_action_config_.th_blocked_exit_mm);
    settings.SetInt("th_near",         motor_action_config_.th_near_mm);
    settings.SetInt("th_near_exit",    motor_action_config_.th_near_exit_mm);

    ESP_LOGI(TAG, "电机配置已保存");
}

void Application::SetMotorActionConfig(const MotorActionConfig& config) {
    motor_action_config_ = config;
    SaveMotorActionConfig();
}

// 板卡侧调用入口（板卡代码不认识 Application 的私有成员）
extern "C" void HandleMotorActionForApplication(int direction, int speed, int duration_ms, int priority) {
    Application::GetInstance().HandleMotorActionWithDuration(direction, speed, duration_ms, priority);
}

// === 网页遥控（第 3 步）===

// 摇杆推着走：直接改 PWM 占空比，不走动作队列。
// 摇杆每 ~100ms 发一次，入队会越积越多，表现就是松手后车还在动。
void Application::HandleWebMotorControl(int direction, int speed) {
    static int last_direction = -1;
    static int last_speed = -1;

    // 网页端会以固定频率重复发同一个方向/速度，重复的没意义
    if (direction == last_direction && speed == last_speed) {
        return;
    }
    last_direction = direction;
    last_speed = speed;

    ESP_LOGI(TAG, "Web motor: direction=%d speed=%d", direction, speed);

    if (direction == 0 || speed <= 0) {
        StopRealtimeMotorControl();
        return;
    }
    // from_web=true：Web 遥控器是手动模式，不受传感器闸门限制。
    SetRealtimeMotorCommand(direction, speed, true);
}

// web_server.cc 里的情绪动作入口。
// 进来的字符串是 web 界面用的名字（wake / happy / sad / thinking / ...），
// 转成一台动作序列：先动一下，再停。全部走队列，不阻塞 HTTP 任务。
extern "C" void HandleMotorActionForEmotion(const char* emotion) {
    if (emotion == nullptr) return;
    auto& app = Application::GetInstance();

    struct Pick { const char* name; int dir1; int dur1; int dir2; int dur2; };
    static const Pick kPicks[] = {
        // name        动作1方向/时长      动作2方向/时长
        {"wake",      4, 180,  0,   0},
        {"happy",     4, 250,  3, 200},
        {"sad",       2, 300,  0,   0},
        {"thinking",  3, 200,  1, 200},
        {"listening", 0,   0,  0,   0},   // 只靠屏幕表达，不动电机
        {"speaking",  4, 150,  0,   0},
        {"excited",   4, 200,  1, 200},
        {"loving",    4, 400,  0,   0},
        {"angry",     2, 200,  2, 200},
        {"surprised", 2, 150,  0,   0},
        {"confused",  3, 200,  0,   0},
    };

    for (const auto& p : kPicks) {
        if (strcmp(emotion, p.name) != 0) continue;
        if (p.dur1 > 0) app.QueueMotorAction(p.dir1, 80, p.dur1, p.name);
        if (p.dur2 > 0) app.QueueMotorAction(p.dir2, 80, p.dur2, p.name);
        return;
    }
}

// web_server.cc 里的跳舞入口。八步序列全部入队，HTTP 立即返回，
// 电机执行器在后台慢慢走完。
extern "C" void HandleMotorActionForDance(uint8_t speed_percent) {
    auto& app = Application::GetInstance();
    static const struct { int dir; int dur; } kSteps[] = {
        {4, 300}, {3, 250}, {1, 250}, {2, 300},
        {4, 200}, {3, 200}, {1, 200}, {4, 400},
    };
    int speed = speed_percent == 0 ? 100 : speed_percent;
    for (const auto& s : kSteps) {
        app.QueueMotorAction(s.dir, speed, s.dur, "dance");
    }
}

// ==================== 第 10 步：机器人四模式 ====================

const char* Application::RobotModeName(int mode) {
    switch (mode) {
        case kRobotExplore:    return "explore";
        case kRobotFree:       return "free";
        case kRobotFollow:     return "follow";
        case kRobotEdgePatrol: return "edge_patrol";
        default:               return "none";
    }
}

std::string Application::StartRobotMode(int mode) {
    if (mode < kRobotExplore || mode > kRobotEdgePatrol) {
        return "未知模式，只支持 explore / free / follow / edge_patrol";
    }

    if (robot_mode_.load() != kRobotNone) {
        return std::string("已经在跑了（") + RobotModeName(robot_mode_.load()) + "），先停再开";
    }

    // follow 和 edge_patrol 靠距离驱动，读不到数据就是瞎跑，直接拒绝。
    // explore 和 free 不碰传感器，传感器坏了也能跑。
    if (mode == kRobotFollow || mode == kRobotEdgePatrol) {
        SensorSnapshot snap = GetSensorSnapshot();
        if (!snap.sensor_ok) {
            return std::string(RobotModeName(mode)) + " 需要距离传感器，现在读不到数据";
        }
    }

    robot_mode_.store(mode);
    robot_paused_.store(false);
    robot_task_running_.store(true);
    // 每次进 explore 都从「还没走过」开始，否则上一次留下的方向会影响这一次。
    last_explore_dir_ = 0;
    follow_lost_count_ = 0;
    follow_search_start_ms_ = 0;

    BaseType_t ok = xTaskCreatePinnedToCore(
        RobotModeTaskEntry, "robot_mode", 4096, this, tskIDLE_PRIORITY + 2,
        &robot_task_, 0);
    if (ok != pdPASS) {
        robot_task_running_.store(false);
        robot_mode_.store(kRobotNone);
        robot_task_ = nullptr;
        return "任务创建失败，内存不足";
    }

    ESP_LOGI(TAG, "robot mode started: %s", RobotModeName(mode));

    // 进探索模式时播一次「机械臂运动完准备好」那类声音，给一段自主活动起个头。
    // 其它三个模式不放，免得吵。
    if (mode == kRobotExplore && GetDeviceState() != kDeviceStateSpeaking) {
        audio_service_.PlaySound(Lang::Sounds::OGG_READY);
    }

    SetDisplayMode(kDisplayModeEyeOnly);

    // 进模式就算一次互动，20 分钟从这里开始数。
    TouchRobotInteraction();
    return "";
}

void Application::StopRobotMode(bool return_to_default) {
    int old = robot_mode_.load();
    if (old == kRobotNone) {
        return;
    }

    robot_task_running_.store(false);
    robot_paused_.store(false);
    robot_mode_.store(kRobotNone);
    StopRealtimeMotorControl();

    // 等任务自己退出，最多 2 秒。
    // 用 tick 计数 + vTaskDelay(1)：CONFIG_FREERTOS_HZ=100 时 pdMS_TO_TICKS(5)
    // 等于 0，用 pdMS_TO_TICKS 做步长会让整个等待在几个循环内跑完。
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(2000);
    if (timeout_ticks == 0) {
        timeout_ticks = 1;
    }
    while (robot_task_ != nullptr && (xTaskGetTickCount() - start) < timeout_ticks) {
        vTaskDelay(1);
    }
    if (robot_task_ != nullptr) {
        // 停不下来就不碰它，让它自己走到循环末尾退出。
        // 强删一个正在调电机接口的任务会留下半套 PWM 状态。
        ESP_LOGW(TAG, "robot task did not exit in time, leaving it to self-delete");
        robot_task_ = nullptr;
    }

    ESP_LOGI(TAG, "robot mode stopped: %s", RobotModeName(old));

    // 退出模式回官方界面。超时那条路径传 false —— 它紧接着 EnterSleep()，
    // 这里先切一次官方界面会白闪一帧。
    if (return_to_default && display_mode_ != kDisplayModeSleep) {
        SetDisplayMode(kDisplayModeDefault);
    }
}

void Application::RobotModeTaskEntry(void* arg) {
    auto* self = static_cast<Application*>(arg);
    self->RobotModeLoop();
    self->robot_task_running_.store(false);
    self->robot_task_ = nullptr;
    vTaskDelete(nullptr);
}

void Application::RobotModeLoop() {
    int mode = robot_mode_.load();
    ESP_LOGI(TAG, "robot loop begin: %s", RobotModeName(mode));

    // 从对话回来时要先校准一次位置，否则接着跑可能撞墙。
    RunRecalibrateStep();

    while (robot_task_running_.load() && robot_mode_.load() == mode) {
        if (robot_paused_.load()) {
            // 唤醒/说话期间停手，但不动模式状态
            StopRealtimeMotorControl();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 从暂停恢复的第一轮：先原地校准一次。
        // 聊天的这几十秒里机器人可能被人挪过，直接接着跑会撞墙或掉桌沿。
        if (robot_need_recalibrate_.exchange(false)) {
            ESP_LOGI(TAG, "robot mode: resuming, recalibrating first");
            RunRecalibrateStep();
        }

        // 最高时限：机器人模式不会自己进时钟（CanEnterSleepMode 拦着），
        // 20 分钟没人理就由这里退出模式、直接进大时钟。
        // 必须走 Schedule：StopRobotMode() 要等这个任务自己退出，
        // 在本任务里直接调等于等自己，白等 2 秒还停不下来。
        const int64_t last_touch = last_robot_interaction_ms_.load();
        if (last_touch != 0 &&
            (esp_timer_get_time() / 1000) - last_touch >
                (int64_t)kRobotIdleTimeoutSeconds * 1000) {
            ESP_LOGI(TAG, "robot mode idle timeout (%d s), exiting and sleeping",
                     kRobotIdleTimeoutSeconds);
            Schedule([this]() {
                StopRobotMode(false);
                EnterSleep();
            });
            return;
        }

        // 双向锁死：前面有障碍、后面也顶着，前进和后退都被闸门拒，
        // 表现为原地僵住（日志里每 100ms 一条 refused）。
        // 连续被拒到 6 次就主动打一次转向，换朝向再试。
        if (motor_refusal_streak_.load() >= 6) {
            EscapeFromDeadlock();
            continue;
        }

        switch (mode) {
            case kRobotExplore:    RunExploreStep();    break;
            case kRobotFree:       RunFreeStep();       break;
            case kRobotFollow:     RunFollowStep();     break;
            case kRobotEdgePatrol: RunEdgePatrolStep(); break;
            default: break;
        }
        // 每步让出 CPU，别把 core 0 占死
        vTaskDelay(1);
    }
    StopRealtimeMotorControl();
    ESP_LOGI(TAG, "robot loop end: %s", RobotModeName(mode));
}

bool Application::EscapeFromDeadlock() {
    ESP_LOGW(TAG, "motors refused %d times in a row: escaping by turning in place",
             motor_refusal_streak_.load());
    motor_refusal_streak_.store(0);
    StopRealtimeMotorControl();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 先向右转一个大角度。转向不经过前后闸门（方向 1/3 只有传感器降级态才拦），
    // 正常情况下一定能下发成功。
    const int spd = motor_action_config_.patrol_speed > 0 ? motor_action_config_.patrol_speed : 50;
    if (SetRealtimeMotorCommand(1, spd)) {
        vTaskDelay(pdMS_TO_TICKS(1400));
        StopRealtimeMotorControl();
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "escape: turned right, retrying");
        return true;
    }
    // 连转向都被拒（传感器不可用降级态才可能），那就只能停下等人。
    ESP_LOGW(TAG, "escape: even turning refused, staying stopped");
    StopRealtimeMotorControl();
    vTaskDelay(pdMS_TO_TICKS(300));

    if (SetRealtimeMotorCommand(3, spd)) {
        vTaskDelay(pdMS_TO_TICKS(1400));
        StopRealtimeMotorControl();
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "escape: turned left, retrying");
        return true;
    }
    return false;
}

void Application::RunRecalibrateStep() {
    // 唤醒期间机器人可能被挪过位置，先原地转半圈重新"看"一遍。
    ESP_LOGI(TAG, "robot mode: recalibrating position");
    SetRealtimeMotorCommand(1, 50);
    vTaskDelay(pdMS_TO_TICKS(800));
    StopRealtimeMotorControl();
    vTaskDelay(pdMS_TO_TICKS(300));
}

void Application::RunExploreStep() {
    // 方向码 4 前 / 3 左 / 1 右 / 2 后。v206 的 5-13 未逆向出来，
    // 那批是更大的组合动作，这里先用 1-4，效果够用。
    static const int kDirs[] = {4, 3, 1, 2};
    // 反方向表，索引就是方向码：1=右 2=后 3=左 4=前。
    //   kOpposite[1] = 3（右的反向是左）
    //   kOpposite[2] = 4（后的反向是前）
    //   kOpposite[3] = 1（左的反向是右）
    //   kOpposite[4] = 2（前的反向是后）
    // 上一段走了「前」，这一段就不许直接「后」——纯随机时这种组合出现得
    // 不算少，看起来是站在原地前后晃，不像在探路。
    // last_explore_dir_ == 0 时 kOpposite[0] == 0，四个方向都可用。
    // 注意：这张表按「索引 = 方向码」填，不是按 1,2,3,4 的顺序填反方向值。
    static const int kOpposite[5] = {0, 3, 4, 1, 2};
    int candidates[3];
    int n = 0;
    for (int d : kDirs) {
        if (d != kOpposite[last_explore_dir_]) {
            candidates[n++] = d;
        }
    }
    int dir   = candidates[esp_random() % n];
    last_explore_dir_ = dir;
    int speed = 80 + (int)(esp_random() % 21);       // 80-100
    int dur   = 1500 + (int)(esp_random() % 501);    // 1500-2000ms

    QueueMotorAction(dir, speed, dur, "explore");

    // 探索模式的音效。用户 2026-11-11 逐个听辨后给了语义：
    //   explore1 (0008D704) 蟋蟀稀疏叫声 —— 环境音
    //   explore2 (00091F62) 扫描 / 扫描中疑问 —— 转向时
    //   explore3 (0009392B) 两节机械臂依次运行 —— 前进时
    //   explore4 (0009842E) 讲话的电子音 —— 停顿时
    // OGG_READY (00098F8D) 机械臂运动完准备好 —— 进探索模式时一次
    // 原来那版每一轮都播，而这些音效本身就有 1.4-3.7 秒长，
    // 一轮才 2.5-5 秒，等于一直在响。改成 16 秒冷却，至多 16 秒一个。
    static int64_t last_sound_ms = 0;
    int64_t now_ms = esp_timer_get_time() / 1000;
    if (now_ms - last_sound_ms >= 16000) {
        std::string_view sound{};
        if (dir == 1 || dir == 3) {
            sound = Lang::Sounds::OGG_EXPLORE2;   // 转向：扫描
        } else if (dir == 4) {
            sound = Lang::Sounds::OGG_EXPLORE3;   // 前进：机械臂
        } else {
            sound = Lang::Sounds::OGG_EXPLORE1;   // 后退：蟋蟀环境音
        }
        if (!sound.empty()) {
            last_sound_ms = now_ms;
            Schedule([this, sound]() {
                if (GetDeviceState() == kDeviceStateSpeaking) {
                    return;
                }
                audio_service_.PlaySound(sound);
            });
        }
    }

    // 动作跑完再随机歇 1-3 秒，让它看起来像在乱逛而不是抽搐
    uint32_t wait_ms = (uint32_t)dur + 1000 + (esp_random() % 2001);
    TickType_t start = xTaskGetTickCount();
    bool rest_sound_done = false;
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(wait_ms)) {
        if (!robot_task_running_.load() || robot_paused_.load()) {
            break;
        }
        // 动作走完那一刻（elapsed 刚过 dur）就是停顿的开始。
        // 用户说 explore4（1.4 秒的电子音）像机器人在讲话，放这儿正好。
        // 和起步音效各用一个 16 秒冷却，避免一停一响。
        if (!rest_sound_done &&
            (xTaskGetTickCount() - start) >= pdMS_TO_TICKS((uint32_t)dur)) {
            rest_sound_done = true;
            static int64_t last_rest_ms = 0;
            int64_t now_rest = esp_timer_get_time() / 1000;
            if (now_rest - last_rest_ms >= 16000) {
                last_rest_ms = now_rest;
                Schedule([this]() {
                    if (GetDeviceState() == kDeviceStateSpeaking) {
                        return;
                    }
                    audio_service_.PlaySound(Lang::Sounds::OGG_EXPLORE4);
                });
            }
        }
        vTaskDelay(10);
    }
}

void Application::RunFreeStep() {
    // 原地做表情，不驱动轮子
    static const char* kEmotions[] = {
        "happy", "laughing", "thinking", "surprised",
        "confused", "loving", "embarrassed"
    };
    const char* emotion = kEmotions[esp_random() % 7];
    auto display = Board::GetInstance().GetDisplay();
    if (display != nullptr) {
        display->SetEmotion(emotion);
    }
    ESP_LOGI(TAG, "free mode: emotion %s", emotion);

    uint32_t wait_ms = 1000 + (esp_random() % 2001);
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(wait_ms)) {
        if (!robot_task_running_.load() || robot_paused_.load()) {
            break;
        }
        vTaskDelay(10);
    }
}

void Application::RunFollowStep() {
    SensorSnapshot snap = GetSensorSnapshot();
    const int far_mm  = motor_action_config_.follow_far_mm;
    const int near_mm = motor_action_config_.follow_near_mm;
    const int spd     = motor_action_config_.follow_speed;

    // 有效读数：传感器在、数据新、量程状态正常、距离非零。
    // 目标侧移出视场时 VL6180X 会报 status 1（超量程）或 2（无目标），
    // 这两种都算"看不见了"，不拿它们当距离用。
    bool valid = snap.sensor_ok && snap.age_ms() <= 1000 &&
                 snap.status == 0 && snap.distance_mm > 0;

    if (valid) {
        follow_lost_count_ = 0;
        follow_search_start_ms_ = 0;
        int d = snap.distance_mm;
        if (d > far_mm) {
            SetRealtimeMotorCommand(4, spd);     // 远，慢慢追
        } else if (d >= near_mm) {
            StopRealtimeMotorControl();          // 中距，保持
        } else {
            SetRealtimeMotorCommand(2, spd);     // 太近，退
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }

    // ---- 读不到：先确认不是偶发抖一下，再进扫描 ----
    if (++follow_lost_count_ < 10) {          // 10 轮 × 100ms ≈ 1 秒
        StopRealtimeMotorControl();
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }

    int64_t now = esp_timer_get_time() / 1000;
    if (follow_search_start_ms_ == 0) {
        follow_search_start_ms_ = now;
        ESP_LOGW(TAG, "follow: target lost, start scanning");
    }

    // 扫一圈还没找到就停手，别一直转到没电
    if (now - follow_search_start_ms_ > 12000) {
        ESP_LOGW(TAG, "follow: scan timeout after %lld ms, giving up",
                 (long long)(now - follow_search_start_ms_));
        StopRealtimeMotorControl();
        follow_search_start_ms_ = 0;
        follow_lost_count_ = 0;
        vTaskDelay(pdMS_TO_TICKS(500));
        return;
    }

    // 原地右转找目标。转的过程中每 100ms 会重新进这个函数，
    // 一旦读到有效距离就走上面的 valid 分支，自然停下。
    SetRealtimeMotorCommand(1, spd);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void Application::RunEdgePatrolStep() {
    SensorSnapshot snap = GetSensorSnapshot();
    if (!snap.sensor_ok || snap.age_ms() > 1000) {
        StopRealtimeMotorControl();
        vTaskDelay(pdMS_TO_TICKS(200));
        return;
    }

    if (snap.edge == 2) {
        ESP_LOGW(TAG, "edge_patrol: edge detected, retreat then turn 90 deg");
        StopRealtimeMotorControl();
        vTaskDelay(pdMS_TO_TICKS(200));

        // 先试着退。后头要是也有人（或者红外模块测到墙），这一条会被闸门拒掉，
        // 那就别再空等 600ms——直接原地大角度转向换个朝向。
        const bool retreat_ok = SetRealtimeMotorCommand(2, motor_action_config_.patrol_speed);
        if (retreat_ok) {
            vTaskDelay(pdMS_TO_TICKS(600));
            StopRealtimeMotorControl();
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            ESP_LOGW(TAG, "edge_patrol: retreat refused (obstacle behind), turning in place");
        }

        // 转 90 度。退不了的时候这一步是唯一的出路，所以给足时间；
        // 退成功时仍然用原来的 900ms，保持原来的转弯幅度。
        const uint32_t turn_ms = retreat_ok ? 900 : 1400;
        SetRealtimeMotorCommand(1, motor_action_config_.patrol_speed);   // 转
        vTaskDelay(pdMS_TO_TICKS(turn_ms));
        StopRealtimeMotorControl();
        vTaskDelay(pdMS_TO_TICKS(200));
        return;
    }

    SetRealtimeMotorCommand(4, motor_action_config_.patrol_speed);   // 没到沿就慢慢直行（40 带不动电机）
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ---- 第 10 步：给 Web 接口用的三个入口 ----
// 返回值指向静态缓冲，调用方要在同一个 httpd 任务里立刻拷走。
extern "C" const char* RobotModeNameOf(int mode) {
    return Application::RobotModeName(mode);
}

// ===== Web 控制台的「系统」页面桥接 =====
// web_server.cc 不 include application.h（会成环），所以这里把需要的能力
// 用 extern "C" 暴露出去。

// 模拟 BOOT 键。短按 = 开/关对话；长按 = 眼睛 ↔ 官方界面互切。
// 跟 boards/deepseeking-robot/deepseeking_robot.cc 里实体键的行为保持一致。
extern "C" void HandleSystemBoot(int long_press) {
    auto& app = Application::GetInstance();
    if (app.GetDeviceState() == kDeviceStateStarting) {
        // 开机阶段按 BOOT 是进配网，网页上不开放这个，直接忽略。
        ESP_LOGW(TAG, "web boot: ignored while starting");
        return;
    }
    if (long_press) {
        ESP_LOGI(TAG, "web boot: long press -> toggle display mode");
        app.ToggleDisplayMode();
    } else {
        ESP_LOGI(TAG, "web boot: short press -> toggle chat");
        app.ToggleChatState();
    }
}

// Web 用来显示状态的只读查询。合在 /api/robot 的响应里，省一个 URI 槽位。
extern "C" int HandleDeviceStateQuery() {
    return (int)Application::GetInstance().GetDeviceState();
}

extern "C" const char* HandleDisplayModeQuery() {
    switch (Application::GetInstance().GetDisplayMode()) {
        case kDisplayModeSleep:  return "sleep";
        case kDisplayModeEyeOnly: return "eyes";
        case kDisplayModeMusic:  return "music";
        default:                 return "default";
    }
}

// 软件重启。等价于按 RESET 键（EN 脚拉低），但走的是 esp_restart()。
extern "C" void HandleSystemRestart() {
    ESP_LOGW(TAG, "web restart requested");
    esp_restart();
}

// 按住说话。start=1 开始听，start=0 停止并发送。
// 对应实体触摸键 GPIO47 的 OnPressDown / OnPressUp。
// HTTP 没有"按住"概念，所以由前端在 mousedown/mouseup 各发一次。
extern "C" void HandleSystemListen(int start) {
    auto& app = Application::GetInstance();
    if (start) {
        ESP_LOGI(TAG, "web listen: start");
        app.StartListening();
    } else {
        ESP_LOGI(TAG, "web listen: stop");
        app.StopListening();
    }
}

extern "C" const char* HandleRobotModeStart(const char* mode) {
    static char err_buf[128];
    if (mode == nullptr) {
        return "缺少 mode 参数";
    }
    int m = kRobotNone;
    if (strcmp(mode, "explore") == 0) {
        m = kRobotExplore;
    } else if (strcmp(mode, "free") == 0) {
        m = kRobotFree;
    } else if (strcmp(mode, "follow") == 0) {
        m = kRobotFollow;
    } else if (strcmp(mode, "edge_patrol") == 0) {
        m = kRobotEdgePatrol;
    } else {
        return "未知模式，只支持 explore / free / follow / edge_patrol";
    }
    std::string err = Application::GetInstance().StartRobotMode(m);
    if (err.empty()) {
        err_buf[0] = '\0';
        return err_buf;
    }
    snprintf(err_buf, sizeof(err_buf), "%s", err.c_str());
    return err_buf;
}

extern "C" const char* HandleRobotModeStop() {
    auto& app = Application::GetInstance();
    if (app.GetRobotMode() == kRobotNone) {
        return "没在跑任何模式";
    }
    app.StopRobotMode();
    return "已停止";
}

extern "C" int HandleRobotModeQuery() {
    return Application::GetInstance().GetRobotMode();
}

// === 7b：音量与静音 ===
// 音量由 AudioCodec 自己存 NVS（键 output_volume，默认 70）。
// 不能用 0 表示静音：audio_codec.cc 看到 <= 0 会警告并强制改成 10。
// 所以静音时把原值记在 volume_before_mute_ 里，音量设成 10，再按一次恢复。

extern "C" int HandleVolumeGet() {
    auto codec = Board::GetInstance().GetAudioCodec();
    if (codec == nullptr) {
        return -1;
    }
    return codec->output_volume();
}

extern "C" int HandleVolumeSet(int volume) {
    auto codec = Board::GetInstance().GetAudioCodec();
    if (codec == nullptr) {
        return -1;
    }
    if (volume < 10) {
        volume = 10;
    }
    if (volume > 100) {
        volume = 100;
    }
    // 手动调过音量就不再算静音态，否则下次点静音按钮会拿这个新值当"原值"。
    Application::GetInstance().SetVolumeBeforeMute(-1);
    codec->SetOutputVolume(volume);
    return codec->output_volume();
}

extern "C" int HandleVolumeToggleMute() {
    auto codec = Board::GetInstance().GetAudioCodec();
    if (codec == nullptr) {
        return -1;
    }
    auto& app = Application::GetInstance();
    int saved = app.GetVolumeBeforeMute();
    if (saved < 0) {
        // 现在没静音，进静音
        int cur = codec->output_volume();
        app.SetVolumeBeforeMute(cur < 10 ? 10 : cur);
        codec->SetOutputVolume(10);
    } else {
        app.SetVolumeBeforeMute(-1);
        codec->SetOutputVolume(saved);
    }
    return codec->output_volume();
}

extern "C" bool HandleVolumeIsMuted() {
    return Application::GetInstance().GetVolumeBeforeMute() >= 0;
}
