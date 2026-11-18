#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "rgb_strip.h"
#include "infrared_avoid.h"
#include "peripherals/vl6180x.h"
#include "i2c_panel_io_timeout.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <wifi_manager.h>
#include <cJSON.h>
#include <cstring>
#include <cstdio>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "XiaozhiAiBoard"

// 给 web_server 用的句柄。web_server 不 include 板卡头文件，
// 只通过下面这组 extern "C" 函数访问灯带。
static RgbStrip* g_rgb_strip_for_web = nullptr;

// 第 1 步：电机动作统一走 Application 的 PWM 中枢（DRV8833 双 H 桥，GPIO 12/13/14/21）
// direction: 4=前进 2=后退 3=左转 1=右转 0=停
// priority:  0=空闲微动作 1=情绪反馈 2=用户明确指令
extern "C" void HandleMotorActionForApplication(int direction, int speed, int duration_ms, int priority);

class XiaozhiAiBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    // 传感器独立走 I2C1（GPIO1=SCL / GPIO2=SDA），不跟屏幕抢总线。
    // v206 把两者放在同一条 400kHz 线上，屏幕 30fps 刷屏吃掉 69% 带宽，
    // 测距超时率 87%。拆开之后这条限制就没有了。
    i2c_master_bus_handle_t sensor_i2c_bus_ = nullptr;
    std::unique_ptr<Vl6180xSensor> sensor_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;

    // WS2812 灯带（GPIO38，5 颗）。和板载那颗（GPIO48，SingleLed）是两套独立实例，
    // 各占一个 RMT 通道，互不影响。
    std::unique_ptr<RgbStrip> rgb_strip_;
    std::unique_ptr<InfraredAvoidSensor> infrared_avoid_;

    void InitializeRgbStrip() {
        // 灯珠数量可以从 Web 改并存 NVS，所以上电时以存下来的为准，
        // 没存过才用 config.h 里的编译期默认值。
        // 第三个参数是物理长度：RMT 按它发数据，多出来的每帧写黑。
        const uint16_t count = RgbStrip::LoadLedCount(WS2812_LED_COUNT);
        rgb_strip_ = std::make_unique<RgbStrip>(WS2812_GPIO, count, WS2812_STRIP_LENGTH);
        g_rgb_strip_for_web = rgb_strip_.get();
        ESP_LOGI(TAG, "LED count %u / strip length %u (default %d)",
                 (unsigned)count, (unsigned)rgb_strip_->StripLength(), (int)WS2812_LED_COUNT);
        rgb_strip_->LoadFromNvs();
        // 上电先亮"启动中"的效果，等设备进 idle 再切到空闲效果
        // 开机不主动亮灯：之前调的是 kSlotStartupInitial（青色常亮），
        // 用户关灯后一重启又亮起来，像是没关掉。改走 idle 槽位（灭灯）。
        rgb_strip_->ApplySlot(RgbStrip::kSlotStartupIdle);

        // 状态 → 灯带槽位。listening/speaking 两个槽位早就定义了却一直没人用，
        // 因为板卡收不到状态变化（Board 基类只有 Led 才有 OnStateChanged）。
        // 这里让 Application 把状态转出来，说话时走语音律动，聆听时灭灯。
        // 只处理这两个状态：idle 不碰，好让手动设的灯色留得住。
        Application::GetInstance().SetLedStateListener([](int state) {
            auto* strip = g_rgb_strip_for_web;
            if (strip == nullptr) {
                return;
            }
            // 用户手动设过效果就不要再抢他的：说话/聆听都不覆盖，
            // 直到他在页面上点"恢复随状态联动"（那会清掉这个标志）。
            if (strip->ManualOverride()) {
                return;
            }
            switch (static_cast<DeviceState>(state)) {
                case kDeviceStateSpeaking:
                    strip->ApplySlot(RgbStrip::kSlotSpeaking);
                    break;
                case kDeviceStateListening:
                    strip->ApplySlot(RgbStrip::kSlotListening);
                    break;
                default:
                    break;
            }
        });
    }

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSensorI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = SENSOR_I2C_SDA_PIN,
            .scl_io_num = SENSOR_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        esp_err_t err = i2c_new_master_bus(&bus_config, &sensor_i2c_bus_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "sensor i2c bus init failed: %s", esp_err_to_name(err));
            sensor_i2c_bus_ = nullptr;
        }
    }

    void InitializeSensor() {
        if (sensor_i2c_bus_ == nullptr) {
            ESP_LOGW(TAG, "sensor bus unavailable, skipping VL6180X");
            return;
        }
        sensor_ = std::make_unique<Vl6180xSensor>();
        if (!sensor_->Begin(sensor_i2c_bus_)) {
            // 读不到不算致命：板子照常启动，测距类和依赖距离的机器人模式会报"没有传感器"。
            ESP_LOGW(TAG, "VL6180X not detected, distance features disabled");
            sensor_.reset();
            return;
        }
        sensor_->Start();
        // 交给 Application，Web 和模式逻辑都从它那儿取快照。
        Application::GetInstance().AttachSensor(sensor_.get());
    }

    // 红外对射避障：只给「挡了 / 没挡」这一个信号，不给距离。
    // 和 VL6180X 不冲突——那个管桌沿和精确距离，这个管太快、太黑、
    // 太近（近到激光测距反而读不出来）的场合。
    void InitializeInfraredAvoid() {
        infrared_avoid_ = std::make_unique<InfraredAvoidSensor>();
        if (!infrared_avoid_->Begin(INFRARED_AVOID_GPIO, INFRARED_AVOID_ACTIVE_LOW != 0)) {
            ESP_LOGW(TAG, "infrared avoid init failed on GPIO%d, backward guard disabled",
                     static_cast<int>(INFRARED_AVOID_GPIO));
            infrared_avoid_.reset();
            return;
        }
        // 包成 lambda 交给 Application：那边不 include 板卡头文件，
        // 电机闸门和看门任务都只认 std::function<bool()>。
        Application::GetInstance().AttachObstacleProbe(
            [this]() { return infrared_avoid_->IsBlocked(); });
        ESP_LOGI(TAG, "infrared avoid ready: GPIO%d, active %s",
                 static_cast<int>(INFRARED_AVOID_GPIO),
                 INFRARED_AVOID_ACTIVE_LOW ? "low" : "high");
    }

    // SSD1306 与 SH1106 共用这套 IO 配置，只有 panel 构造函数不同
    void InitializeOledDisplay() {
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
            .scl_speed_hz = 400 * 1000,
        };

        // 用自己实现的 panel io：IDF 原版把 I2C 超时硬编码成 -1（永久等待），
        // 总线被占死时刷屏任务会卡在里面出不来，析构等不到它退出。
        // 详见 i2c_panel_io_timeout.h 开头的说明。100ms 对 400kHz 刷一屏（约 23ms）留了四倍余量。
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_timeout(display_i2c_bus_, &io_config, 100, &panel_io_));

#ifdef SH1106
        ESP_LOGI(TAG, "Install SH1106 driver");
#else
        ESP_LOGI(TAG, "Install SSD1306 driver");
#endif
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

#ifdef SH1106
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
        ESP_LOGI(TAG, "SH1106 initialized with corrected display settings");
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
        ESP_LOGI(TAG, "SSD1306 driver installed");
#endif

        // 官方原版这里是 ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_))，
        // OLED 不回 ACK 时会直接 abort 重启。这里降级成可恢复，面板没接也能跑起来。
        if (esp_lcd_panel_reset(panel_) != ESP_OK || esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display, falling back to NoDisplay");
            esp_lcd_panel_del(panel_);
            esp_lcd_panel_io_del(panel_io_);
            panel_ = nullptr;
            panel_io_ = nullptr;
            display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));

        ESP_LOGI(TAG, "Turning display on");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                   DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        ESP_LOGI(TAG, "Display ready: %dx%d", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();

            // 开机前 9.6 秒（starting）按 BOOT 进配网，这是官方原有语义
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }

            // 短按就是「说话键」：
            //   睡眠中（大时钟）→ 先唤醒（切到眼睛），再开始对话，一步到位
            //   其它状态 → 开始 / 结束对话（说话中则打断）
            // 不再用手动进睡眠——60 秒不动会自动进，真要用长按也能切界面，
            // 把它占在短按上反而挤掉了最常用的动作。
            if (app.GetDisplayMode() == kDisplayModeSleep) {
                app.WakeUp();
            }
            app.ToggleChatState();
        });

        // 长按 1 秒在「眼睛」和「官方界面」之间切。
        // 官方界面就是小智原本那套 UI：顶部 WiFi/静音/电量、中间状态字与字幕，
        // 主要用途是看它刚才说了什么（字幕区就是 chat_message_label_）。
        boot_button_.OnLongPress([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                return;
            }
            app.ToggleDisplayMode();
        });

        touch_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    // ===== 第 1 步：电机动作封装 =====
    // 全部转发到 Application::HandleMotorActionWithDuration，priority=2 表示用户明确指令，
    // 会抢占正在执行的低优先级动作（空闲微动作 priority=0、情绪反馈 priority=1）
    void MotorMoveForward(int duration_ms = 5000, uint8_t speed_percent = 100) {
        HandleMotorActionForApplication(4, speed_percent, duration_ms, 2);
    }
    void MotorMoveBackward(int duration_ms = 5000, uint8_t speed_percent = 100) {
        HandleMotorActionForApplication(2, speed_percent, duration_ms, 2);
    }
    void MotorTurnLeft(int duration_ms = 600, uint8_t speed_percent = 100) {
        HandleMotorActionForApplication(3, speed_percent, duration_ms, 2);
    }
    void MotorTurnRight(int duration_ms = 600, uint8_t speed_percent = 100) {
        HandleMotorActionForApplication(1, speed_percent, duration_ms, 2);
    }
    void MotorStop() {
        HandleMotorActionForApplication(0, 0, 0, 2);
    }
    void MotorSpinAround(uint8_t speed_percent = 100) {
        HandleMotorActionForApplication(3, speed_percent, 2000, 2);   // 原地左转 2 秒 ≈ 一圈
    }
    void MotorQuickForward(uint8_t speed_percent = 100) {
        HandleMotorActionForApplication(4, speed_percent, 500, 2);
    }
    void MotorQuickBackward(uint8_t speed_percent = 100) {
        HandleMotorActionForApplication(2, speed_percent, 500, 2);
    }
    void MotorWiggle(uint8_t speed_percent = 100) {
        HandleMotorActionForApplication(1, speed_percent, 300, 2);    // 右扭一下
    }

    // 8 步舞蹈：左右交替 + 前后，每步之间留出停顿让人看清
    void MotorDance(uint8_t speed_percent = 100) {
        struct Step { int dir; int dur; int gap; };
        static const Step kSteps[] = {
            {4, 300, 120}, {3, 250, 120}, {1, 250, 120}, {2, 300, 150},
            {4, 200, 100}, {3, 200, 100}, {1, 200, 100}, {4, 400,   0},
        };
        for (const auto& s : kSteps) {
            HandleMotorActionForApplication(s.dir, speed_percent, s.dur, 2);
            vTaskDelay(pdMS_TO_TICKS(s.dur + s.gap));
        }
    }

    // 随机挑一个动作 + 返回名字，给 self.motor.random_emotion 用
    std::string MotorRandomEmotion(uint8_t speed_percent = 100) {
        struct Pick { const char* name; int dir; int dur; int gap; };
        static const Pick kPicks[] = {
            {"happy",     4, 250, 150},
            {"sad",       2, 300, 150},
            {"thinking",  3, 200, 200},
            {"excited",   1, 200, 150},
            {"surprised", 2, 150, 100},
            {"loving",    4, 400, 150},
            {"confused",  3, 200, 150},
            {"angry",     2, 200, 100},
        };
        static constexpr int kCount = sizeof(kPicks) / sizeof(kPicks[0]);
        const Pick& p = kPicks[esp_random() % kCount];
        HandleMotorActionForApplication(p.dir, speed_percent, p.dur, 2);
        vTaskDelay(pdMS_TO_TICKS(p.dur + p.gap));
        HandleMotorActionForApplication(0, 0, 0, 2);
        return std::string(p.name);
    }

    // 0a 阶段只保留官方自带的 lamp；第 1 步起接入电机
    // ── 情绪动作 ────────────────────────────────────────────────
    // 全部入队，不在这里 vTaskDelay 阻塞。Application 里有一个
    // motor_executor 任务在串行消费队列，每步执行完再等 50ms 才取下一个，
    // 所以序列天然是"做一步、停一下、再做下一步"。
    //
    // 动作序列搬自 2.1.0 的 boards/qebabe-xiaoche/compact_wifi_board.cc
    // 里的 OnXxx() 系列（PerformMotorAction 的 action 编号和 direction
    // 编号不是一回事：action1→dir4 前进、action2→dir2 后退、
    // action3→dir3 左转、action4→dir1 右转）。
    static constexpr int kEmotionSpeed = 80;
    static constexpr int kEmotionPriority = 1;

    void EmotionStep(int direction, int duration_ms) {
        Application::GetInstance().QueueMotorAction(
            direction, kEmotionSpeed, duration_ms, "emotion");
    }

    void InitializeTools() {
        // 不要注册 self.lamp.*。
        // 那是官方面包板上的单色 LED（LAMP_GPIO=18），这台机器没接那颗灯。
        // 留着它的代价很直接：LLM 看到 "Turn on the lamp" 就去调 self.lamp.turn_on，
        // 用户说"开彩灯"它调 lamp，屏上回"开好了"，灯带却纹丝不动。
        // static LampController lamp(LAMP_GPIO);

        auto& mcp_server = McpServer::GetInstance();
        // ---- 运动：10 合 1 ----
        // 服务端只接受 32 个 MCP 工具（超了会报 "The number of tools has
        // reached the limit: 32" 并丢掉多余的工具）。原本 10 个动作各占一个名额，
        // 加上情绪、彩灯、官方 common 和 user-only，预算正好撑满，后面测距和
        // 机器人模式就没位置了。合并成一个带 action 参数的工具，能力一点没少。
        mcp_server.AddTool("self.motor.move",
            "Arm the robot to perform an action.\n"
            "Args:\n"
            "  `action`: one of\n"
            "      forward         move forward, default 5000ms\n"
            "      backward        move backward, default 5000ms\n"
            "      turn_left       turn left, default 600ms\n"
            "      turn_right      turn right, default 600ms\n"
            "      spin            spin in place, default 2000ms\n"
            "      quick_forward   short dash forward, 400ms\n"
            "      quick_backward  short dash backward, 400ms\n"
            "      wiggle          wag left and right\n"
            "      dance           eight-step dance\n"
            "      stop            stop immediately\n"
            "  `speed_percent`: motor speed 0-100, default 100\n"
            "  `duration_ms`: duration in ms, 0 means use the action's default\n"
            "Return:\n"
            "  Result message",
            PropertyList({
                Property("action", kPropertyTypeString, std::string("forward")),
                Property("speed_percent", kPropertyTypeInteger, 100, 0, 100),
                Property("duration_ms", kPropertyTypeInteger, 0, 0, 10000)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string action = properties["action"].value<std::string>();
                int speed = properties["speed_percent"].value<int>();
                int duration = properties["duration_ms"].value<int>();

                auto pick = [duration](int fallback) -> int {
                    return duration > 0 ? duration : fallback;
                };

                if (action == "forward") {
                    MotorMoveForward(pick(5000), speed);
                } else if (action == "backward") {
                    MotorMoveBackward(pick(5000), speed);
                } else if (action == "turn_left") {
                    MotorTurnLeft(pick(600), speed);
                } else if (action == "turn_right") {
                    MotorTurnRight(pick(600), speed);
                } else if (action == "spin") {
                    HandleMotorActionForApplication(3, speed, pick(2000), 2);
                } else if (action == "quick_forward") {
                    MotorQuickForward(speed);
                } else if (action == "quick_backward") {
                    MotorQuickBackward(speed);
                } else if (action == "wiggle") {
                    MotorWiggle(speed);
                } else if (action == "dance") {
                    MotorDance(speed);
                } else if (action == "stop") {
                    MotorStop();
                } else {
                    return std::string("unknown action: ") + action;
                }
                return std::string("action: ") + action + " at " + std::to_string(speed) + "%";
            });


        // ---- 情绪：11 合 1 ----
        // 服务端只接受 32 个 MCP 工具（超了会报 "The number of tools has
        // reached the limit: 32" 并丢掉多余的工具），原本 11 个独立的情绪工具
        // 加起来就把预算撑爆了。合并成一个带名字参数的工具，能力一点没少。
        mcp_server.AddTool("self.motor.emotion",
            "做一个小幅度的情绪动作。做情绪时调这个，不要用 move_forward 之类拼凑。\n"
            "Args:\n"
            "  `name`: 情绪名，可选 wake_up / happy / sad / thinking / listening /\n"
            "          speaking / excited / loving / angry / surprised / confused / random\n"
            "Return:\n"
            "  执行结果\n"
            "动作会串行排进电机队列，不会打断正在说的语音。",
            PropertyList({
                Property("name", kPropertyTypeString, std::string("happy")),
                Property("speed_percent", kPropertyTypeInteger, 100, 0, 100)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string name = properties["name"].value<std::string>();
                if (name == "random") {
                    return MotorRandomEmotion(properties["speed_percent"].value<int>());
                }
                if (name == "wake_up")        { EmotionStep(4, 300); }
                else if (name == "happy")     { EmotionStep(4, 200); EmotionStep(3, 200); }
                else if (name == "sad")       { EmotionStep(2, 400); }
                else if (name == "thinking")  { EmotionStep(3, 150); EmotionStep(1, 150); }
                else if (name == "listening") { EmotionStep(3, 100); EmotionStep(1, 100); }
                else if (name == "speaking")  { EmotionStep(4, 250); }
                else if (name == "excited")   { EmotionStep(4, 150); EmotionStep(3, 150); EmotionStep(1, 150); }
                else if (name == "loving")    { EmotionStep(4, 300); EmotionStep(3, 200); }
                else if (name == "angry")     { EmotionStep(2, 200); EmotionStep(4, 200); }
                else if (name == "surprised") { EmotionStep(2, 100); EmotionStep(4, 200); }
                else if (name == "confused")  { EmotionStep(3, 100); EmotionStep(1, 100); EmotionStep(3, 100); }
                else {
                    return std::string("unknown emotion: ") + name;
                }
                return std::string("emotion: ") + name;
            });

        mcp_server.AddTool("self.network.get_ip",
            "获取当前WiFi IP地址信息，用于语音播报或状态查询",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                auto& wifi = WifiManager::GetInstance();
                std::string ip = wifi.GetIpAddress();
                if (ip.empty()) {
                    return std::string("当前未连接到WiFi网络，无法获取IP地址");
                }
                return std::string("当前IP地址是") + ip;
            });

        ESP_LOGI(TAG, "MCP 工具注册完成：12 个（motor.move + motor.emotion + get_ip + 9 彩灯）");

        // ---- 6 个彩灯工具（v206 的 self.rgb_led.*）----

        mcp_server.AddTool("self.rgb_led.set_color",
            "设置彩灯颜色，全部灯珠显示同一颜色。\n"
            "Args:\n"
            "  `red`: 红色分量 (0-255)\n"
            "  `green`: 绿色分量 (0-255)\n"
            "  `blue`: 蓝色分量 (0-255)\n"
            "Return:\n"
            "  设置结果",
            PropertyList({
                Property("red", kPropertyTypeInteger, 80, 0, 255),
                Property("green", kPropertyTypeInteger, 80, 0, 255),
                Property("blue", kPropertyTypeInteger, 80, 0, 255)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int r = properties["red"].value<int>();
                int g = properties["green"].value<int>();
                int b = properties["blue"].value<int>();
                // 纯白（三通道相等且偏高）直接按 255,255,255 下发太刺眼，
                // 电流也大。这里压到 1/3，视觉上仍是白的，功耗降一个数量级。
                if (r == g && g == b && r > 90) {
                    r = g = b = 85;
                }
                rgb_strip_->SetAll(StripColor{(uint8_t)r, (uint8_t)g, (uint8_t)b});
                return std::string("彩灯已设为 RGB(") + std::to_string(r) + "," +
                       std::to_string(g) + "," + std::to_string(b) + ")";
            });

        mcp_server.AddTool("self.rgb_led.turn_off",
            "关闭所有彩灯。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                // 先停掉正在跑的特效（彩虹/呼吸/脉冲/闪烁），否则下一次刷新
                // 会把灭灯的结果盖回去——之前表现为"关灯要连说几遍才生效"。
                rgb_strip_->StopEffect();
                vTaskDelay(pdMS_TO_TICKS(60));
                rgb_strip_->TurnOff();
                return std::string("彩灯已关闭");
            });

        mcp_server.AddTool("self.rgb_led.turn_on",
            "打开彩灯，恢复到开机时的颜色（默认是柔和青色）。\n"
            "如果之前用 set_color 设过颜色，会恢复那一组。\n"
            "Return:\n"
            "  当前颜色 gcode",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                rgb_strip_->StopEffect();
                vTaskDelay(pdMS_TO_TICKS(60));
                rgb_strip_->ApplySlot(RgbStrip::kSlotStartupInitial);
                return std::string("彩灯已打开：") +
                       std::string(rgb_strip_->GetSlotGcode(RgbStrip::kSlotStartupInitial));
            });

        mcp_server.AddTool("self.rgb_led.blink",
            "让彩灯按指定颜色闪烁若干次。\n"
            "Args:\n"
            "  `times`: 闪烁次数 (1-50)\n"
            "  `interval_ms`: 一次亮灭的总时长毫秒数 (100-5000)\n"
            "  `red`: 红色分量 (0-255)\n"
            "  `green`: 绿色分量 (0-255)\n"
            "  `blue`: 蓝色分量 (0-255)",
            PropertyList({
                Property("times", kPropertyTypeInteger, 3, 1, 50),
                Property("interval_ms", kPropertyTypeInteger, 500, 100, 5000),
                Property("red", kPropertyTypeInteger, 255, 0, 255),
                Property("green", kPropertyTypeInteger, 0, 0, 255),
                Property("blue", kPropertyTypeInteger, 0, 0, 255)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int times = properties["times"].value<int>();
                int interval = properties["interval_ms"].value<int>();
                int r = properties["red"].value<int>();
                int g = properties["green"].value<int>();
                int b = properties["blue"].value<int>();
                rgb_strip_->Blink(StripColor{(uint8_t)r, (uint8_t)g, (uint8_t)b}, times, interval);
                return std::string("彩灯开始闪烁 ") + std::to_string(times) + " 次";
            });

        mcp_server.AddTool("self.rgb_led.rainbow",
            "让彩灯显示彩虹颜色循环。\n"
            "Args:\n"
            "  `speed_ms`: 每帧步进的毫秒数 (10-500)，越小越快",
            PropertyList({
                Property("speed_ms", kPropertyTypeInteger, 50, 10, 500)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int speed = properties["speed_ms"].value<int>();
                rgb_strip_->Rainbow(speed, 0);   // 0 = 无限循环，不会转一圈就停
                return std::string("彩灯开始彩虹循环");
            });

        mcp_server.AddTool("self.rgb_led.set_brightness",
            "调节彩灯亮度。\n"
            "每个颜色通道硬上限是 100（0-255 量程），这一条无法超过。\n"
            "这个百分比是在硬上限之内继续缩小，100 = 只吃硬上限。\n"
            "会存入 NVS，重启后保持。\n"
            "Args:\n"
            "  `percent`: 亮度百分比 (1-100)，默认 100，嫌亮可说 30-50",
            PropertyList({
                Property("percent", kPropertyTypeInteger, 100, 1, 100)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int percent = properties["percent"].value<int>();
                rgb_strip_->SetBrightness(percent);
                rgb_strip_->SaveToNvs();
                return std::string("彩灯亮度已设为 ") +
                       std::to_string(RgbStrip::GetBrightness()) + "%";
            });

        mcp_server.AddTool("self.rgb_led.custom",
            "用格式指令自定义彩灯效果。\n"
            "语法：<LED数量>,<速度ms>,<特效>[,R,G,B]\n"
            "  特效 W = 常亮（需跟 R,G,B）\n"
            "       K = 呼吸（需跟 R,G,B）\n"
            "       P = 脉冲（可选 R,G,B）\n"
            "       R = 彩虹（不带 RGB）\n"
            "       N = 灭灯\n"
            "例如 \"5,50,W,255,0,0\" 表示 5 颗红灯常亮。\n"
            "Args:\n"
            "  `gcode`: 格式指令字符串",
            PropertyList({
                Property("gcode", kPropertyTypeString)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string gcode = properties["gcode"].value<std::string>();
                std::string err;
                if (!rgb_strip_->ApplyGcode(gcode, &err)) {
                    return std::string("错误: ") + err;
                }
                return std::string("彩灯指令已执行: ") + gcode;
            });

        mcp_server.AddTool("self.rgb_led.restore_default_idle",
            "把彩灯恢复成默认的空闲效果，并清除当前会话中的自定义灯效覆盖。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                rgb_strip_->ApplySlot(RgbStrip::kSlotStartupIdle);
                return std::string("彩灯已恢复默认空闲效果");
            });

        mcp_server.AddTool("self.rgb_led.get_state",
            "读取彩灯当前状态。返回：当前亮度百分比（1-100，每个颜色通道的硬上限是 100）、"
            "当前正在显示的效果 gcode、灯珠数量、以及五个槽位各自存的灯效。"
            "被问「灯现在多亮」「现在是什么灯效」时用这个，不要猜。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                auto root = cJSON_CreateObject();
                cJSON_AddNumberToObject(root, "brightness", RgbStrip::GetBrightness());
                cJSON_AddStringToObject(root, "current_gcode", rgb_strip_->GetCurrentGcode().c_str());
                cJSON_AddNumberToObject(root, "led_count", rgb_strip_->LedCount());
                auto slots = cJSON_AddObjectToObject(root, "slots");
                for (int i = 0; i < RgbStrip::kSlotCount; ++i) {
                    auto slot = static_cast<RgbStrip::Slot>(i);
                    cJSON_AddStringToObject(slots, RgbStrip::SlotName(slot),
                                            rgb_strip_->GetSlotGcode(slot).c_str());
                }
                return root;
            });

        ESP_LOGI(TAG, "彩灯工具注册完成：7 个（set_color/turn_off/blink/rainbow/custom/restore_default_idle/get_state）");

        // ---- 2 个测距工具 ----

        mcp_server.AddTool("self.sensor.get_distance",
            "读取激光测距传感器的当前距离。返回 distance_mm（毫米）、status（ok/out_of_range/no_target/i2c_error/not_ready）、"
            "age_ms（数据距今多少毫秒）、obstacle（0 通畅 / 1 靠近 / 2 停车）、"
            "edge（0 未知 / 1 在桌上 / 2 桌沿）。\n"
            "重要：如果 age_ms 大于 1000，说明传感器没有新数据，"
            "应当如实告诉用户「传感器暂时没有数据」，不要拿旧读数当作当前距离去推断。",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                auto snap = Application::GetInstance().GetSensorSnapshot();
                if (!snap.sensor_ok) {
                    return std::string("距离传感器当前不可用，读不到距离。");
                }
                int age = snap.age_ms();
                if (age > 1000) {
                    return std::string("距离传感器没有新数据（上次采样在 ") + std::to_string(age / 1000) + " 秒前）。";
                }
                static const char* kStatusName[] = {"正常", "超出量程", "目标过弱", "I2C 错误", "未就绪"};
                const char* st = (snap.status >= 0 && snap.status <= 4) ? kStatusName[snap.status] : "未知";
                std::string out = "距离 " + std::to_string(snap.distance_mm) + " 毫米（" + st + "，";
                out += std::to_string(age) + " 毫秒前）";
                static const char* kObstacleName[] = {"通畅", "靠近", "停车"};
                static const char* kEdgeName[] = {"未知", "在桌上", "桌沿"};
                if (snap.obstacle >= 0 && snap.obstacle <= 2) {
                    out += "，前方";
                    out += kObstacleName[snap.obstacle];
                }
                if (snap.edge >= 0 && snap.edge <= 2) {
                    out += "，位置";
                    out += kEdgeName[snap.edge];
                }
                if (snap.blocked) {
                    out += "，安全闸门已触发";
                }
                return out;
            });

        mcp_server.AddTool("self.sensor.calibrate",
            "校准激光测距的桌沿基线。调用时会先停电机，然后连续采样约 1 秒取中位数，"
            "写入记忆。校准之后机器人才能判断自己离桌沿有多远。"
            "请在机器人静止、前方无障碍物、且正对空旷处时调用。",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                int baseline = 0;
                if (!Application::GetInstance().CalibrateSensor(&baseline)) {
                    return std::string("校准失败：传感器不可用，或读数波动太大（可能有人或物体在传感器前晃动）。");
                }
                return std::string("校准完成，基线 ") + std::to_string(baseline) + " 毫米。";
            });

        ESP_LOGI(TAG, "测距工具注册完成：2 个（sensor.get_distance / sensor.calibrate）");

        // ---- 2 个机器人模式工具 ----

        // ---- 1 个红外避障工具 ----

        mcp_server.AddTool("self.avoid.status",
            "读取红外对射避障的当前状态。这是开关量：只回答「后方挡了 / 没挡」，不提供距离。\\n"
            "返回原始电平、开机以来触发次数，以及此刻后退指令会不会被拦截。\\n"
            "传感器装在后头，管的是倒车这一侧；要前方距离用 self.sensor.get_distance。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!infrared_avoid_) {
                    return std::string("红外避障没有装，或者启动失败。");
                }
                std::string out = infrared_avoid_->IsBlocked() ? "后方有遮挡。" : "后方通畅。";
                out += "原始电平 ";
                out += infrared_avoid_->RawLevel() ? "高" : "低";
                out += "，开机以来触发 ";
                out += std::to_string(infrared_avoid_->TriggerCount());
                out += " 次。";
                if (infrared_avoid_->IsBlocked()) {
                    out += "此刻后退会被拦截，前进和转向不受影响。";
                }
                return out;
            });

        ESP_LOGI(TAG, "红外避障工具注册完成：1 个（avoid.status）");

        mcp_server.AddTool("self.robot.start",
            "让机器人进入自动模式，开始自己动起来。mode 只能是这四个之一："
            "explore（到处乱走探索，不需要传感器）、"
            "free（原地做各种表情，不乱走，不需要传感器）、"
            "follow（跟着人走：超过10厘米就追，5到10厘米保持，5厘米以内后退，需要距离传感器）、"
            "edge_patrol（沿桌沿巡逻：直行到边缘就退回来再转个方向，需要距离传感器）。"
            "如果已经在跑别的模式会返回错误，要先调用 self.robot.stop。"
            "follow 和 edge_patrol 在读不到传感器数据时会拒绝启动。"
            "用户说「开始探索」「自由活动」「跟着我」「别掉下去」时对应调用。",
            PropertyList({
                Property("mode", kPropertyTypeString)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                std::string mode = properties["mode"].value<std::string>();
                int m = kRobotNone;
                if (mode == "explore") {
                    m = kRobotExplore;
                } else if (mode == "free") {
                    m = kRobotFree;
                } else if (mode == "follow") {
                    m = kRobotFollow;
                } else if (mode == "edge_patrol") {
                    m = kRobotEdgePatrol;
                } else {
                    return std::string("未知模式：") + mode +
                           "。只支持 explore / free / follow / edge_patrol";
                }
                std::string err = Application::GetInstance().StartRobotMode(m);
                if (!err.empty()) {
                    return std::string("启动失败：") + err;
                }
                return std::string("已进入 ") + mode + " 模式";
            });

        mcp_server.AddTool("self.robot.stop",
            "让机器人停下来，退出当前自动模式。用户说「退出」「停下来」「别跑了」"
            "「回来」「结束」时调用。本来就没在跑模式时会返回提示，不算出错。",
            PropertyList(),
            [](const PropertyList&) -> ReturnValue {
                auto& app = Application::GetInstance();
                int m = app.GetRobotMode();
                if (m == kRobotNone) {
                    return std::string("现在没在跑任何模式");
                }
                app.StopRobotMode();
                return std::string("已退出 ") + Application::RobotModeName(m) + " 模式";
            });

        ESP_LOGI(TAG, "机器人工具注册完成：2 个（robot.start / robot.stop）");
    }

public:
    XiaozhiAiBoard() :
        boot_button_(BOOT_BUTTON_GPIO, false, 1000),   // 第 6 步：长按判定 1 秒
        touch_button_(TOUCH_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializeDisplayI2c();
        InitializeOledDisplay();
        InitializeSensorI2c();
        InitializeSensor();
        InitializeInfraredAvoid();
        InitializeButtons();
        InitializeRgbStrip();
        InitializeTools();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(XiaozhiAiBoard);

// ===== 灯光 Web 接口 =====
// web_server.cc 是公共文件，不 include 板卡头文件；所以这里只导出 C 接口，
// JSON 的拼装和解析都放在板卡侧（只有它知道 StripParams 长什么样）。
// 约定：调用方传一块缓冲进来，返回值表示填充成功，避免跨模块传堆指针。

namespace {

bool JsonToBuf(cJSON* root, char* out, size_t out_len) {
    if (root == nullptr) return false;
    char* s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (s == nullptr) return false;
    size_t n = strlen(s);
    bool ok = (n + 1) <= out_len;
    if (ok) memcpy(out, s, n + 1);
    cJSON_free(s);
    return ok;
}

void SetErr(char* err, size_t errlen, const char* msg) {
    if (err != nullptr && errlen > 0) {
        snprintf(err, errlen, "%s", msg);
    }
}

// 从 JSON 对象里读参数。字段都可以缺省，缺省就保持 out 里的原值，
// 这样 Web 端只发改动过的那几个也能生效。
bool ParseParamsObject(const cJSON* obj, StripParams* out, char* err, size_t errlen) {
    if (obj == nullptr) return true;
    if (!cJSON_IsObject(obj)) {
        SetErr(err, errlen, "params must be an object");
        return false;
    }

    const cJSON* effect = cJSON_GetObjectItemCaseSensitive(obj, "effect");
    if (effect != nullptr) {
        uint8_t id = kFxCount;
        if (cJSON_IsNumber(effect)) {
            int v = effect->valueint;
            if (v >= 0 && v < (int)kFxCount) id = (uint8_t)v;
        } else if (cJSON_IsString(effect)) {
            id = FindEffectByName(effect->valuestring);
            if (id == kFxCount && effect->valuestring[0] != 0 && effect->valuestring[1] == 0) {
                id = FindEffectByLetter(effect->valuestring[0]);
            }
        }
        if (id >= kFxCount) {
            SetErr(err, errlen, "unknown effect");
            return false;
        }
        out->effect = id;
    }

    const struct {
        const char* key;
        uint8_t* dst;
        int lo;
        int hi;
    } num_fields[] = {
        {"speed", &out->speed, 0, 255},
        {"intensity", &out->intensity, 0, 255},
        {"c1", &out->c1, 0, 255},
        {"c2", &out->c2, 0, 255},
    };
    for (const auto& f : num_fields) {
        const cJSON* v = cJSON_GetObjectItemCaseSensitive(obj, f.key);
        if (v == nullptr) continue;
        if (!cJSON_IsNumber(v)) {
            SetErr(err, errlen, "numeric field expected");
            return false;
        }
        int n = v->valueint;
        if (n < f.lo) n = f.lo;
        if (n > f.hi) n = f.hi;
        *f.dst = (uint8_t)n;
    }

    const cJSON* colors = cJSON_GetObjectItemCaseSensitive(obj, "colors");
    if (colors != nullptr) {
        if (!cJSON_IsArray(colors)) {
            SetErr(err, errlen, "colors must be an array");
            return false;
        }
        int n = cJSON_GetArraySize(colors);
        if (n > 6) n = 6;
        out->color_count = 0;
        for (int i = 0; i < n; i++) {
            const cJSON* t = cJSON_GetArrayItem(colors, i);
            if (!cJSON_IsArray(t) || cJSON_GetArraySize(t) < 3) {
                SetErr(err, errlen, "each color must be [r,g,b]");
                return false;
            }
            const cJSON* r = cJSON_GetArrayItem(t, 0);
            const cJSON* g = cJSON_GetArrayItem(t, 1);
            const cJSON* b = cJSON_GetArrayItem(t, 2);
            if (!cJSON_IsNumber(r) || !cJSON_IsNumber(g) || !cJSON_IsNumber(b)) {
                SetErr(err, errlen, "color components must be numbers");
                return false;
            }
            StripColor c;
            c.r = (uint8_t)(r->valueint < 0 ? 0 : (r->valueint > 255 ? 255 : r->valueint));
            c.g = (uint8_t)(g->valueint < 0 ? 0 : (g->valueint > 255 ? 255 : g->valueint));
            c.b = (uint8_t)(b->valueint < 0 ? 0 : (b->valueint > 255 ? 255 : b->valueint));
            out->colors[out->color_count++] = c;
        }
    }
    return true;
}

}  // namespace

extern "C" {

bool HandleLedEffectsJson(char* out, size_t out_len) {
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr) return false;
    cJSON* arr = cJSON_AddArrayToObject(root, "effects");
    for (int i = 0; i < (int)kFxCount; i++) {
        const EffectDef& d = kEffects[i];
        cJSON* item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "id", i);
        cJSON_AddStringToObject(item, "name", d.name);
        char letter[2] = {d.letter, 0};
        cJSON_AddStringToObject(item, "letter", d.letter ? letter : "");
        cJSON_AddNumberToObject(item, "mask", d.param_mask);
        cJSON_AddStringToObject(item, "p1", d.p1 ? d.p1 : "");
        cJSON_AddStringToObject(item, "p2", d.p2 ? d.p2 : "");
        cJSON_AddStringToObject(item, "p3", d.p3 ? d.p3 : "");
        cJSON_AddStringToObject(item, "p4", d.p4 ? d.p4 : "");
        cJSON_AddNumberToObject(item, "cmin", d.color_min);
        cJSON_AddNumberToObject(item, "cmax", d.color_max);
        cJSON_AddItemToArray(arr, item);
    }
    cJSON_AddNumberToObject(root, "brightness", RgbStrip::GetBrightness());
    return JsonToBuf(root, out, out_len);
}

bool HandleLedStatusJson(char* out, size_t out_len) {
    if (g_rgb_strip_for_web == nullptr) return false;
    StripParams p = g_rgb_strip_for_web->GetParams();

    cJSON* root = cJSON_CreateObject();
    if (root == nullptr) return false;
    cJSON_AddNumberToObject(root, "brightness", RgbStrip::GetBrightness());
    cJSON_AddNumberToObject(root, "led_count", (int)g_rgb_strip_for_web->LedCount());
    cJSON_AddNumberToObject(root, "led_max", (int)RgbStrip::kMaxLeds);
    cJSON_AddNumberToObject(root, "strip_length", (int)g_rgb_strip_for_web->StripLength());
    cJSON_AddBoolToObject(root, "manual_override", g_rgb_strip_for_web->ManualOverride());
    cJSON_AddStringToObject(root, "gcode", g_rgb_strip_for_web->GetCurrentGcode().c_str());

    cJSON* params = cJSON_AddObjectToObject(root, "params");
    cJSON_AddNumberToObject(params, "effect", p.effect);
    cJSON_AddNumberToObject(params, "speed", p.speed);
    cJSON_AddNumberToObject(params, "intensity", p.intensity);
    cJSON_AddNumberToObject(params, "c1", p.c1);
    cJSON_AddNumberToObject(params, "c2", p.c2);
    cJSON* colors = cJSON_AddArrayToObject(params, "colors");
    for (int i = 0; i < p.color_count && i < 6; i++) {
        cJSON* c = cJSON_CreateArray();
        cJSON_AddItemToArray(c, cJSON_CreateNumber(p.colors[i].r));
        cJSON_AddItemToArray(c, cJSON_CreateNumber(p.colors[i].g));
        cJSON_AddItemToArray(c, cJSON_CreateNumber(p.colors[i].b));
        cJSON_AddItemToArray(colors, c);
    }

    cJSON* slots = cJSON_AddArrayToObject(root, "slots");
    for (int i = 0; i < (int)RgbStrip::kSlotCount; i++) {
        cJSON* item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "id", i);
        cJSON_AddStringToObject(item, "key", RgbStrip::SlotName((RgbStrip::Slot)i));
        cJSON_AddStringToObject(item, "gcode",
                                g_rgb_strip_for_web->GetSlotGcode((RgbStrip::Slot)i).c_str());
        cJSON_AddItemToArray(slots, item);
    }
    return JsonToBuf(root, out, out_len);
}

// 立即生效，不写 NVS。body 形如 {"effect":5,"speed":128,...}
bool HandleLedApplyJson(const char* json, char* err, size_t errlen) {
    if (g_rgb_strip_for_web == nullptr) {
        SetErr(err, errlen, "led strip not ready");
        return false;
    }
    if (json == nullptr || json[0] == 0) {
        SetErr(err, errlen, "empty body");
        return false;
    }
    cJSON* root = cJSON_Parse(json);
    if (root == nullptr) {
        SetErr(err, errlen, "invalid json");
        return false;
    }
    StripParams p = g_rgb_strip_for_web->GetParams();
    bool ok = ParseParamsObject(root, &p, err, errlen);
    // 允许顺带改亮度
    const cJSON* br = cJSON_GetObjectItemCaseSensitive(root, "brightness");
    int brightness = -1;
    if (ok && cJSON_IsNumber(br)) brightness = br->valueint;
    cJSON_Delete(root);
    if (!ok) return false;

    g_rgb_strip_for_web->ApplyParams(p);
    if (brightness >= 0) g_rgb_strip_for_web->SetBrightness(brightness);
    return true;
}

// 直接吃 gcode，给"从旧值恢复/手工输入"用。
bool HandleLedApplyGcode(const char* gcode, char* err, size_t errlen) {
    if (g_rgb_strip_for_web == nullptr) {
        SetErr(err, errlen, "led strip not ready");
        return false;
    }
    std::string error;
    if (!g_rgb_strip_for_web->ApplyGcode(gcode ? gcode : "", &error)) {
        SetErr(err, errlen, error.empty() ? "invalid gcode" : error.c_str());
        return false;
    }
    return true;
}

// body 形如 {"slot":"listening","params":{...}} 或 {"slot":3,"gcode":"7,90,W"}
bool HandleLedSaveSlotJson(const char* json, char* err, size_t errlen) {
    if (g_rgb_strip_for_web == nullptr) {
        SetErr(err, errlen, "led strip not ready");
        return false;
    }
    cJSON* root = cJSON_Parse(json ? json : "");
    if (root == nullptr) {
        SetErr(err, errlen, "invalid json");
        return false;
    }

    RgbStrip::Slot slot = RgbStrip::kSlotCount;
    const cJSON* s = cJSON_GetObjectItemCaseSensitive(root, "slot");
    if (cJSON_IsNumber(s)) {
        int v = s->valueint;
        if (v >= 0 && v < (int)RgbStrip::kSlotCount) slot = (RgbStrip::Slot)v;
    } else if (cJSON_IsString(s)) {
        for (int i = 0; i < (int)RgbStrip::kSlotCount; i++) {
            if (strcmp(s->valuestring, RgbStrip::SlotName((RgbStrip::Slot)i)) == 0) {
                slot = (RgbStrip::Slot)i;
                break;
            }
        }
    }
    if (slot >= RgbStrip::kSlotCount) {
        cJSON_Delete(root);
        SetErr(err, errlen, "unknown slot");
        return false;
    }

    const cJSON* gcode = cJSON_GetObjectItemCaseSensitive(root, "gcode");
    if (cJSON_IsString(gcode)) {
        std::string error;
        if (!RgbStrip::ValidateGcode(gcode->valuestring, &error)) {
            cJSON_Delete(root);
            SetErr(err, errlen, error.empty() ? "invalid gcode" : error.c_str());
            return false;
        }
        g_rgb_strip_for_web->SetSlotGcode(slot, gcode->valuestring);
        cJSON_Delete(root);
        return true;
    }

    // 变体：{"slot":"listening","params":{...}}，从当前参数改
    StripParams p = g_rgb_strip_for_web->GetParams();
    bool ok = ParseParamsObject(cJSON_GetObjectItemCaseSensitive(root, "params"), &p, err, errlen);
    cJSON_Delete(root);
    if (!ok) return false;

    g_rgb_strip_for_web->SetSlotParams(slot, p);
    return true;
}

int HandleLedSetBrightness(int percent) {
    if (g_rgb_strip_for_web == nullptr) return -1;
    g_rgb_strip_for_web->SetBrightness(percent);
    RgbStrip::PersistBrightness();
    return RgbStrip::GetBrightness();
}

int HandleLedGetBrightness() {
    return RgbStrip::GetBrightness();
}

// 运行时改灯珠数量。返回改完之后的数量，-1 表示失败。
// 上限 RgbStrip::kMaxLeds，超出的在 SetLedCount 里夹住。
int HandleLedSetCount(int count) {
    if (g_rgb_strip_for_web == nullptr) return -1;
    return (int)g_rgb_strip_for_web->SetLedCount((uint16_t)count);
}

int HandleLedGetCount() {
    if (g_rgb_strip_for_web == nullptr) return -1;
    return (int)g_rgb_strip_for_web->LedCount();
}

int HandleLedMaxCount() {
    return (int)RgbStrip::kMaxLeds;
}

// 恢复"跟随设备状态"：清掉手动覆盖，按当前状态重新挑槽位。
int HandleLedFollowState(int device_state) {
    if (g_rgb_strip_for_web == nullptr) return -1;
    g_rgb_strip_for_web->ClearManualOverride();
    switch (static_cast<DeviceState>(device_state)) {
        case kDeviceStateSpeaking:
            g_rgb_strip_for_web->ApplySlot(RgbStrip::kSlotSpeaking);
            break;
        case kDeviceStateListening:
            g_rgb_strip_for_web->ApplySlot(RgbStrip::kSlotListening);
            break;
        default:
            g_rgb_strip_for_web->ApplySlot(RgbStrip::kSlotStartupIdle);
            break;
    }
    return 0;
}

}  // extern "C"

