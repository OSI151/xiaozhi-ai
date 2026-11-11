#include "console.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "esp_chip_info.h"
#include "esp_console.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "application.h"

static const char* TAG = "Console";

namespace {

// ---------------------------------------------------------------- 工具

const char* ResetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_UNKNOWN:   return "unknown";
        case ESP_RST_POWERON:   return "poweron";
        case ESP_RST_EXT:       return "ext";
        case ESP_RST_SW:        return "sw";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:   return "int_wdt";
        case ESP_RST_TASK_WDT:  return "task_wdt";
        case ESP_RST_WDT:       return "wdt";
        case ESP_RST_DEEPSLEEP: return "deepsleep";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_SDIO:      return "sdio";
        default:                return "other";
    }
}

// 运行时长，拆成 天/时/分/秒。开机不到一分钟的时候也好看。
void PrintUptime() {
    // 用 int 而不是 long long：这颗芯片的 sdkconfig 开着 CONFIG_NEWLIB_NANO_FORMAT，
    // printf 不认 %lld，会原样打出 "ldd" 这种字符。十亿秒有 31 年，够用了。
    int sec = (int)(esp_timer_get_time() / 1000000);
    printf("%dd %02d:%02d:%02d\n",
           sec / 86400,
           (sec % 86400) / 3600,
           (sec % 3600) / 60,
           sec % 60);
}

bool ParseLogLevel(const char* s, esp_log_level_t* out) {
    if (strcmp(s, "none") == 0 || strcmp(s, "off") == 0)    { *out = ESP_LOG_NONE;    return true; }
    if (strcmp(s, "e") == 0 || strcmp(s, "error") == 0)     { *out = ESP_LOG_ERROR;   return true; }
    if (strcmp(s, "w") == 0 || strcmp(s, "warn") == 0)      { *out = ESP_LOG_WARN;    return true; }
    if (strcmp(s, "i") == 0 || strcmp(s, "info") == 0)      { *out = ESP_LOG_INFO;    return true; }
    if (strcmp(s, "d") == 0 || strcmp(s, "debug") == 0)     { *out = ESP_LOG_DEBUG;   return true; }
    if (strcmp(s, "v") == 0 || strcmp(s, "verbose") == 0)   { *out = ESP_LOG_VERBOSE; return true; }
    return false;
}

// ---------------------------------------------------------------- 命令

int CmdHelp(int argc, char** argv);

int CmdStatus(int argc, char** argv) {
    auto& app = Application::GetInstance();

    esp_chip_info_t chip;
    esp_chip_info(&chip);
    printf("chip        : %s rev%d.%d, %d core(s)\n",
           CONFIG_IDF_TARGET,
           chip.revision / 100, chip.revision % 100,
           chip.cores);
    printf("idf         : %s\n", esp_get_idf_version());
    printf("reset reason: %s\n", ResetReasonName(esp_reset_reason()));
    printf("uptime      : ");
    PrintUptime();

    const char* dm = "default";
    switch (app.GetDisplayMode()) {
        case kDisplayModeDefault: dm = "default(官方界面)"; break;
        case kDisplayModeEyeOnly: dm = "eyes(眼睛)";    break;
        case kDisplayModeSleep:   dm = "sleep(大时钟)"; break;
        case kDisplayModeMusic:   dm = "music(音乐)";   break;
    }
    printf("display     : %s\n", dm);
    printf("robot mode  : %s\n", Application::RobotModeName(app.GetRobotMode()));
    printf("device state: %d\n", (int)app.GetDeviceState());

    SensorSnapshot snap = app.GetSensorSnapshot();
    static const char* kObstacleName[] = {"通畅", "靠近", "停车"};
    static const char* kEdgeName[]     = {"未知", "在桌上", "桌沿"};
    if (!snap.sensor_ok) {
        printf("sensor      : 不可用 (fail_count=%d)\n", snap.fail_count);
    } else if (snap.age_ms() > 1000) {
        printf("sensor      : 没有新数据（%.1f 秒前）\n", snap.age_ms() / 1000.0f);
    } else {
        printf("sensor      : %d mm, %s, %s, age %d ms, baseline %d mm\n",
               snap.distance_mm,
               (snap.obstacle >= 0 && snap.obstacle <= 2) ? kObstacleName[snap.obstacle] : "?",
               (snap.edge >= 0 && snap.edge <= 2) ? kEdgeName[snap.edge] : "?",
               snap.age_ms(), snap.baseline_mm);
    }

    printf("heap internal: free %u, min %u, largest %u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    return 0;
}

int CmdHeap(int argc, char** argv) {
    printf("              free      min   largest\n");
    printf("internal  %8u %8u %9u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    printf("spiram    %8u %8u %9u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    printf("8bit      %8u\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
    printf("dma       %8u\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA));
    return 0;
}

int CmdTasks(int argc, char** argv) {
    // 用 uxTaskGetSystemState 拿一次快照。任务数先探一次再分配，
    // 免得固定写死 32 个在任务多的时候截断。
    UBaseType_t count = uxTaskGetNumberOfTasks();
    TaskStatus_t* arr = (TaskStatus_t*)malloc(sizeof(TaskStatus_t) * (count + 4));
    if (arr == nullptr) {
        printf("out of memory (%u tasks)\n", (unsigned)count);
        return 1;
    }
    uint32_t total_runtime = 0;
    UBaseType_t got = uxTaskGetSystemState(arr, count + 4, &total_runtime);

    // stack_free 是 usStackHighWaterMark，含义是"历史最低剩余量"，
    // 数字越小越接近溢出，不是已用字节数。
    printf("%-16s %-4s %-5s %-10s %s\n", "name", "pri", "core", "stack_free", "state");
    for (UBaseType_t i = 0; i < got; ++i) {
        // tskNO_AFFINITY 是 0x7FFFFFFF，直接打出来是 2147483647，看不出意思。
        char core_buf[16];   // %d 最坏能到 "-2147483648"，8 字节不够，编译器会报截断
        if (arr[i].xCoreID == tskNO_AFFINITY) {
            snprintf(core_buf, sizeof(core_buf), "any");
        } else {
            snprintf(core_buf, sizeof(core_buf), "%d", (int)arr[i].xCoreID);
        }

        const char* st = "?";
        switch (arr[i].eCurrentState) {
            case eRunning:   st = "run";   break;
            case eReady:     st = "ready"; break;
            case eBlocked:   st = "block"; break;
            case eSuspended: st = "susp";  break;
            case eDeleted:   st = "del";   break;
            default:         st = "?";     break;
        }
        printf("%-16s %-4u %-5s %-10u %s\n",
               arr[i].pcTaskName,
               (unsigned)arr[i].uxCurrentPriority,
               core_buf,
               (unsigned)arr[i].usStackHighWaterMark,
               st);
    }
    printf("\n%u tasks\n", (unsigned)got);
    free(arr);
    return 0;
}

int CmdLog(int argc, char** argv) {
    if (argc < 3) {
        printf("usage: log <tag|*> <none|error|warn|info|debug|verbose>\n");
        printf("       log * info        恢复全局 info\n");
        printf("       log Application d 只把 Application 调到 debug\n");
        return 1;
    }
    esp_log_level_t level;
    if (!ParseLogLevel(argv[2], &level)) {
        printf("bad level '%s'\n", argv[2]);
        return 1;
    }
    if (strcmp(argv[1], "*") == 0) {
        esp_log_level_set("*", level);
        printf("all tags -> %s\n", argv[2]);
        return 0;
    }
    esp_log_level_set(argv[1], level);
    printf("tag '%s' -> %s\n", argv[1], argv[2]);
    return 0;
}

int CmdMotor(int argc, char** argv) {
    if (argc < 2) {
        printf("usage: motor <1|2|3|4|0> [speed 0-100]\n");
        printf("       1=右 2=后 3=左 4=前 0=停\n");
        return 1;
    }
    int dir = atoi(argv[1]);
    if (dir == 0) {
        Application::GetInstance().DebugMotorStop();
        printf("stopped\n");
        return 0;
    }
    if (dir < 1 || dir > 4) {
        printf("bad direction %d\n", dir);
        return 1;
    }
    int speed = (argc >= 3) ? atoi(argv[2]) : 55;
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    // 走 from_web=true 这条路，绕开传感器闸门，方便单独验电机。
    Application::GetInstance().DebugMotorCommand(dir, speed);
    printf("motor dir=%d speed=%d\n", dir, speed);
    return 0;
}

int CmdRobot(int argc, char** argv) {
    auto& app = Application::GetInstance();
    if (argc < 2) {
        printf("usage: robot <explore|free|follow|edge_patrol|stop>\n");
        printf("current: %s\n", Application::RobotModeName(app.GetRobotMode()));
        return 1;
    }
    if (strcmp(argv[1], "stop") == 0) {
        app.StopRobotMode();
        printf("stopped\n");
        return 0;
    }
    int mode = kRobotNone;
    if (strcmp(argv[1], "explore") == 0)          mode = kRobotExplore;
    else if (strcmp(argv[1], "free") == 0)        mode = kRobotFree;
    else if (strcmp(argv[1], "follow") == 0)      mode = kRobotFollow;
    else if (strcmp(argv[1], "edge_patrol") == 0) mode = kRobotEdgePatrol;
    else {
        printf("unknown mode '%s'\n", argv[1]);
        return 1;
    }
    std::string err = app.StartRobotMode(mode);
    if (err.empty()) {
        printf("robot mode: %s\n", Application::RobotModeName(mode));
        return 0;
    }
    printf("failed: %s\n", err.c_str());
    return 1;
}

int CmdSensor(int argc, char** argv) {
    SensorSnapshot snap = Application::GetInstance().GetSensorSnapshot();
    static const char* kObstacleName[] = {"通畅", "靠近", "停车"};
    static const char* kEdgeName[]     = {"未知", "在桌上", "桌沿"};
    printf("sensor_ok   : %s\n", snap.sensor_ok ? "true" : "false");
    printf("distance_mm : %d\n", snap.distance_mm);
    printf("status      : %d\n", snap.status);
    printf("age_ms      : %d\n", snap.age_ms());
    printf("baseline_mm : %d\n", snap.baseline_mm);
    printf("obstacle    : %d (%s)\n", snap.obstacle,
           (snap.obstacle >= 0 && snap.obstacle <= 2) ? kObstacleName[snap.obstacle] : "?");
    printf("edge        : %d (%s), confirm %d\n", snap.edge,
           (snap.edge >= 0 && snap.edge <= 2) ? kEdgeName[snap.edge] : "?",
           snap.edge_confirm);
    printf("blocked     : %s\n", snap.blocked ? "true" : "false");
    printf("fail_count  : %d\n", snap.fail_count);
    return 0;
}

int CmdReboot(int argc, char** argv) {
    printf("rebooting...\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return 0;
}

int CmdHelp(int argc, char** argv) {
    printf("commands:\n");
    printf("  help                     这个列表\n");
    printf("  status                   运行时长/显示模式/机器人模式/传感器/堆\n");
    printf("  heap                     堆明细\n");
    printf("  tasks                    任务列表与栈水位\n");
    printf("  log <tag|*> <level>      改日志等级 (none/error/warn/info/debug/verbose)\n");
    printf("  motor <dir> [speed]      1=右 2=后 3=左 4=前 0=停，speed 默认 55\n");
    printf("  robot <mode|stop>        explore / free / follow / edge_patrol / stop\n");
    printf("  sensor                   读一次传感器快照\n");
    printf("  reboot                   重启\n");
    return 0;
}

}  // namespace

// ---------------------------------------------------------------- 注册与启动

extern "C" void console_start(void) {
    esp_console_repl_t* repl = nullptr;

    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "xiaozhi> ";
    repl_config.max_cmdline_length = 128;
    repl_config.task_stack_size = 6144;
    repl_config.task_priority = tskIDLE_PRIORITY + 1;

    const esp_console_cmd_t cmds[] = {
        {.command = "help",   .help = "列出命令",              .hint = nullptr, .func = CmdHelp},
        {.command = "status", .help = "状态总览",              .hint = nullptr, .func = CmdStatus},
        {.command = "heap",   .help = "堆明细",                .hint = nullptr, .func = CmdHeap},
        {.command = "tasks",  .help = "任务列表与栈水位",      .hint = nullptr, .func = CmdTasks},
        {.command = "log",    .help = "改日志等级",            .hint = "<tag|*> <level>", .func = CmdLog},
        {.command = "motor",  .help = "手动驱动电机",          .hint = "<dir> [speed]",  .func = CmdMotor},
        {.command = "robot",  .help = "切换机器人模式",        .hint = "<mode|stop>",    .func = CmdRobot},
        {.command = "sensor", .help = "读一次传感器快照",      .hint = nullptr, .func = CmdSensor},
        {.command = "reboot", .help = "重启",                  .hint = nullptr, .func = CmdReboot},
    };
    // 顺序不能换。esp_console_new_repl_uart 内部会先 esp_console_init 填好 s_config
    // （包括 heap_alloc_caps），再注册 help 命令。如果在它之前调 esp_console_cmd_register，
    // s_config.heap_alloc_caps 还是 0，heap_caps_calloc 会拿到一个不可用的指针，
    // 后面 free 它时直接 tlsf_free 断言 "block already marked as free"。
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    esp_err_t err = esp_console_new_repl_uart(&uart_config, &repl_config, &repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_console_new_repl_uart failed: %s", esp_err_to_name(err));
        return;
    }

    for (const auto& c : cmds) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&c));
    }

    err = esp_console_start_repl(repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_console_start_repl failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "console ready, 9 commands (help/status/heap/tasks/log/motor/robot/sensor/reboot)");
}
