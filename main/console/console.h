#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 启动串口服务终端。
 *
 * 挂在 UART0（就是平时看日志的那个 COM3，115200），用 ESP-IDF 的 esp_console
 * 做 REPL，因此自带行编辑、历史、方向键和 tab 补全。日志输出不受影响，
 * 两边共用一个串口，只是方向不同。
 *
 * 命令：
 *   help                      列出命令
 *   status                    运行时长、显示模式、机器人模式、传感器、电机、堆
 *   heap                      堆明细（内部 / PSRAM / 最大块）
 *   tasks                     任务列表与栈水位
 *   log <tag|*> <level>       运行时改日志等级，level 取 none/error/warn/info/debug/verbose
 *   motor <dir> [speed]       手动驱动，dir 1=右 2=后 3=左 4=前 0=停，speed 0-100
 *   robot <mode|stop>         切机器人模式：explore/free/follow/edge_patrol/stop
 *   sensor                    读一次距离传感器的完整快照
 *   reboot                    重启
 *
 * 只在开发阶段有用。量产要拿掉的话，删掉启动调用即可，命令实现会被 --gc-sections 带走。
 */
void console_start(void);

#ifdef __cplusplus
}
#endif
