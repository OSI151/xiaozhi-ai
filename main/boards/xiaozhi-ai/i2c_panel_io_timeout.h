// I2C panel IO（带传输超时）
//
// 为什么需要这个文件：
//   IDF 的 esp_lcd_panel_io_i2c_v2 把 i2c_master_* 的 xfer_timeout_ms 硬编码成 -1
//   （见 esp-idf-v5.5.4/components/esp_lcd/i2c/esp_lcd_panel_io_i2c_v2.c:145/147/193）。
//   -1 表示永久等待。I2C 总线被占死的时候，刷屏任务会卡在 i2c_master_multi_buffer_transmit
//   里面出不来，析构函数等 800ms 也等不到它退出，最后表现为：
//     W RoboEyesAdapter: panel task did not exit in time
//     assert failed: heap_caps_free (free() target pointer is outside heap areas)
//   实测发生在进入睡眠（切大时钟）时析构 adapter 的路径上，第 5 次触发崩溃。
//
// 做法：
//   照抄 IDF 的实现，只把 -1 换成调用方指定的毫秒数，并统计超时次数供上层判断。
//   交给 tny-robotics/sh1106 和 roboeyes_adapter 的仍然是标准 esp_lcd_panel_io_handle_t，
//   上层代码一行不用改。
//
// 对应 skill 条目：
//   esp32-firmware-engineer/references/communication-protocols.md:14
//     "Always use transaction timeouts; never wait forever on a busy bus."
//   :15 "Handle bus recovery/reset on repeated timeout conditions"
//   :16 "Serialize access with a mutex or a dedicated bus-owner task."

#ifndef _I2C_PANEL_IO_TIMEOUT_H_
#define _I2C_PANEL_IO_TIMEOUT_H_

#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_types.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_io_i2c.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// 建一个带超时的 I2C panel io。
//   xfer_timeout_ms：单次 I2C 传输的上限，0 表示用默认值（100ms）
// 返回的 handle 可以当普通 esp_lcd_panel_io_handle_t 用。
esp_err_t esp_lcd_new_panel_io_i2c_timeout(i2c_master_bus_handle_t bus,
                                           const esp_lcd_panel_io_i2c_config_t *io_config,
                                           uint32_t xfer_timeout_ms,
                                           esp_lcd_panel_io_handle_t *ret_io);

// 累计的传输超时次数。上层可以据此判断总线是否已经不健康。
uint32_t i2c_panel_io_timeout_count(esp_lcd_panel_io_handle_t io);

// 把超时计数清零。总线恢复后调用。
void i2c_panel_io_reset_timeout_count(esp_lcd_panel_io_handle_t io);

#ifdef __cplusplus
}
#endif

#endif  // _I2C_PANEL_IO_TIMEOUT_H_
