// 见 i2c_panel_io_timeout.h 开头的说明。
//
// 本文件是 esp-idf-v5.5.4/components/esp_lcd/i2c/esp_lcd_panel_io_i2c_v2.c 的改写版，
// 唯一的功能差异是：所有 i2c_master_* 传输的超时参数从 -1（永久等待）
// 换成调用方指定的毫秒数。其余数据结构、调用顺序、回调时机都与 IDF 原版一致。
//
// 改动处用 "// [超时]" 标出，方便将来 IDF 升级时对照。

#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_io.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"

#include "i2c_panel_io_timeout.h"

static const char *TAG = "lcd_panel.io.i2c.to";

#define BYTESHIFT(VAR, IDX) (((VAR) >> ((IDX) * 8)) & 0xFF)
// IDF 原文件是 .c，数组初始化允许隐式截断；这里是 .cc，C++ 把 narrowing 当错误。
// 用这个宏把每个字节显式转成 uint8_t。
#define CMD_BYTE(VAR, IDX) static_cast<uint8_t>(BYTESHIFT(VAR, IDX))
#define CONTROL_PHASE_LENGTH  (1)
#define CMD_LENGTH            (4)

// 默认传输超时。400 kHz 下刷一整屏 1024 字节约 23 ms，
// 100 ms 留了四倍余量；总线真挂死时也能在可接受的时间内返回。
#define I2C_PANEL_IO_DEFAULT_TIMEOUT_MS  (100)

typedef struct {
    esp_lcd_panel_io_t base;
    i2c_master_dev_handle_t i2c_handle;
    uint32_t dev_addr;
    int lcd_cmd_bits;
    int lcd_param_bits;
    bool control_phase_enabled;
    uint32_t control_phase_cmd;
    uint32_t control_phase_data;
    uint32_t xfer_timeout_ms;           // [超时] 单次传输上限
    uint32_t timeout_count;             // [超时] 累计超时次数
    esp_lcd_panel_io_color_trans_done_cb_t on_color_trans_done;
    void *user_ctx;
} lcd_panel_io_i2c_to_t;

static esp_err_t panel_io_i2c_del(esp_lcd_panel_io_t *io);
static esp_err_t panel_io_i2c_rx_param(esp_lcd_panel_io_t *io, int lcd_cmd, void *param, size_t param_size);
static esp_err_t panel_io_i2c_tx_param(esp_lcd_panel_io_t *io, int lcd_cmd, const void *param, size_t param_size);
static esp_err_t panel_io_i2c_tx_color(esp_lcd_panel_io_t *io, int lcd_cmd, const void *color, size_t color_size);
static esp_err_t panel_io_i2c_register_event_callbacks(esp_lcd_panel_io_handle_t io,
                                                       const esp_lcd_panel_io_callbacks_t *cbs,
                                                       void *user_ctx);

esp_err_t esp_lcd_new_panel_io_i2c_timeout(i2c_master_bus_handle_t bus,
                                           const esp_lcd_panel_io_i2c_config_t *io_config,
                                           uint32_t xfer_timeout_ms,
                                           esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    lcd_panel_io_i2c_to_t *panel_io = NULL;
    i2c_master_dev_handle_t i2c_handle = NULL;
    // 声明放在 goto 之前并只做零初始化：C++ 不允许 goto 跳过带初始化的变量声明，
    // IDF 原文件是 .c 没这个限制。
    i2c_device_config_t i2c_lcd_cfg = {};

    ESP_GOTO_ON_FALSE(io_config && ret_io, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE(io_config->control_phase_bytes * 8 > io_config->dc_bit_offset,
                      ESP_ERR_INVALID_ARG, err, TAG, "D/C bit exceeds control bytes");

    panel_io = (lcd_panel_io_i2c_to_t *)calloc(1, sizeof(lcd_panel_io_i2c_to_t));
    ESP_GOTO_ON_FALSE(panel_io, ESP_ERR_NO_MEM, err, TAG, "no mem for i2c panel io");

    i2c_lcd_cfg.device_address = io_config->dev_addr;
    i2c_lcd_cfg.scl_speed_hz = io_config->scl_speed_hz;
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &i2c_lcd_cfg, &i2c_handle), err, TAG, "i2c add device fail");

    panel_io->i2c_handle = i2c_handle;
    panel_io->lcd_cmd_bits = io_config->lcd_cmd_bits;
    panel_io->lcd_param_bits = io_config->lcd_param_bits;
    panel_io->on_color_trans_done = io_config->on_color_trans_done;
    panel_io->user_ctx = io_config->user_ctx;
    panel_io->control_phase_enabled = (!io_config->flags.disable_control_phase);
    panel_io->control_phase_data = (!io_config->flags.dc_low_on_data) << (io_config->dc_bit_offset);
    panel_io->control_phase_cmd = (io_config->flags.dc_low_on_data) << (io_config->dc_bit_offset);
    panel_io->dev_addr = io_config->dev_addr;
    // [超时] 0 或异常值一律回落到默认值，避免调用方传 0 导致立即超时
    panel_io->xfer_timeout_ms = (xfer_timeout_ms == 0) ? I2C_PANEL_IO_DEFAULT_TIMEOUT_MS
                                                       : xfer_timeout_ms;
    panel_io->timeout_count = 0;

    panel_io->base.del = panel_io_i2c_del;
    panel_io->base.rx_param = panel_io_i2c_rx_param;
    panel_io->base.tx_param = panel_io_i2c_tx_param;
    panel_io->base.tx_color = panel_io_i2c_tx_color;
    panel_io->base.register_event_callbacks = panel_io_i2c_register_event_callbacks;

    *ret_io = &(panel_io->base);
    ESP_LOGI(TAG, "new i2c lcd panel io @%p, xfer timeout %" PRIu32 " ms",
             panel_io, panel_io->xfer_timeout_ms);
    return ESP_OK;

err:
    if (i2c_handle) {
        i2c_master_bus_rm_device(i2c_handle);
    }
    if (panel_io) {
        free(panel_io);
    }
    return ret;
}

uint32_t i2c_panel_io_timeout_count(esp_lcd_panel_io_handle_t io)
{
    if (io == NULL) {
        return 0;
    }
    lcd_panel_io_i2c_to_t *panel_io = __containerof(io, lcd_panel_io_i2c_to_t, base);
    return panel_io->timeout_count;
}

void i2c_panel_io_reset_timeout_count(esp_lcd_panel_io_handle_t io)
{
    if (io == NULL) {
        return;
    }
    lcd_panel_io_i2c_to_t *panel_io = __containerof(io, lcd_panel_io_i2c_to_t, base);
    panel_io->timeout_count = 0;
}

static esp_err_t panel_io_i2c_del(esp_lcd_panel_io_t *io)
{
    lcd_panel_io_i2c_to_t *panel_io = __containerof(io, lcd_panel_io_i2c_to_t, base);
    ESP_LOGD(TAG, "del lcd panel io i2c @%p (timeouts so far: %" PRIu32 ")", panel_io, panel_io->timeout_count);
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(panel_io->i2c_handle));
    free(panel_io);
    return ESP_OK;
}

static esp_err_t panel_io_i2c_register_event_callbacks(esp_lcd_panel_io_handle_t io,
                                                       const esp_lcd_panel_io_callbacks_t *cbs,
                                                       void *user_ctx)
{
    lcd_panel_io_i2c_to_t *panel_io = __containerof(io, lcd_panel_io_i2c_to_t, base);
    if (panel_io->on_color_trans_done != NULL) {
        ESP_LOGW(TAG, "Callback on_color_trans_done was already set and now it was overwritten!");
    }
    panel_io->on_color_trans_done = cbs->on_color_trans_done;
    panel_io->user_ctx = user_ctx;
    return ESP_OK;
}

// 统一的超时错误处理：累计计数 + 打日志，返回原始错误码由调用方决定
static esp_err_t handle_xfer_error(lcd_panel_io_i2c_to_t *panel_io, esp_err_t err, const char *what)
{
    if (err == ESP_ERR_TIMEOUT) {
        panel_io->timeout_count++;
        ESP_LOGW(TAG, "i2c %s timeout (%" PRIu32 " ms), total %" PRIu32 " timeouts",
                 what, panel_io->xfer_timeout_ms, panel_io->timeout_count);
    } else {
        ESP_LOGE(TAG, "i2c %s failed: %s", what, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t panel_io_i2c_rx_buffer(esp_lcd_panel_io_t *io, int lcd_cmd, void *buffer, size_t buffer_size)
{
    esp_err_t ret = ESP_OK;
    lcd_panel_io_i2c_to_t *panel_io = __containerof(io, lcd_panel_io_i2c_to_t, base);
    bool send_param = (lcd_cmd >= 0);

    if (send_param) {
        int write_size = 0;
        uint8_t write_buffer[CONTROL_PHASE_LENGTH + CMD_LENGTH] = {0};

        if (panel_io->control_phase_enabled) {
            write_buffer[0] = panel_io->control_phase_cmd;
            write_size += 1;
        }
        uint8_t cmds[4] = {CMD_BYTE(lcd_cmd, 3), CMD_BYTE(lcd_cmd, 2), CMD_BYTE(lcd_cmd, 1), CMD_BYTE(lcd_cmd, 0)};
        size_t cmds_size = panel_io->lcd_cmd_bits / 8;
        if (cmds_size > 0 && cmds_size <= sizeof(cmds)) {
            memcpy(write_buffer + write_size, cmds + (sizeof(cmds) - cmds_size), cmds_size);
            write_size += cmds_size;
        }

        // [超时] 原版是 -1
        ret = i2c_master_transmit_receive(panel_io->i2c_handle, write_buffer, write_size,
                                          static_cast<uint8_t *>(buffer), buffer_size,
                                          panel_io->xfer_timeout_ms);
        if (ret != ESP_OK) {
            return handle_xfer_error(panel_io, ret, "rx(with cmd)");
        }
    } else {
        // [超时] 原版是 -1
        ret = i2c_master_receive(panel_io->i2c_handle, static_cast<uint8_t *>(buffer), buffer_size,
                                 panel_io->xfer_timeout_ms);
        if (ret != ESP_OK) {
            return handle_xfer_error(panel_io, ret, "rx");
        }
    }
    return ESP_OK;
}

static esp_err_t panel_io_i2c_tx_buffer(esp_lcd_panel_io_t *io, int lcd_cmd,
                                        const void *buffer, size_t buffer_size, bool is_param)
{
    lcd_panel_io_i2c_to_t *panel_io = __containerof(io, lcd_panel_io_i2c_to_t, base);
    bool send_param = (lcd_cmd >= 0);

    uint8_t control_phase_byte = 0;
    size_t control_phase_size = 0;
    if (panel_io->control_phase_enabled) {
        control_phase_byte = is_param ? panel_io->control_phase_cmd : panel_io->control_phase_data;
        control_phase_size = 1;
    }

    uint8_t *cmd_buffer = NULL;
    size_t cmd_buffer_size = 0;
    uint8_t cmds[4] = {CMD_BYTE(lcd_cmd, 3), CMD_BYTE(lcd_cmd, 2), CMD_BYTE(lcd_cmd, 1), CMD_BYTE(lcd_cmd, 0)};
    if (send_param) {
        size_t cmds_size = panel_io->lcd_cmd_bits / 8;
        if (cmds_size > 0 && cmds_size <= sizeof(cmds)) {
            cmd_buffer = cmds + (sizeof(cmds) - cmds_size);
            cmd_buffer_size = cmds_size;
        }
    }

    uint8_t *lcd_buffer = NULL;
    size_t lcd_buffer_size = 0;
    if (buffer) {
        lcd_buffer = (uint8_t *)buffer;
        lcd_buffer_size = buffer_size;
    }

    i2c_master_transmit_multi_buffer_info_t lcd_i2c_buffer[3] = {
        {.write_buffer = &control_phase_byte, .buffer_size = control_phase_size},
        {.write_buffer = cmd_buffer, .buffer_size = cmd_buffer_size},
        {.write_buffer = lcd_buffer, .buffer_size = lcd_buffer_size},
    };

    // [超时] 原版是 -1。刷屏走的就是这一条。
    esp_err_t ret = i2c_master_multi_buffer_transmit(panel_io->i2c_handle, lcd_i2c_buffer,
                                                     sizeof(lcd_i2c_buffer) / sizeof(i2c_master_transmit_multi_buffer_info_t),
                                                     panel_io->xfer_timeout_ms);
    if (ret != ESP_OK) {
        return handle_xfer_error(panel_io, ret, is_param ? "tx(cmd)" : "tx(color)");
    }

    // 只在数据（非命令参数）传完之后回调，与 IDF 原版一致。
    // 这一句的位置不能提前：提前了 LVGL 会以为 buffer 已空，下一帧覆盖正在发送的数据，
    // 表现为高负载下的随机横条和撕裂，而且不复现。
    if (!is_param) {
        if (panel_io->on_color_trans_done) {
            panel_io->on_color_trans_done(&(panel_io->base), NULL, panel_io->user_ctx);
        }
    }
    return ESP_OK;
}

static esp_err_t panel_io_i2c_rx_param(esp_lcd_panel_io_t *io, int lcd_cmd, void *param, size_t param_size)
{
    return panel_io_i2c_rx_buffer(io, lcd_cmd, param, param_size);
}

static esp_err_t panel_io_i2c_tx_param(esp_lcd_panel_io_t *io, int lcd_cmd, const void *param, size_t param_size)
{
    return panel_io_i2c_tx_buffer(io, lcd_cmd, param, param_size, true);
}

static esp_err_t panel_io_i2c_tx_color(esp_lcd_panel_io_t *io, int lcd_cmd, const void *color, size_t color_size)
{
    return panel_io_i2c_tx_buffer(io, lcd_cmd, color, color_size, false);
}
