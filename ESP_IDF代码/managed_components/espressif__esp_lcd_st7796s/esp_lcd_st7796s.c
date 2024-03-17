/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "st7796s";

static esp_err_t panel_st7796s_del(esp_lcd_panel_t *panel);
static esp_err_t panel_st7796s_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_st7796s_init(esp_lcd_panel_t *panel);
static esp_err_t panel_st7796s_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_st7796s_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_st7796s_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_st7796s_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_st7796s_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_st7796s_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_cal; // save surrent value of LCD_CMD_COLMOD register
} st7796s_panel_t;

esp_err_t esp_lcd_new_panel_st7796s(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    st7796s_panel_t *st7796s = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    st7796s = calloc(1, sizeof(st7796s_panel_t));
    ESP_GOTO_ON_FALSE(st7796s, ESP_ERR_NO_MEM, err, TAG, "no mem for st7796s panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
        st7796s->madctl_val = 0;
        break;
    case ESP_LCD_COLOR_SPACE_BGR:
        st7796s->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        st7796s->colmod_cal = 0x55;
        fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        st7796s->colmod_cal = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    st7796s->io = io;
    st7796s->fb_bits_per_pixel = fb_bits_per_pixel;
    st7796s->reset_gpio_num = panel_dev_config->reset_gpio_num;
    st7796s->reset_level = panel_dev_config->flags.reset_active_high;
    st7796s->base.del = panel_st7796s_del;
    st7796s->base.reset = panel_st7796s_reset;
    st7796s->base.init = panel_st7796s_init;
    st7796s->base.draw_bitmap = panel_st7796s_draw_bitmap;
    st7796s->base.invert_color = panel_st7796s_invert_color;
    st7796s->base.set_gap = panel_st7796s_set_gap;
    st7796s->base.mirror = panel_st7796s_mirror;
    st7796s->base.swap_xy = panel_st7796s_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    st7796s->base.disp_off = panel_st7796s_disp_on_off;
#else
    st7796s->base.disp_on_off = panel_st7796s_disp_on_off;
#endif
    *ret_panel = &(st7796s->base);
    ESP_LOGD(TAG, "new st7796s panel @%p", st7796s);

    return ESP_OK;

err:
    if (st7796s) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(st7796s);
    }
    return ret;
}

static esp_err_t panel_st7796s_del(esp_lcd_panel_t *panel)
{
    st7796s_panel_t *st7796s = __containerof(panel, st7796s_panel_t, base);

    if (st7796s->reset_gpio_num >= 0) {
        gpio_reset_pin(st7796s->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del st7796s panel @%p", st7796s);
    free(st7796s);
    return ESP_OK;
}

static esp_err_t panel_st7796s_reset(esp_lcd_panel_t *panel)
{
    st7796s_panel_t *st7796s = __containerof(panel, st7796s_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796s->io;

    // perform hardware reset
    if (st7796s->reset_gpio_num >= 0) {
        gpio_set_level(st7796s->reset_gpio_num, st7796s->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(st7796s->reset_gpio_num, !st7796s->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else { // perform software reset
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t data_bytes; // Length of data in above data array; 0xFF = end of cmds.
} lcd_init_cmd_t;

static const lcd_init_cmd_t vendor_specific_init[] = {
    // Enable Inter Register
    
    //ST7796初始化
    {0xF0, {0xC3}, 1},
    {0xF0, {0x96}, 1},
    {0x36, {0x48}, 1},
    {0x3A, {0x55}, 1},//16bit/pixel=0X55
    {0xB4, {0x00}, 1},
    {0xB7, {0xC6}, 1},
    {0xB9, {0x02,0xE0}, 2},		 /*Power control*/
    {0xC0, {0x80,0x64}, 2},		 /*Power control */
    {0xC1, {0x13}, 1}, /*VCOM control*/
    {0xC2, {0xA7}, 1},		 /*VCOM control*/
    {0xC5, {0x08}, 1},		 /*Memory Access Control*/
    {0xE8, {0x40,0x8a,0x00,0x00,0x29,0x19,0xA5,0x33}, 8},		 /*Pixel Format Set*/
    {0xE0, {0xF0, 0x06, 0x0B, 0x07, 0x06, 0x05, 0x2E, 0x33, 0x47, 0x3A, 0x17, 0x16, 0x2E, 0x31}, 14},
    {0XE1, {0xF0, 0x09, 0x0D, 0x09, 0x08, 0x23, 0x2E, 0x33, 0x46, 0x38, 0x13, 0x13, 0x2C, 0x32}, 14},
    {0xF0, {0x3C}, 1},
    {0xF0, {0x69}, 1},
    {0x35, {0x00}, 1},
    {0x21, {0}, 0},
    {0x29, {0}, 0},

    {0x2A, {0x00, 0x00, 0x01, 0x3F}, 4},
    {0x2B, {0x00, 0x00, 0x01, 0xDF}, 4},
    {0x2C, {0}, 0},
    {0, {0}, 0xff},
};

static esp_err_t panel_st7796s_init(esp_lcd_panel_t *panel)
{
    st7796s_panel_t *st7796s = __containerof(panel, st7796s_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796s->io;

    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        st7796s->madctl_val,
    }, 1);
    esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[]) {
        st7796s->colmod_cal,
    }, 1);

    // vendor specific initialization, it can be different between manufacturers
    // should consult the LCD supplier for initialization sequence code
    int cmd = 0;
    while (vendor_specific_init[cmd].data_bytes != 0xff) {
        esp_lcd_panel_io_tx_param(io, vendor_specific_init[cmd].cmd, vendor_specific_init[cmd].data, vendor_specific_init[cmd].data_bytes & 0x1F);
        cmd++;
    }

    return ESP_OK;
}

static esp_err_t panel_st7796s_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    st7796s_panel_t *st7796s = __containerof(panel, st7796s_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = st7796s->io;

    x_start += st7796s->x_gap;
    x_end += st7796s->x_gap;
    y_start += st7796s->y_gap;
    y_end += st7796s->y_gap;

    // define an area of frame memory where MCU can access
    esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4);
    esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
        y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF,
        (y_end - 1) & 0xFF,
    }, 4);
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * st7796s->fb_bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len);

    return ESP_OK;
}

static esp_err_t panel_st7796s_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    st7796s_panel_t *st7796s = __containerof(panel, st7796s_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796s->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}

static esp_err_t panel_st7796s_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    st7796s_panel_t *st7796s = __containerof(panel, st7796s_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796s->io;
    if (mirror_x) {
        st7796s->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        st7796s->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        st7796s->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        st7796s->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        st7796s->madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t panel_st7796s_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    st7796s_panel_t *st7796s = __containerof(panel, st7796s_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796s->io;
    if (swap_axes) {
        st7796s->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        st7796s->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        st7796s->madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t panel_st7796s_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    st7796s_panel_t *st7796s = __containerof(panel, st7796s_panel_t, base);
    st7796s->x_gap = x_gap;
    st7796s->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_st7796s_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    st7796s_panel_t *st7796s = __containerof(panel, st7796s_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7796s->io;
    int command = 0;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    on_off = !on_off;
#endif

    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}
