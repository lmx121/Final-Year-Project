/*
*********************************************************************************************************
*
*	模块名称 : GUI任务
*	文件名称 : lvgl_task.c
*	版    本 : V1.0
*	说    明 :
*   联系方式  ：嘉友创-小步（微信：15962207161）
*
*	修改记录 :
*	版本号          日期        		作者     		说明
*	V1.0           2022-09-08 		  DOUBLE  		 正式发布
*********************************************************************************************************
*/
#include "main.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"

#include "lvgl.h"
#include "bsp_gpio.h"

#include "ui.h"
#include "lv_demos.h"

static const char *TAG = "lvglTask";

#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
#include "esp_lcd_ili9341.h"
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
#include "esp_lcd_gc9a01.h"
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_ST7796S
#include "esp_lcd_st7796s.h"
#endif

#if CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_STMPE610
#include "esp_lcd_touch_stmpe610.h"
#endif

#include "esp_lcd_touch_xpt2046.h"

// Using SPI2 in the example
#define LCD_HOST SPI2_HOST
#define TOUCH_HOST SPI3_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK 14
#define EXAMPLE_PIN_NUM_MOSI 13
#define EXAMPLE_PIN_NUM_MISO 12
#define EXAMPLE_PIN_NUM_LCD_DC 2
#define EXAMPLE_PIN_NUM_LCD_RST 4
#define EXAMPLE_PIN_NUM_LCD_CS 15
#define EXAMPLE_PIN_NUM_BK_LIGHT 16
//touch io口
#define EXAMPLE_PIN_NUM_TOUCH_CS 5
#define EXAMPLE_PIN_NUM_TOUCH_SCLK 18
#define EXAMPLE_PIN_NUM_TOUCH_MOSI 23
#define EXAMPLE_PIN_NUM_TOUCH_MISO 19
#define EXAMPLE_PIN_NUM_TOUCH_IRQ 25

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 320
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
#define EXAMPLE_LCD_H_RES 320
#define EXAMPLE_LCD_V_RES 480
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_ST7796S
#define EXAMPLE_LCD_H_RES 320
#define EXAMPLE_LCD_V_RES 480
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

// 否则断言失败
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
esp_lcd_touch_handle_t tp = NULL;
#endif

static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;      // contains callback functions

esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
    .rgb_endian = LCD_RGB_ENDIAN_BGR,
    .bits_per_pixel = 16,
};




void scan_xpt2046(void);
void bsp_touch_init(void);
static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
static void ui_Screen1_cb(lv_timer_t *timer);

/*
*********************************************************************************************************
*	函 数 名: bsp_lcd_init
*	功能说明: 液晶屏硬件初始化
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_lcd_init(void)
{
    // 背光初始化
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    // spi初始化
    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(uint16_t), // 80
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 乐鑫LCD接口配置：其他IO初始化、时钟等
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // 乐鑫lcd接口初始化：也可以自己SPI操作初始化液晶屏
    // 使用乐鑫的gc9a01框架，复制修改为7796s
    ESP_LOGI(TAG, "Install ili9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
}
/*
*********************************************************************************************************
*	函 数 名: bsp_touch_init
*	功能说明: 触摸初始化
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_touch_init(void)
{   
    //重新初始化一个SPI留给触摸
    ESP_LOGI(TAG, "Initialize Touch SPI bus");
    spi_bus_config_t touch_buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_TOUCH_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_TOUCH_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_TOUCH_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(uint16_t), // 80
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TOUCH_HOST, &touch_buscfg, SPI_DMA_CH_AUTO));

    // touch的io配置
    // esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(EXAMPLE_PIN_NUM_TOUCH_CS);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TOUCH_HOST, &tp_io_config, &tp_io_handle));

    // Create a new XPT2046 touch driver 创建一个新的触摸控制器实例
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller XPT2046");
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp));
}
/*
*********************************************************************************************************
*	函 数 名: scan_xpt2046
*	功能说明: 读取触摸状态和触摸值
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void scan_xpt2046(void)
{
    ESP_ERROR_CHECK(esp_lcd_touch_read_data(tp));
}

/*
*********************************************************************************************************
*	函 数 名: example_notify_lvgl_flush_ready
*	功能说明: 刷新完成
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}
/*
*********************************************************************************************************
*	函 数 名: example_lvgl_flush_cb
*	功能说明: 刷新显存
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}
/*
*********************************************************************************************************
*	函 数 名: example_lvgl_port_update_callback
*	功能说明: 旋转屏和触摸
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void example_lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    drv->rotated = 0;
    switch (drv->rotated)
    {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, true);
        esp_lcd_touch_set_mirror_x(tp, true);
#endif
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    }
}
/*
*********************************************************************************************************
*	函 数 名: example_lvgl_touch_cb
*	功能说明: LVGL触摸回调：根据设置定时获取触摸状态和触摸值
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t x[1];
    uint16_t y[1];
    uint16_t  strength[1];
    uint8_t count = 0;
    // 扫描触摸点，存储到SRAM
    scan_xpt2046();
    // 检索触摸点
    data->state = LV_INDEV_STATE_REL;
    if (esp_lcd_touch_get_coordinates(tp, x, y, strength, &count, 1))
    {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PR;
    }
    data->continue_reading = false;
    //printf("x: %d, y: %d\n", x[0], y[0]);
}
/*
*********************************************************************************************************
*	函 数 名: example_increase_lvgl_tick
*	功能说明: LVGL心跳：使用idf软定时器实现
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}
/*
*********************************************************************************************************
*	函 数 名: lvgl_task
*	功能说明: GUI任务
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void lvgl_task(void *parm)
{
    uint32_t tick_bk = 0;
    ESP_LOGI(TAG, "Initialize lcd bsp");
    // 硬件初始化
    bsp_lcd_init();
    bsp_touch_init();

    // LVGL初始化
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 96 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 96 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 96);

    // LVGL显示注册
    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    // 旋转0,并镜像xy触摸坐标
    disp_drv.rotated = 0;
    example_lvgl_port_update_callback(&disp_drv);

    // LVGL心跳：使用idf软定时器
    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    // LVGL输入注册
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;
    lv_indev_drv_register(&indev_drv);
#endif

    // 运行demo
    ui_init();
    // 定时器重绘demo的控件
    lv_timer_create(ui_Screen1_cb, 100, NULL);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (tick_bk++ > 20)
        {
            // 延时打开背光
            ESP_LOGI(TAG, "Turn on LCD backlight");
            gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
            break;
        }

        lv_timer_handler();
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();
    }
}


// 刷新显示
static void ui_Screen1_cb(lv_timer_t *timer)
{
    // ESP_LOGI(TAG, "timer ui_Screen1_cb");
    // 当前值
}
