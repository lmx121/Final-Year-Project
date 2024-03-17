#ifndef __MAIN_H
#define	__MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "lvgl.h"

#include "bsp_gpio.h"

#include "ui.h"
#include "lv_demos.h"
// #include "addr_from_stdin.h"
#include "max30102_luo.h"
#include "MLX90614_API.h"
#include "MLX90614_SMBus_Driver.h"
#include "DHT11_luo.h"


#include "lvgl_task.h"


#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_SMARTCONFIG_BIT      BIT2

#define ADC_GET_SUCCESS     BIT0

#define CONFIG_EXAMPLE_IPV4         1

#define st(x)      do { x } while (__LINE__ == -1)


extern EventGroupHandle_t adc_event_group;
#ifdef __cplusplus
}
#endif


#endif