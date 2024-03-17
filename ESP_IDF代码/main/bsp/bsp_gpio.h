#ifndef __BSP_GPIO_H
#define	__BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include "sdkconfig.h"
#include <stdio.h>

#define BLINK_GPIO 1
void bsp_gpio_init(void);
void led_on(void);
void led_off(void);
#ifdef __cplusplus
}
#endif


#endif