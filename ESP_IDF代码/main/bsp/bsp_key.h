#ifndef __BSP_KEY_H
#define	__BSP_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include "sdkconfig.h"
#include <stdio.h>
#include "esp_log.h"
#include "main.h"

void bsp_key_init(void);
int get_key_gpio(unsigned char num);
uint8_t bind_key_read(void);
void user_key_read(uint16_t *key,uint16_t *time);
#ifdef __cplusplus
}
#endif


#endif