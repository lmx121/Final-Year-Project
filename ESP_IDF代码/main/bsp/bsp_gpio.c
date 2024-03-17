﻿
*********************************************************************************************************
*/
#include "bsp_gpio.h"

/*
*********************************************************************************************************
*	函 数 名: bsp_gpio_init
*	功能说明: IO初始化
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_gpio_init(void)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT); 

    
   //  gpio_reset_pin(48);
   //  gpio_set_direction(48, GPIO_MODE_OUTPUT); 
   //  gpio_set_level(48, 1);
}
/*
*********************************************************************************************************
*	函 数 名: led_on
*	功能说明: 开灯
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void led_on(void)
{
   gpio_set_level(BLINK_GPIO, 0);
}
/*
*********************************************************************************************************
*	函 数 名: led_off
*	功能说明: 关灯
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void led_off(void)
{
   gpio_set_level(BLINK_GPIO, 1);
}
