
#include "bsp_key.h"

#define BIND1_GPIO 13//13
#define BIND2_GPIO 12

#define USER_KEY_GPIO 12

enum key_type{
    NC_KEY = 0,
    BIND1_KEY,
    BIND2_KEY,
    USER_KEY_KEY
};
/*
*********************************************************************************************************
*	函 数 名: bsp_gpio_init
*	功能说明: IO初始化
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_key_init(void)
{
  gpio_reset_pin(BIND1_GPIO);
  gpio_set_direction(BIND1_GPIO, GPIO_MODE_INPUT); 
  gpio_reset_pin(BIND2_GPIO);
  gpio_set_direction(BIND2_GPIO, GPIO_MODE_INPUT); 
  
}
/*
*********************************************************************************************************
*	函 数 名: led_on
*	功能说明: 开灯
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
int get_key_gpio(unsigned char num)
{
  int key=0;
  if(num==1)
  {
    key = gpio_get_level(BIND1_GPIO);
  }else if(num==2)
  {
    key = gpio_get_level(BIND2_GPIO);
  }else if(num==USER_KEY_KEY)
  {
    key = gpio_get_level(USER_KEY_GPIO);
  }
  return key;
}

/*
*********************************************************************************************************
*	函 数 名: key_num_map
*	功能说明: 绑定按键编号映射成物理按键软编号
*	形    参：无
*	返 回 值: 绑定按键触发值
*********************************************************************************************************
*/
uint8_t key_num_map(uint8_t num)
{
    uint8_t key=num;
    
    return key;
}
/*
*********************************************************************************************************
*	函 数 名: bind_key_read
*	功能说明: 外接绑定按键
*	形    参：无
*	返 回 值: 绑定按键触发值
*********************************************************************************************************
*/
uint8_t bind_key_read(void)
{
    uint8_t keys = 0;
    static uint8_t key1_status[2]={0,0};
    static uint8_t key2_status[2]={0,0};

        if(get_key_gpio(1) == 1)
        {
            key1_status[0] = 1;
        }
        else
        {
            key1_status[0] = 0;
        }
        if(key1_status[0] != key1_status[1])
        {
            key1_status[1] = key1_status[0];
            keys = 1;
        }

        if(get_key_gpio(2) == 1)
        {
            key2_status[0] = 1;
        }
        else
        {
            key2_status[0] = 0;
        }
        if(key2_status[0] != key2_status[1])
        {
            key2_status[1] = key2_status[0];
            keys = 2;
        }

    return keys;
}
/*
*********************************************************************************************************
*	函 数 名: user_key_read
*	功能说明: 用户按键
*	形    参：无
*	返 回 值: 绑定按键触发值
*********************************************************************************************************
*/
void user_key_read(uint16_t *key,uint16_t *time)
{
    static uint16_t user_status = 0;
    static uint16_t user_time = 0;
    if(get_key_gpio(USER_KEY_KEY) == 0)
    {
        user_status = 1;
        user_time++;//50ms++
    }
    else
    {
        user_status = 0;
        user_time = 0;
    }
    *key = user_status;
    *time = user_time;
}