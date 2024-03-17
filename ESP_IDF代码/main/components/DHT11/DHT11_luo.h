#include <stdio.h>
#ifndef DHT11_H_  
#define DHT11_H_

#define ets_delay_us esp_rom_delay_us
#define DHT11_PIN   0//定义DHT11的引脚
 
#define uchar unsigned char
#define uint8 unsigned char
#define uint16 unsigned short

extern uchar ucharFLAG,uchartemp;
extern uchar Humi,Humi_small,Temp,Temp_small;
extern uchar ucharT_data_H,ucharT_data_L,ucharRH_data_H,ucharRH_data_L,ucharcheckdata;
extern uchar ucharT_data_H_temp,ucharT_data_L_temp,ucharRH_data_H_temp,ucharRH_data_L_temp,ucharcheckdata_temp;
extern uchar ucharcomdata;

static void InputInitial();

static void OutputHigh();

static void OutputLow();

static uint8 getData();

static void COM();

void Delay_ms(uint16 ms);

void DHT11();

void DHT11_Task(void *pvParameters);

#endif