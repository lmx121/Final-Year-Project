# Final Year Project
 Intelligent detection equipment - including embedded ESP-IDF code, PCB board hardware, android studio code, and model deployment
## ESI-IDF部分修改自SPI LCD and Touch Panel Example
[esp_lcd](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html)
### Touch controller XPT2046
In this example you can enable touch controller XPT2046 connected via SPI. 
### Display controller ILI9341
显示芯片ILI9341， 也是spi驱动
### Sensor
max30102：心率 血氧 算法代码   
dht11： 环境温湿度   
红外测量：原作者参考见注释   
### lvgl设计代码

## android模型训练及部署
###  模型训练及格式转换
用scikit - learn以及tfl
### 模型部署到android
主要看main的kotin文件   
以及layout_xml文件

## 硬件
（待更新）

## web模型部署
（待更新）

