
# 修改自SPI LCD and Touch Panel Example

[esp_lcd](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html) provides several panel drivers out-of box, e.g. ST7789, SSD1306, NT35510. However, there're a lot of other panels on the market, it's beyond `esp_lcd` component's responsibility to include them all.


## Touch controller XPT2046

In this example you can enable touch controller XPT2046 connected via SPI. 

## Display controller ILI9341
显示芯片ILI9341， 也是spi驱动

## Sensor
max30102：心率 血氧 算法代码
dht11： 环境温湿度
红外测量：原作者参考见注释

## lvgl设计代码
