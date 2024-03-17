
#include "main.h"

#define MODE_SPO2 1
#define MODE_HR 2

const char *TAG = "main";

EventGroupHandle_t adc_event_group;


void app_main(void)
{
    ESP_LOGI(TAG, "main...");
    // MLX90614初始化
    max30102_init();
    TaskHandle_t Para_handle;
    adc_event_group = xEventGroupCreate();

    xTaskCreate(&lvgl_task, "lvgl_task", 8192, NULL, 5, NULL);
    //xTaskCreate(&DHT11_Task, "DHT11_Task", 8192, NULL, 3, NULL);


    /*
    while (1)
    {
        DHT11(); // 读取温湿度
        printf("Temp=%d.%d℃--Humi=%d.%d%%RH \r\n", Temp, Temp_small, Humi, Humi_small);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
    */

    vTaskDelay(3000 / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(50000));
}
