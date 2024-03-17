#include "max30102_luo.h"
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include <math.h>

static const char *TAG = "max30102";
#define START_NUM_lowpass 0
#define END_NUM_lowpass 320
#define Length_smooth 300
#define window_size_smooth 5
#define Length_findpeaks 280
#define Length_numpeaks 11

#define MAX30102_I2C_SCL 33           // GPIO number used for I2C master clock
#define MAX30102_I2C_SDA 32           // GPIO number used for I2C master data
#define MAX30102_I2C_NUM 0            // I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip
#define MAX30102_I2C_FREQ_HZ 50000    // I2C master clock frequency
#define MAX30102_I2C_TX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define MAX30102_I2C_RX_BUF_DISABLE 0
#define MAX30102_I2C_TIMEOUT_MS 1000

#define MAX30102_GPIO_INT 35 // GPIO number used for MAX30102 int

// | B7 | B6 | B5 | B4 | B3 | B2 | B1 | B0 | WRITE ADDRESS | READ ADDRESS |
// | 1  | 0  | 1  | 0  | 1  | 1  | 1  | R/W| 0xAE          | 0xAF         |
#define MAX30102_ADDR 0x57 //  I2C device MAX30102's 7-bit address
#define MAX30102_PART_ID_REG_ADDR 0xff
int MODE_CHOICE = 0;
// 存放测心率时候的红光光强
int red[360];
int Red_Num = 0;
// 存放测量血氧时候的红光和红外光强
int Red_spo2_num = 0;
int Ired_spo2_num = 0;
// 防止溢出重启，多设一点
int Red_spo2[100];
int Ired_spo2[100];
// 存放测量心率时候的数据
double red_filtered[320];
double Red_Smoothed[300];
int peak_indices[11] = {0};

typedef struct
{
    double dc_component[100];
    double ac_component[100];
} Result_acdc;

Result_acdc ACDC_Red, ACDC_Ired;

static QueueHandle_t gpio_evt_queue = NULL;

/**
 * @brief init the i2c port for MAX30102
 */
static esp_err_t max30102_i2c_init()
{
    int i2c_master_port = MAX30102_I2C_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MAX30102_I2C_SDA,
        .scl_io_num = MAX30102_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MAX30102_I2C_FREQ_HZ,
    };

    // 配置I2C总线，用给定的结构体配置
    i2c_param_config(i2c_master_port, &conf);
    // 安装I2C的驱动
    return i2c_driver_install(i2c_master_port, conf.mode, MAX30102_I2C_RX_BUF_DISABLE, MAX30102_I2C_TX_BUF_DISABLE, 0);
}

/**
 * @brief Read a sequence of bytes from a MAX30102 registers
 */
static esp_err_t max30102_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(MAX30102_I2C_NUM, MAX30102_ADDR, &reg_addr, 1, data, len, MAX30102_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MAX30102 register
 */
static esp_err_t max30102_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(MAX30102_I2C_NUM, MAX30102_ADDR, write_buf, sizeof(write_buf), MAX30102_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

void gpio_intr_task()
{
    uint8_t byte[6];
    int data[2];
    uint8_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            if (MODE_CHOICE == 1)
            {
                ESP_ERROR_CHECK(max30102_register_read(0x07, &byte, 6));
                data[0] = ((byte[0] << 16 | byte[1] << 8 | byte[2]) & 0x03ffff);
                data[1] = ((byte[3] << 16 | byte[4] << 8 | byte[5]) & 0x03ffff);
                // printf("IR: %d, Red: %d\n", data[1], data[0]);
                Red_spo2[Red_spo2_num] = data[0];
                Ired_spo2[Ired_spo2_num] = data[1];
                printf("%d,%d\n", Ired_spo2[Ired_spo2_num], Red_spo2[Red_spo2_num]);
                Red_spo2_num = Red_spo2_num + 1;
                Ired_spo2_num = Ired_spo2_num + 1;
            }
            else if (MODE_CHOICE == 2)
            {
                ESP_ERROR_CHECK(max30102_register_read(0x07, &byte, 3));
                data[0] = ((byte[0] << 16 | byte[1] << 8 | byte[2]) & 0x03ffff);
                red[Red_Num] = data[0];
                printf("%d\n", red[Red_Num]);
                // printf("%d\n", Red_Num);
                Red_Num = Red_Num + 1;
            }
        }
    }
}


void printf_red()
{
    printf("原始red光强\n");
    int a = 0;
    while (1)
    {
        printf("%d\n", red[a]);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        a = a + 1;
        printf("a=%d\n", a);

        if (a == 340)
        {
            vTaskDelete(NULL);
        }
    }
}

void Mode_Change(int MODE)
{
    if (MODE == 1)
    {
        ESP_ERROR_CHECK(max30102_register_write_byte(0x09, 0x03)); // MODE = 0b011: SpO2 mode，
        MODE_CHOICE = MODE;
    }
    else if (MODE == 2)
    {
        ESP_ERROR_CHECK(max30102_register_write_byte(0x09, 0x02)); // 心率是0x01
        MODE_CHOICE = MODE;
    }
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/**
 * @brief init the gpio intr for MAX30102
 */
static esp_err_t max30102_gpio_intr_init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MAX30102_GPIO_INT);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // start gpio task
    xTaskCreate(gpio_intr_task, "gpio_intr_task", 2048, NULL, 10, NULL);
    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(MAX30102_GPIO_INT, gpio_isr_handler, (void *)MAX30102_GPIO_INT);
    return ESP_OK;
}

void Para_GET()
{
    uint8_t byte1[1];
    uint8_t Times = 0;
    max30102_register_read(0x09, &byte1, 1);
    // printf("value of register:%d\n",byte1[0]);
    if (byte1[0] == 2)
        Times = 170; // 一次采样个点，采样170次,340个点
    else if (byte1[0] == 3)
        Times = 50;//血氧采样100个点
    // printf("value of times:%d\n",Times);
    while (1)
    {
        Times = Times - 1;
        uint8_t byte[2];
        float temp;

        // 使能一次温度采集
        ESP_ERROR_CHECK(max30102_register_write_byte(0x21, 0x01));
        // 延时可以改
        vTaskDelay(50 / portTICK_PERIOD_MS);
        // 温度采集
        // 读0x20清除中断标志位
        ESP_ERROR_CHECK(max30102_register_read(0x1f, &byte[0], 1));
        ESP_ERROR_CHECK(max30102_register_read(0x20, &byte[1], 1));
        temp = (int8_t)(byte[0]) + byte[1] * 0.0625;
        //printf("Temperature:%f\n", temp);

        // FIFO
        ESP_ERROR_CHECK(max30102_register_write_byte(0x04, 0x00)); // clear FIFO Write Pointer
        ESP_ERROR_CHECK(max30102_register_write_byte(0x05, 0x00)); // clear FIFO Overflow Counter
        ESP_ERROR_CHECK(max30102_register_write_byte(0x06, 0x00)); // clear FIFO Read Pointer

        // clear PPG_RDY ! Cannot receive the first interrupt without clearing !
        uint8_t data;
        ESP_ERROR_CHECK(max30102_register_read(0x00, &data, 1));
        ESP_ERROR_CHECK(max30102_register_read(0x01, &data, 1));

        //printf("value of times:%d\n", Times);
        if (!Times)
        {
            //vTaskDelete(NULL);
            Red_Num = 0;
            Red_spo2_num = 0;
            Ired_spo2_num = 0;
            break;
        }
    }
}

void max30102_init()
{
    ESP_ERROR_CHECK(max30102_i2c_init());
    ESP_LOGI(TAG, "MAX30102 I2C initialized successfully");
    max30102_gpio_intr_init();
    ESP_LOGI(TAG, "MAX30102 GPIO INTR initialized successfully");

    // reset
    ESP_ERROR_CHECK(max30102_register_write_byte(0x09, 0x40));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Interrupt Enable
    ESP_ERROR_CHECK(max30102_register_write_byte(0x02, 0xc0)); // enable interrupts: A_FULL: FIFO Almost Full Flag and PPG_RDY: New FIFO Data Ready
    ESP_ERROR_CHECK(max30102_register_write_byte(0x03, 0x02)); // enable interrupt: DIE_TEMP_RDY: Internal Temperature Ready Flag

    // FIFO
    ESP_ERROR_CHECK(max30102_register_write_byte(0x04, 0x00)); // clear FIFO Write Pointer
    ESP_ERROR_CHECK(max30102_register_write_byte(0x05, 0x00)); // clear FIFO Overflow Counter
    ESP_ERROR_CHECK(max30102_register_write_byte(0x06, 0x00)); // clear FIFO Read Pointer

    // FIFO Configuration
    ESP_ERROR_CHECK(max30102_register_write_byte(0x08, 0x0f)); // SMP_AVE = 0b000: 1 averaging, FIFO_ROLLOVER_EN = 0, FIFO_A_FULL = 0xf

    // Mode Configuration
    ESP_ERROR_CHECK(max30102_register_write_byte(0x09, 0x03)); // MODE = 0b011: SpO2 mode，

    // SpO2 Configuration
    ESP_ERROR_CHECK(max30102_register_write_byte(0x0a, 0x47)); // SPO2_ADC_RGE = 0b10: 8192, SPO2_SR = 0b001: 100 SAMPLES PER SECOND,
                                                               // LED_PW = 0b11: PULSE WIDTH 411, ADC RESOLUTION 18

    // LED Pulse Amplitude
    ESP_ERROR_CHECK(max30102_register_write_byte(0x0c, 0x50)); // LED1_PA(red) = 0x24, LED CURRENT 16mA
    ESP_ERROR_CHECK(max30102_register_write_byte(0x0d, 0x50)); // LED2_PA(IR) = 0x24, LED CURRENT 16mA
    ESP_ERROR_CHECK(max30102_register_write_byte(0x10, 0x50)); // PILOT_PA = 0x24, LED CURRENT 16mA

    // clear PPG_RDY ! Cannot receive the first interrupt without clearing !
    uint8_t data;
    ESP_ERROR_CHECK(max30102_register_read(0x00, &data, 1));
    ESP_LOGI(TAG, "Interrupt Status 1: 0x%x", data);
    ESP_ERROR_CHECK(max30102_register_read(0x01, &data, 1));
    ESP_LOGI(TAG, "Interrupt Status 2: 0x%x", data);
}

// 低通滤波器
double butterworth_lowpass_filter(double input, double *prev_inputs, double *prev_outputs, double dt, double cutoff_freq)
{
    // 计算滤波器系数
    double omega_c = 1.0 / (2.0 * 3.14159265358979323846 * cutoff_freq);
    double alpha = omega_c * dt / (1.0 + omega_c * dt);

    // 计算输出
    double output = alpha * (input + prev_inputs[0]) + (1.0 - alpha) * prev_outputs[1];

    // 更新先前的输入和输出
    prev_inputs[0] = input;
    prev_outputs[0] = output;
    prev_outputs[1] = prev_outputs[0];
    

    return output;
}

void butterworth_lowpass(int START_NUM, int END_NUM)
{
    // 设定巴特沃斯低通滤波器参数
    double cutoff_freq = 0.01; // 低通滤波器的截止频率
    double dt = 1.0;           // 采样时间间隔

    // 初始化先前的输入和输出
    double prev_inputs[1] = {0.0};
    double prev_outputs[2] = {0.0};

    int red_before_Low_pass[320];

    // 将 red 数组的第21到340个数据存储到 red_before_Low_pass 数组中,red_before_Low_pass数组长度280
    for (int i = 20; i < 340; ++i)
    {
        red_before_Low_pass[i - 20] = red[i];
    }

    // 应用滤波器
    printf("Filtered Data:————————————————————————————————————————————————————————————————————————\n");
    for (int i = START_NUM; i < END_NUM; ++i)
    {
        double filtered_value = butterworth_lowpass_filter(red_before_Low_pass[i], prev_inputs, prev_outputs, dt, cutoff_freq);
        red_filtered[i] = filtered_value;
        printf("%f\n", red_filtered[i]);
    }
}

void moving_average(int length, int window_size)
{
    // red_filtered长度320
    //  将 red 数组的第21到340个数据存储到 red_before_Low_pass 数组中,red_before_Low_pass数组长度280
    printf("平滑滤波后的数据——————————————————————————————————————————————————————————————————————————\n");
    float red_before_smooth[300];
    for (int i = 20; i < 320; ++i)
    {
        red_before_smooth[i - 20] = red_filtered[i];
    }
    for (int i = 0; i < length; ++i)
    {
        int sum = 0;
        int count = 0;
        // 计算窗口内的和
        for (int j = i - window_size / 2; j <= i + window_size / 2; ++j)
        {
            if (j >= 0 && j < length)
            {
                sum += red_before_smooth[j];
                count++;
            }
        }
        // 计算平均值并存储到输出数组
        Red_Smoothed[i] = sum / count;
        printf("%f\n", Red_Smoothed[i]);
    }
}

void find_peaks(int length, int num_peaks)
{
    float red_before_findpeak[280];
    for (int i = 20; i < 300; ++i)
    {
        red_before_findpeak[i - 20] = red_filtered[i];
    }
    int peak_count = 0;
    int i;

    for (i = 1; i < length - 1; ++i)
    {
        if (red_before_findpeak[i] > red_before_findpeak[i - 1] && red_before_findpeak[i] > red_before_findpeak[i + 1])
        {
            // 发现峰值
            peak_indices[peak_count++] = i;

            // 如果找到足够的峰值，就退出循环
            if (peak_count == num_peaks)
            {
                break;
            }
        }
    }
    // 如果未找到足够的峰值，将多余的数组下标设置为-1
    for (; peak_count < num_peaks; ++peak_count)
    {
        peak_indices[peak_count] = -1;
    }
}

void printf_peak_indices()
{
    // 打印找到的峰值的数组下标
    printf("Peak indices:————————————————————————————\n");
    for (int i = 0; i < 11; ++i)
    {
        if (peak_indices[i] != -1)
        {
            printf("%d\n", peak_indices[i]);
        }
    }
}

int caculate_bpm()
{   
    int result = 10500 / (peak_indices[10] - peak_indices[0]);
    printf("bpm = %d\n", result);
    return result;
}

int get_bpm()
{   
    int HR_result;
    butterworth_lowpass(START_NUM_lowpass, END_NUM_lowpass);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    moving_average(300, 5);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    find_peaks(Length_findpeaks, Length_numpeaks);
    printf_peak_indices();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    HR_result = caculate_bpm();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return HR_result;
}

//----------------------------------------------------------------------------------------------------------------
//以下为血氧计算代码

//输出测得的光强
void printf_spo2()
{
    for (int i = 0; i < 100; i++)
    {
        printf("原始红光光强：%d\n", Red_spo2[i]);
    }
    for (int i = 0; i < 100; i++)
    {
        printf("原始红外光光强：%d\n", Ired_spo2[i]);
    }
}

// 二阶巴特沃斯低通滤波器
double butterworth_lowpass_filter_2th(double input, double *prev_inputs, double *prev_outputs, double dt, double cutoff_freq)
{
    // 计算滤波器系数
    double omega_c = 1.0 / (2.0 * 3.14159265358979323846 * cutoff_freq);
    double alpha = omega_c * dt / (1.0 + omega_c * dt);
    double beta = alpha / (1.0 + omega_c * dt);

    // 计算输出
    double output = beta * (input + 2.0 * prev_inputs[0] + prev_inputs[1])
                    + (2.0 - 2.0 * alpha) * prev_outputs[2]
                    + (1.0 - alpha) * prev_outputs[1];

    // 更新先前的输入和输出
    prev_inputs[1] = prev_inputs[0];
    prev_inputs[0] = input;

    prev_outputs[2] = prev_outputs[1];
    prev_outputs[1] = output;

    return output;
}

//低通滤波得到直流
void SPO2_red_butterworth_lowpass(int START_NUM, int END_NUM)
{
    // 设定巴特沃斯低通滤波器参数
    double cutoff_freq = 0.01; // 低通滤波器的截止频率
    double dt = 1.0;           // 采样时间间隔

    // 初始化先前的输入和输出
    double prev_inputs[2] = {0.0, 0.0};
    double prev_outputs[3] = {0.0, 0.0, 0.0};


    // 应用滤波器
    printf("Filtered Data Red 直流:————————————————————————————————————————————————————————————————————————\n");
    for (int i = START_NUM; i < END_NUM; ++i)
    {
        double filtered_value = butterworth_lowpass_filter(Red_spo2[i], prev_inputs, prev_outputs, dt, cutoff_freq);
        ACDC_Red.dc_component[i] = filtered_value;
        ACDC_Red.ac_component[i] = Red_spo2[i]-ACDC_Red.dc_component[i];
        printf("%f\n", ACDC_Red.dc_component[i]);
    }
}

//红外光低通滤波
void SPO2_ired_butterworth_lowpass(int START_NUM, int END_NUM)
{
    // 设定巴特沃斯低通滤波器参数
    double cutoff_freq = 0.01; // 低通滤波器的截止频率
    double dt = 1.0;           // 采样时间间隔

    // 初始化先前的输入和输出
    double prev_inputs[2] = {0.0, 0.0};
    double prev_outputs[3] = {0.0, 0.0, 0.0};


    // 应用滤波器
    printf("Filtered Data IRED 直流:————————————————————————————————————————————————————————————————————————\n");
    for (int i = START_NUM; i < END_NUM; ++i)
    {
        double filtered_value = butterworth_lowpass_filter(Ired_spo2[i], prev_inputs, prev_outputs, dt, cutoff_freq);
        ACDC_Ired.dc_component[i] = filtered_value;
        ACDC_Ired.ac_component[i] = Ired_spo2[i]-ACDC_Ired.dc_component[i];
        printf("%f\n", ACDC_Ired.dc_component[i]);
    }
}

double calculate_SPO2()
{   
    double sum = 0;
    double s;
    double result;
    printf("血氧值----------------------------------------------------------\n");
    for(int i = 0;i<100;i++)
    {
    s = (fabs(ACDC_Red.ac_component[i]) / ACDC_Red.dc_component[i]) / (fabs(ACDC_Ired.ac_component[i]) / ACDC_Ired.dc_component[i]);
    result = 115 - 17* s;
    sum += result;
    printf("%f\n", result);      
    }    
    double mean = sum / 100;
    printf("血氧值：%.2f\n", mean);
    return mean;
}

double get_spo2()
{   
    double spo2_result;
    //printf_spo2();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    SPO2_red_butterworth_lowpass(0,100);
    SPO2_ired_butterworth_lowpass(0,100);
    spo2_result = calculate_SPO2();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return spo2_result;
}