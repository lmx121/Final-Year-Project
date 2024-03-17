#include <stdio.h>
/*
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
*/

extern void max30102_init();
void Para_GET();
void Mode_Change(int MODE);
void printf_red();
double butterworth_lowpass_filter(double input, double *prev_inputs, double *prev_outputs, double dt, double cutoff_freq);
void butterworth_lowpass(int START_NUM,int END_NUM);
void moving_average(int length, int window_size);
void find_peaks(int length,int num_peaks);
void printf_peak_indices();
int caculate_bpm();
int get_bpm();
void printf_spo2();
void calculate_Red_DCAC();
void calculate_Ired_DCAC();
double calculate_SPO2();
void SPO2_red_butterworth_lowpass(int START_NUM, int END_NUM);
double butterworth_lowpass_filter_2th(double input, double *prev_inputs, double *prev_outputs, double dt, double cutoff_freq);
void SPO2_ired_butterworth_lowpass(int START_NUM, int END_NUM);
double get_spo2();

