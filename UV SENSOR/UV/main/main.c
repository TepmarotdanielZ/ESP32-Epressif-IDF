#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SENSOR_PIN ADC1_CHANNEL_5   /* GPIO33 (ADC1_CH5) */
#define DEFAULT_VREF 1100           /* DEFAULT REFERENCE VOLTAGE IN mV */
#define NO_OF_SAMPLES 64            /* MULTISAMPLING */

static const char *TAG = "UV_SENSOR";
static esp_adc_cal_characteristics_t *adc_chars;

void init_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_PIN, ADC_ATTEN_DB_11);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

void read_uv_sensor() {
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw(SENSOR_PIN);
    }
    adc_reading /= NO_OF_SAMPLES;

    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    float uv_intensity = (voltage / 2500.0) * 1000.0;  /* CONVERT TO µW/cm² */

    ESP_LOGI(TAG, "Raw Value: %d | Voltage: %.2f mV | UV Intensity: %.2f µW/cm²", adc_reading, voltage / 1000.0, uv_intensity);
}

void app_main() {
    init_adc();
    while (1) {
        read_uv_sensor();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
