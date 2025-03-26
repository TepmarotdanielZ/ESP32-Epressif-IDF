
/* MS5611.c */
#include "MS5611.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

/* MS5611 COMMANDS */
#define CMD_RESET            0x1E
#define CMD_CONV_D1          0x48 /* PRESSURE CONVERSION */
#define CMD_CONV_D2          0x58 /* TEMPERATURE CONVERSION */
#define CMD_ADC_READ         0x00
#define CMD_PROM_READ        0xA2

static uint16_t calibration_data[6];

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void ms5611_reset() {
    uint8_t cmd = CMD_RESET;
    i2c_master_write_to_device(I2C_MASTER_NUM, MS5611_ADDR, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(10));
}

uint32_t ms5611_read_adc() {
    uint8_t data[3];
    uint8_t cmd = CMD_ADC_READ;
    i2c_master_write_read_device(I2C_MASTER_NUM, MS5611_ADDR, &cmd, 1, data, 3, 1000 / portTICK_PERIOD_MS);
    return ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
}

void ms5611_read_prom() {
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t data[2];
        uint8_t cmd = CMD_PROM_READ + (i * 2);
        i2c_master_write_read_device(I2C_MASTER_NUM, MS5611_ADDR, &cmd, 1, data, 2, 1000 / portTICK_PERIOD_MS);
        calibration_data[i] = ((uint16_t)data[0] << 8) | data[1];
    }
}

void ms5611_read_data(float *temperature, float *pressure, float *altitude) {
    uint32_t D1, D2;
    int32_t dT, TEMP;
    int64_t OFF, SENS, P;

    uint8_t cmd = CMD_CONV_D1;
    i2c_master_write_to_device(I2C_MASTER_NUM, MS5611_ADDR, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(10));
    D1 = ms5611_read_adc();

    cmd = CMD_CONV_D2;
    i2c_master_write_to_device(I2C_MASTER_NUM, MS5611_ADDR, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(10));
    D2 = ms5611_read_adc();

    dT = D2 - ((int32_t)calibration_data[4] << 8);
    TEMP = 2000 + ((dT * (int64_t)calibration_data[5]) >> 23);
    OFF = ((int64_t)calibration_data[1] << 16) + ((calibration_data[3] * (int64_t)dT) >> 7);
    SENS = ((int64_t)calibration_data[0] << 15) + ((calibration_data[2] * (int64_t)dT) >> 8);
    P = (((D1 * SENS) >> 21) - OFF) >> 15;

    *temperature = TEMP / 100.0;
    *pressure = P / 100.0;
    *altitude = 44330.0 * (1.0 - pow(*pressure / 1013.25, 0.1903));
}

