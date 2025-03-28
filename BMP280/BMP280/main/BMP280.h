#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"

#define BMP280_I2C_ADDRESS_PRIMARY   0x76	/* TRY 0x76 IF 0x77  DOESN'T WORK*/
#define BMP280_I2C_ADDRESS_SECONDARY 0x77
#define BMP280_CHIP_ID               0x58	/* EXPECTED CHIP ID */

/* REGISTER ADDRESSES */
#define BMP280_REG_ID               0xD0	/* ID REGISTER */
#define BMP280_REG_RESET            0xE0
#define BMP280_REG_STATUS           0xF3
#define BMP280_REG_CONTROL          0xF4
#define BMP280_REG_CONFIG           0xF5
#define BMP280_REG_PRESS_MSB        0xF7
#define BMP280_REG_TEMP_MSB         0xFA
#define BMP280_REG_CALIB            0x88

/* CALIBRATION DATA STRUCTURE */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    int32_t  t_fine;
} bmp280_calib_data_t;

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    bmp280_calib_data_t calib_data;
} bmp280_dev_t;

/* FUNCTION PROTOTYPES */
esp_err_t bmp280_init(bmp280_dev_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr);
esp_err_t bmp280_check_id(bmp280_dev_t *dev);
esp_err_t bmp280_read_calibration_data(bmp280_dev_t *dev);
esp_err_t bmp280_set_config(bmp280_dev_t *dev, uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter);
esp_err_t bmp280_read_data(bmp280_dev_t *dev, float *temperature, float *pressure);
float bmp280_calculate_altitude(float pressure);

#endif /* BMP280_H */
