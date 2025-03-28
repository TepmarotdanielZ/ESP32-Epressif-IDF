#include "BMP280.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "math.h"

static const char *TAG = "BMP280";

/* INITIALIZE BMP280 */
esp_err_t bmp280_init(bmp280_dev_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr) {
    dev->i2c_port = i2c_port;
    dev->i2c_addr = i2c_addr;

    /* CHECK DEVICE ID */
    esp_err_t ret = bmp280_check_id(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to verify BMP280 ID");
        return ret;
    }

    /* READ CALIBRATION DATA */
    ret = bmp280_read_calibration_data(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP280 calibration data");
        return ret;
    }

    /* SET DEFAULT CONFIGURATION (NORMAL MODE, 16x OVERSAMPLING FOR BOTH TEMPERATURE AND PRESSURE) */
    ret = bmp280_set_config(dev, 0x05, 0x05, 0x03, 0x00, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BMP280");
        return ret;
    }

    return ESP_OK;
}

/* CHECK BMP280 ID */
esp_err_t bmp280_check_id(bmp280_dev_t *dev) {
    uint8_t id;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_ID, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &id, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        if (id == BMP280_CHIP_ID) {
            ESP_LOGI(TAG, "BMP280 chip ID verified: 0x%02x", id);
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Wrong chip ID: expected 0x%02x, got 0x%02x", BMP280_CHIP_ID, id);
            return ESP_FAIL;
        }
    }
    return ret;
}

/* READ CALIBRATION DATA */
esp_err_t bmp280_read_calibration_data(bmp280_dev_t *dev) {
    uint8_t calib_data[24];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CALIB, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, calib_data, 24, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    /* PARSE CALIBRATION DATA */
    dev->calib_data.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    dev->calib_data.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    dev->calib_data.dig_T3 = (calib_data[5] << 8) | calib_data[4];
    dev->calib_data.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    dev->calib_data.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    dev->calib_data.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    dev->calib_data.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    dev->calib_data.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    dev->calib_data.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    dev->calib_data.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    dev->calib_data.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    dev->calib_data.dig_P9 = (calib_data[23] << 8) | calib_data[22];

    return ESP_OK;
}

/* CONFIGURE BMP280 */
esp_err_t bmp280_set_config(bmp280_dev_t *dev, uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter) {
    /* ctrl_meas register (osrs_t[7:5], osrs_p[4:2], mode[1:0]) */
    uint8_t ctrl_meas = (osrs_t << 5) | (osrs_p << 2) | mode;

    /* CONFIGURE REGISTER  (t_sb[7:5], filter[4:2], spi3w_en[0]) */
    uint8_t config = (t_sb << 5) | (filter << 2);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CONTROL, true);
    i2c_master_write_byte(cmd, ctrl_meas, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CONFIG, true);
    i2c_master_write_byte(cmd, config, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/* READ TEMPERATURE AND PRESSURE */
esp_err_t bmp280_read_data(bmp280_dev_t *dev, float *temperature, float *pressure) {
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_PRESS_MSB, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
        int32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);

        /* TEMPERATURE CALCULATION */
        int32_t var1 = ((((adc_T >> 3) - ((int32_t)dev->calib_data.dig_T1 << 1))) * ((int32_t)dev->calib_data.dig_T2)) >> 11;
        int32_t var2 = (((((adc_T >> 4) - ((int32_t)dev->calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->calib_data.dig_T1))) >> 12) * ((int32_t)dev->calib_data.dig_T3)) >> 14;
        dev->calib_data.t_fine = var1 + var2;
        *temperature = (float)((dev->calib_data.t_fine * 5 + 128) >> 8) / 100.0;

        /* PRESSURE CALCULATION */
        var1 = (((int32_t)dev->calib_data.t_fine) >> 1) - 64000;
        var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dev->calib_data.dig_P6);
        var2 = var2 + ((var1 * ((int32_t)dev->calib_data.dig_P5)) << 1);
        var2 = (var2 >> 2) + (((int32_t)dev->calib_data.dig_P4) << 16);
        var1 = (((dev->calib_data.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dev->calib_data.dig_P2) * var1) >> 1)) >> 18;
        var1 = ((((32768 + var1)) * ((int32_t)dev->calib_data.dig_P1)) >> 15);
        if (var1 == 0) return ESP_FAIL;

        uint32_t p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
        if (p < 0x80000000) {
            p = (p << 1) / ((uint32_t)var1);
        } else {
            p = (p / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t)dev->calib_data.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
        var2 = (((int32_t)(p >> 2)) * ((int32_t)dev->calib_data.dig_P8)) >> 13;
        p = (uint32_t)((int32_t)p + ((var1 + var2 + dev->calib_data.dig_P7) >> 4));
        *pressure = (float)p / 100.0; /* CONVERT TO (hPa) */
    }
    return ret;
}

/* CALCULATE ALTITUDE BASED ON PRESSURE */
float bmp280_calculate_altitude(float pressure) {
    return (1 - powf(pressure / 1013.25f, 1 / 5.257f)) * 288.15f / 0.0065f;
}
