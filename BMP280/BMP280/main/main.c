

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "BMP280.h"

#define I2C_MASTER_SCL_IO           22 		/* GPIO SCL */
#define I2C_MASTER_SDA_IO           21		/* GPIO SDA */
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000	/* REDUCED TO 100KHZ FOR BETTER STABILITY */

static const char *TAG = "BMP280";

/* INITIALIZE I2C */
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/* SCAN I2C BUS */
static void i2c_scan(void) {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t i = 0; i < 128; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02x", i);
        }
    }
}

void app_main(void) {
	/* INITIALIZE I2C */
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* SCAN I2C BUS */
    i2c_scan();
    vTaskDelay(pdMS_TO_TICKS(100));

    /* INITIALIZE BMP280 */
    bmp280_dev_t bmp280;
    esp_err_t ret = bmp280_init(&bmp280, I2C_MASTER_NUM, BMP280_I2C_ADDRESS_PRIMARY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMP280, trying secondary address");
        ret = bmp280_init(&bmp280, I2C_MASTER_NUM, BMP280_I2C_ADDRESS_SECONDARY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BMP280 initialization failed");
            return;
        }
    }
    ESP_LOGI(TAG, "BMP280 initialized successfully");

    float temperature, pressure, altitude;

    while(1) {
    	/* READ BMP280 DATA */
        if (bmp280_read_data(&bmp280, &temperature, &pressure) == ESP_OK) {
            altitude = bmp280_calculate_altitude(pressure);
            ESP_LOGI(TAG, "Temperature: %.2f°C | Pressure: %.2f hPa | Altitude: %.2f m",
                     temperature, pressure, altitude);
        } else {
            ESP_LOGE(TAG, "Error reading BMP280 data");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}






