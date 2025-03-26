
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MS5611.h"

void app_main() {
    float temperature, pressure, altitude;

    i2c_master_init();
    ms5611_reset();
    ms5611_read_prom();

    while (1) {
        ms5611_read_data(&temperature, &pressure, &altitude);
        printf("Temperature: %.2f°C | Pressure: %.2f hPa | Altitude: %.2f m\n", temperature, pressure, altitude);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
