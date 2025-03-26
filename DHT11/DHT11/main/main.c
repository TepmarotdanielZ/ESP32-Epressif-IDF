

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "dht11.h"

void app_main()
{
    DHT11_init(GPIO_NUM_8);

    while(1) {
        struct dht11_reading data = DHT11_read();
        printf("Temperature: %d  | Humidity: %d\n", data.temperature, data.humidity);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
