#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN GPIO_NUM_5  /* LED PIN 5 */ 

void app_main(void)
{
    /* CONFIGURE THE LED PIN AS AN OUTPUT */ 
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        /* TURN THE LED ON (OUTPUT HIGH) */ 
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS); /* WAIT FOR 1 SECOND */ 

        /* TURN THE LED OFF (OUTPUT LOW) */ 
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS); /* WAIT FOR 1 SECOND */ 
    }
}
