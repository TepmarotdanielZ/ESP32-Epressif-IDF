/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"

#define BUTTON1_PIN GPIO_NUM_2 /* PIN FOR BUTTON 1 (TO TURN ON THE LEDS) */ 
#define BUTTON2_PIN GPIO_NUM_1 /* PIN FOR BUTTON 2 (TO TURN OFF THE LEDS) */ 
#define LED1_PIN GPIO_NUM_5    /* PIN FOR THE FIRST LED */ 
#define LED2_PIN GPIO_NUM_6    /* PIN FOR THE SECOND LED */ 

/* DEBOUNCE TIMING VARIABLES */ 
#define DEBOUNCE_DELAY 50 /* MILLISECONDS */ 

/* CURRENT BUTTON STATES */ 
static int button1State = 1; /* INITIAL STATE FOR BUTTON 1 (NOT PRESSED) */ 
static int button2State = 1; /* INITIAL STATE FOR BUTTON 2 (NOT PRESSED) */ 
static bool ledState = false; /* INITIAL STATE FOR THE LEDS (OFF) */ 

/* TASK TO HANDLE BUTTON PRESSES */ 
void button_task(void *arg) {
    while (1) {
        int reading1 = gpio_get_level(BUTTON1_PIN); /* READ THE CURRENT STATE OF BUTTON 1 */ 
        int reading2 = gpio_get_level(BUTTON2_PIN); /* READ THE CURRENT STATE OF BUTTON 2 */ 

        /* HANDLE BUTTON 1 (TURN ON LEDS) */ 
        if (reading1 == 0 && button1State == 1) { /* BUTTON 1 PRESSED */ 
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY)); /* DEBOUNCE DELAY */ 
            reading1 = gpio_get_level(BUTTON1_PIN); /* READ AGAIN */ 
            if (reading1 == 0) { /* CONFIRM BUTTON PRESS */ 
                ledState = true; /* SET LED STATE TO ON */ 
                gpio_set_level(LED1_PIN, ledState); /* UPDATE LED 1 */ 
                gpio_set_level(LED2_PIN, ledState); /* UPDATE LED 2 */ 
            }
        }

        /* HANDLE BUTTON 2 (TURN OFF LEDS) */ 
        if (reading2 == 0 && button2State == 1) { /* BUTTON 2 PRESSED */ 
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY)); /* DEBOUNCE DELAY */ 
            reading2 = gpio_get_level(BUTTON2_PIN); /* READ AGAIN */ 
            if (reading2 == 0) { /* CONFIRM BUTTON PRESS */ 
                ledState = false; /* SET LED STATE TO OFF */
                gpio_set_level(LED1_PIN, ledState); /* UPDATE LED 1 */ 
                gpio_set_level(LED2_PIN, ledState); /* UPDATE LED 2 */ 
            }
        }

        /* UPDATE BUTTON STATES FOR THE NEXT ITERATION */ 
        button1State = reading1;
        button2State = reading2;

        vTaskDelay(pdMS_TO_TICKS(10)); /* SHORT DELAY TO PREVENT BUSY-WAITING */ 
    }
}

void app_main(void) {
    /* CONFIGURE THE GPIO PINS */ 
    gpio_set_direction(BUTTON1_PIN, GPIO_MODE_INPUT); /* BUTTON 1 INPUT */ 
    gpio_set_direction(BUTTON2_PIN, GPIO_MODE_INPUT); /* BUTTON 2 INPUT */ 
    gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);   /* LED PIN 1 AS OUTPUT */ 
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);   /* LED PIN 2 AS OUTPUT */ 

    /* INITIALIZE LEDS TO OFF */ 
    gpio_set_level(LED1_PIN, ledState);
    gpio_set_level(LED2_PIN, ledState);

    /* CREATE THE BUTTON HANDLING TASK */ 
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
}
