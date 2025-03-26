#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "dht11.h"

/* NETWORK CONFIGURATION */
#define WIFI_SSID           "Neatic"					/* REPLACE WITH YOUR WIFI SSID */
#define WIFI_PASSWORD       "011570035"					/* REPLACE WITH YOUR WIFI PASSWORD */
#define MQTT_BROKER         "chiptree-iot.aifarm.dev" 	/* REPLACE WITH YOUR THINGSBOARD MQTT_BROKER */
#define MQTT_PORT           1883
#define MQTT_USERNAME       "MfeoqvKMLqA13fpg2JIJ"    	/* REPLACE WITH YOUR THINGSBOARD TOKEN */
#define MQTT_TOPIC          "v1/devices/me/telemetry" 	/* REPLACE WITH YOUR THINGSBOARD MQTT_TOPIC */

/* EVENT FLAGS FOR WI-FI CONNECTION */
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static const char *TAG = "MQTT_TELEMETRY";

/* MQTT CLIENT HANDLER */
static esp_mqtt_client_handle_t mqtt_client = NULL;

/* WI-FI EVENT HANDLER */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Reconnecting to WiFi...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* MQTT EVENT HANDLER */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT message sent successfully");
            break;
        default:
            break;
    }
}

/* WI-FI INITIALIZATION */
void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization completed");
}

/* MQTT INITIALIZATION */
void mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://chiptree-iot.aifarm.dev:1883",
        .username = MQTT_USERNAME,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

/* PUBLISH SENSOR DATA FUNCTION */
void publish_sensor_data(int temperature, int humidity) {
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"temperature\":%d, \"humidity\":%d}", temperature, humidity);

    /* SERIAL PRINT THE SENSOR PAYLOAD */
    printf("Publishing sensor data payload: %s\n", payload);

    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
    ESP_LOGI(TAG, "Published message with ID: %d", msg_id);
}

void app_main(void) {
    /* INITIALIZE NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* INITIALIZE WI-FI */
    wifi_init_sta();

    /* WAIT FOR WI-FI CONNECTION */
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi");

        /* INITIALIZE MQTT */
        mqtt_init();
        DHT11_init(GPIO_NUM_8); /* INITIALIZE DHT11 SENSOR ON GPIO 8 */

        /* MAIN LOOP TO READ SENSOR AND PUBLISH DATA */
        while (1) {
            struct dht11_reading data = DHT11_read();
            printf("Temperature: %d  | Humidity: %d\n", data.temperature, data.humidity);
            publish_sensor_data(data.temperature, data.humidity); /* PUBLISH TO MQTT */
            vTaskDelay(pdMS_TO_TICKS(1000));  /* DELAY FOR 5 SECONDS */
        }
    }
}
