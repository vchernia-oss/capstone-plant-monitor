#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "constants.h"
#include "secrets.h"

#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_18

static const char *TAG = "PLANT_SYSTEM";

typedef struct {  
    float temperature;
    uint16_t light_level;
    uint16_t humidity;
    uint16_t moisture;
    bool water_level;
    uint32_t raw_id;      
} SensorData;

void hardware_init(void); //declaring functions
void can_driver_init(void);
bool can_driver_read_sensor(SensorData *out_data);
void wifi_init(void);
void publish_all_sensors(SensorData *data);
void process_sensor_data(SensorData *data, bool *pump_state, bool *light_state);
void update_hardware_actuators(bool pump_state, bool light_state);
bool read_water_level_sensor(void);

void app_main(void) {
    can_driver_init(); 
    hardware_init(); 
    wifi_init(); 

    SensorData current_sensor_data = {0};
    bool is_pump_active = false;
    bool is_light_active = false;
    int64_t last_adafruit_post = 0;
    
    printf("system initialized, now listening for CANBUS messages\n");

    while (1) {
        int64_t current_time = esp_timer_get_time();

        if (can_driver_read_sensor(&current_sensor_data)) {  //read sensor (non blocking)
            current_sensor_data.water_level = read_water_level_sensor(); //reads water level
            printf("new message - temp: %.1f C, light: %u lux, hum: %u%%, moist: %u, water: %s\n", 
                current_sensor_data.temperature, 
                current_sensor_data.light_level,
                current_sensor_data.humidity,
                current_sensor_data.moisture,
                current_sensor_data.water_level ? "HIGH" : "LOW");

            if (current_time - last_adafruit_post >= 1000000ULL) {  //adafruit limiter
                publish_all_sensors(&current_sensor_data);
                last_adafruit_post = current_time;
            }
        }

        process_sensor_data(&current_sensor_data, &is_pump_active, &is_light_active);  //logic processing

        update_hardware_actuators(is_pump_active, is_light_active); //adjust outputs

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void process_sensor_data(SensorData *data, bool *pump_state, bool *light_state) { 
    static int64_t pump_start_time = 0;
    static int64_t cooldown_start_time = 0;
    static bool is_cooldown = false;

    if (data->raw_id == 0) return; // Ignore boot cycle null data
    
    int64_t current_time = esp_timer_get_time();

    if (data->light_level < 90) { //light
        *light_state = true;  //too dark, turn on
    } else if (data->light_level > 110) {
        *light_state = false; //bright enough, turn off
    }

    if (*pump_state == false && is_cooldown == false) { //pump
        if (data->moisture < 100) {
            *pump_state = true;
            pump_start_time = current_time;
        }
    }

    if (*pump_state == true) {
        if (current_time - pump_start_time >= 10000000ULL) { //run for 10 seconds
            *pump_state = false;
            is_cooldown = true;
            cooldown_start_time = current_time;
        }
    }

    if (is_cooldown == true) {
        if (current_time - cooldown_start_time >= 100000000ULL) { //1 min pump cooldown
            is_cooldown = false;
        }
    }
}

void update_hardware_actuators(bool pump_state, bool light_state) {  
    static bool last_pump_state = false;//trigger only when changing them
    static bool last_light_state = false;
    
    if (pump_state != last_pump_state) {
        gpio_set_level(PUMP_PIN, pump_state ? 1 : 0);
        printf(" ACTION: water pump turned %s\n", pump_state ? "ON" : "OFF");
        last_pump_state = pump_state;
    }

    if (light_state != last_light_state) {
        gpio_set_level(LIGHT_PIN, light_state ? 1 : 0);
        printf(" ACTION: led grow lights turned %s\n", light_state ? "ON" : "OFF");
        last_light_state = light_state;
    }
}

void hardware_init(void) { //GPIO initializations 

    gpio_reset_pin(PUMP_PIN); 
    gpio_set_direction(PUMP_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LIGHT_PIN); 
    gpio_set_direction(LIGHT_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(WATER_LEVEL_PIN); 
    gpio_set_direction(WATER_LEVEL_PIN, GPIO_MODE_INPUT);
}

void can_driver_init(void) {  
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL); 
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); 
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); 
    
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "Driver started");
}

bool can_driver_read_sensor(SensorData *out_data) {
    twai_message_t rx_msg;
    esp_err_t ret = twai_receive(&rx_msg, 0); 

    if (ret == ESP_OK) { 
        if (rx_msg.identifier == 0x101 && rx_msg.data_length_code >= 8) { 
            
            int16_t raw_temp = (rx_msg.data[0] << 8) | rx_msg.data[1];
            out_data->temperature = raw_temp / 10.0f;
            out_data->light_level = (rx_msg.data[2] << 8) | rx_msg.data[3];
            out_data->humidity = (rx_msg.data[4] << 8) | rx_msg.data[5];
            out_data->moisture = (rx_msg.data[6] << 8) | rx_msg.data[7];
            out_data->raw_id = rx_msg.identifier;

            return true;
        }
    }
    return false; 
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // Try to connect when Wi-Fi driver starts
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        printf(" WIFI UPDATE: wifi connection lost or failed. attempting to reconnect\n");
        esp_wifi_connect(); // Infinite retry loop in the background
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        printf(" WIFI UPDATE: wifi connected\n");
    }
}

void wifi_init(void) {  
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

 
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    printf("starting wifi driver\n");
    ESP_ERROR_CHECK(esp_wifi_start());
}

void publish_all_sensors(SensorData *data) { 
    char url[256];
    snprintf(url, sizeof(url), "https://io.adafruit.com/api/v2/%s/groups/%s/data", AIO_USERNAME, GROUP_KEY);
    char post_data[512];
    
    snprintf(post_data, sizeof(post_data), 
        "{\"feeds\": ["
            "{\"key\": \"%s\", \"value\": \"%.2f\"}, "
            "{\"key\": \"%s\", \"value\": \"%u\"}, "
            "{\"key\": \"%s\", \"value\": \"%u\"}, "
            "{\"key\": \"%s\", \"value\": \"%u\"}, "
            "{\"key\": \"%s\", \"value\": \"%d\"}"
        "]}", 
        FEED_TEMPERATURE, data->temperature,
        FEED_LIGHT, data->light_level,
        FEED_HUMIDITY, data->humidity,
        FEED_MOISTURE, data->moisture,
        FEED_WATER_LEVEL, data->water_level ? 1 : 0
    );

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach, 
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "X-AIO-Key", AIO_KEY);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        printf(" SUCCESSFULLY uploaded all data to adafruit\n");
    } else {
        printf(" FAILED to upload data to adafruit\n");
    }
    
    esp_http_client_cleanup(client);
}

bool read_water_level_sensor(void) {
    return gpio_get_level(WATER_LEVEL_PIN) == 1;  //returns 1 if water is detected, 0 if empty
}