idf_component_register(SRCS "MCU_code.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_client nvs_flash driver esp_timer esp_wifi esp_event esp_netif mbedtls)