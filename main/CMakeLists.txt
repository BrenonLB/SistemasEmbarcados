idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES 
                      protocol_examples_common 
                      nvs_flash 
                      esp_wifi 
                      esp_event 
                      driver 
                      esp_adc 
                      esp_lcd 
                      lvgl 
                      mqtt
                      spi_flash
                       lwip)