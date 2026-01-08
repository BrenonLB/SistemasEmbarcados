/**
 * Projeto Final: Smart Greenhouse (Estufa de Orquídeas)
 * Autores: Brenon Lenes Bettcher, Patrick Gonçalves
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "mqtt_client.h"
#include "esp_timer.h"
#include "esp_netif_sntp.h" 

// Hardware Drivers
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

//Display & LVGL
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "lvgl.h"

//Configurações e Defines
#define TAG "GREENHOUSE"

// MQTT
#define MQTT_BROKER_URI "mqtt://g2device:g2device@node02.myqtthub.com:1883"
#define MQTT_TOPIC_CONFIG "g2device/config"
#define MQTT_TOPIC_COLOR  "g2device/color"
#define MQTT_TOPIC_DATA   "g2device/monitor"

//PINAGEM CORRIGIDA PARA LED RGB
#define PWM_PIN_RED       17  // Modo SOLAR (Vermelho)
#define PWM_PIN_BLUE      26  // Modo UV (Azul)
#define BUTTON_PIN        23  // Botão de Troca
#define I2C_SDA           19
#define I2C_SCL           18

// Display Resolução
#define LCD_H_RES         128
#define LCD_V_RES         64
#define LVGL_PALETTE_SIZE 8 

// Buffer estático
static uint8_t oled_buffer[LCD_H_RES * LCD_V_RES / 8];

// Estado Global
typedef enum {
    MODE_SOLAR_RED = 0, // Vermelho
    MODE_UV_BLUE   = 1  // Azul
} greenhouse_mode_t;

typedef struct {
    int target_temp;
    int target_hum;
    int current_temp;
    int current_hum;
    greenhouse_mode_t current_mode;
    int pwm_intensity;
} greenhouse_state_t;

static greenhouse_state_t g_state = {
    .target_temp = 25, 
    .target_hum = 60,
    .current_temp = 0,
    .current_hum = 55,
    .current_mode = MODE_SOLAR_RED,
    .pwm_intensity = 0
};

//Mutexes e Filas
static SemaphoreHandle_t xStateMutex = NULL;
static SemaphoreHandle_t xGuiSemaphore = NULL;
static QueueHandle_t gpio_evt_queue = NULL;

esp_mqtt_client_handle_t mqtt_client = NULL;

static lv_obj_t *lbl_clock;
static lv_obj_t *lbl_temp;
static lv_obj_t *lbl_target;
static lv_obj_t *lbl_mode;

//SNTP (Assíncrono)
void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "SNTP: Relógio sincronizado!");
}

void initialize_sntp(void) {
    setenv("TZ", "BRT3", 1);
    tzset();
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    config.sync_cb = time_sync_notification_cb; 
    esp_netif_sntp_init(&config);
}

void get_time_str(char *buffer, size_t size) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(buffer, size, "%H:%M:%S", &timeinfo);
}

//MQTT
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    if (event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT Conectado.");
        esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_CONFIG, 0);
        esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_COLOR, 0);
    } 
    else if (event_id == MQTT_EVENT_DATA) {
        char payload[32];
        snprintf(payload, event->data_len + 1, "%.*s", event->data_len, event->data);
        
        if (xSemaphoreTake(xStateMutex, portMAX_DELAY)) {
            if (strncmp(event->topic, MQTT_TOPIC_COLOR, event->topic_len) == 0) {
                //Se receber via MQTT, atualiza também
                if (strstr(payload, "BLUE") || strstr(payload, "UV")) g_state.current_mode = MODE_UV_BLUE;
                else g_state.current_mode = MODE_SOLAR_RED;
            } 
            else if (strncmp(event->topic, MQTT_TOPIC_CONFIG, event->topic_len) == 0) {
                int t, h;
                if (sscanf(payload, "T:%d H:%d", &t, &h) == 2) {
                    g_state.target_temp = t;
                    g_state.target_hum = h;
                }
            }
            xSemaphoreGive(xStateMutex);
        }
    }
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.client_id = "g2device",
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

//BOTÃO (Alterna Modos)
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void button_task(void* arg) {
    uint32_t io_num;
    while(1) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(50)); //Debounce
            
            if(gpio_get_level(io_num) == 0) {
                if (xSemaphoreTake(xStateMutex, portMAX_DELAY)) {
                    // Alterna Lógica: Se era Solar(Red) vira UV(Blue) e vice-versa
                    if (g_state.current_mode == MODE_SOLAR_RED) {
                        g_state.current_mode = MODE_UV_BLUE;
                        ESP_LOGI(TAG, "Botão: Mudou para UV (Azul)");
                    } else {
                        g_state.current_mode = MODE_SOLAR_RED;
                        ESP_LOGI(TAG, "Botão: Mudou para SOLAR (Vermelho)");
                    }
                    xSemaphoreGive(xStateMutex);
                    
                    //Publica novo estado
                    const char *msg = (g_state.current_mode == MODE_SOLAR_RED) ? "SOLAR_RED" : "UV_BLUE";
                    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_COLOR, msg, 0, 0, 0);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(250)); //Evita repetição rápida
        }
    }
}

//ADC
void adc_task(void *pvParameters) {
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_11 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));

    while (1) {
        int raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &raw));
        int temp_celsius = raw / 60; 

        if (xSemaphoreTake(xStateMutex, portMAX_DELAY)) {
            g_state.current_temp = temp_celsius;
            g_state.current_hum = 80 - (temp_celsius / 2); 
            xSemaphoreGive(xStateMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Controle PWM (Lógica Invertida Aplicada) 
void control_task(void *pvParameters) {
    ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0, .duty_resolution = LEDC_TIMER_13_BIT, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t c_red = { .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE, .gpio_num = PWM_PIN_RED, .duty = 0, .hpoint = 0 };
    ledc_channel_config(&c_red);

    ledc_channel_config_t c_blue = { .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE, .gpio_num = PWM_PIN_BLUE, .duty = 0, .hpoint = 0 };
    ledc_channel_config(&c_blue);

    while (1) {
        if (xSemaphoreTake(xStateMutex, portMAX_DELAY)) {
            // Lógica de Aquecimento (Simples Proporcional)
            int error = g_state.target_temp - g_state.current_temp;
            int new_duty = 0;

            if (error > 0) {
                new_duty = error * 1000; 
                if (new_duty > 8191) new_duty = 8191;
                if (new_duty < 500) new_duty = 500; 
            } else {
                new_duty = 0; //Se temperatura ok, desliga
            }
            g_state.pwm_intensity = new_duty;
            
            //APLICAÇÃO DA COR
            if (g_state.current_mode == MODE_SOLAR_RED) {
                //Modo SOLAR: Liga Vermelho, Desliga Azul
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, new_duty); // Red ON
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);        // Blue OFF
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            } else {
                //Modo UV: Liga Azul, Desliga Vermelho
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, new_duty); // Blue ON
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);        // Red OFF
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            }
            xSemaphoreGive(xStateMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

//Display
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    px_map += LVGL_PALETTE_SIZE;
    uint16_t hor_res = lv_display_get_physical_horizontal_resolution(disp);
    int x1 = area->x1, x2 = area->x2, y1 = area->y1, y2 = area->y2;

    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            bool chroma_color = (px_map[(hor_res >> 3) * y + (x >> 3)] & 1 << (7 - x % 8));
            uint8_t *buf = oled_buffer + hor_res * (y >> 3) + (x);
            if (chroma_color) (*buf) &= ~(1 << (y % 8));
            else (*buf) |= (1 << (y % 8));
        }
    }
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, oled_buffer);
    lv_display_flush_ready(disp);
}

static void increase_lvgl_tick(void *arg) { lv_tick_inc(5); }

static void update_ui_callback(lv_timer_t *timer) {
    if (xSemaphoreTake(xStateMutex, 100)) {
        char time_buf[16];
        get_time_str(time_buf, sizeof(time_buf));
        
        lv_label_set_text_fmt(lbl_clock, "Hora: %s", time_buf);
        lv_label_set_text_fmt(lbl_temp, "Amb: %d C | %d%%", g_state.current_temp, g_state.current_hum);
        lv_label_set_text_fmt(lbl_target, "Meta: %d C", g_state.target_temp);
        
        if(g_state.current_mode == MODE_SOLAR_RED) 
            lv_label_set_text(lbl_mode, "Modo: SOLAR (Red)");
        else 
            lv_label_set_text(lbl_mode, "Modo: UV (Blue)");
            
        xSemaphoreGive(xStateMutex);
        
        static int count = 0;
        if(++count > 10) { 
            count = 0;
            char msg[64];
            sprintf(msg, "Temp:%d Mode:%d Duty:%d", g_state.current_temp, g_state.current_mode, g_state.pwm_intensity);
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DATA, msg, 0, 0, 0);
        }
    }
}

void create_gui(lv_display_t *disp) {
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lbl_clock = lv_label_create(scr);
    lv_obj_align(lbl_clock, LV_ALIGN_TOP_MID, 0, 0);
    lbl_temp = lv_label_create(scr);
    lv_obj_align(lbl_temp, LV_ALIGN_TOP_MID, 0, 16);
    lbl_target = lv_label_create(scr);
    lv_obj_align(lbl_target, LV_ALIGN_TOP_MID, 0, 32);
    lbl_mode = lv_label_create(scr);
    lv_obj_align(lbl_mode, LV_ALIGN_TOP_MID, 0, 48);
}

static void lvgl_port_task(void *arg) {
    while (1) {
        if (xSemaphoreTake(xGuiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            lv_timer_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    xStateMutex = xSemaphoreCreateMutex();
    xGuiSemaphore = xSemaphoreCreateRecursiveMutex(); 
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Botão
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE; 
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1; 
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void*) BUTTON_PIN);
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    ESP_ERROR_CHECK(example_connect());
    initialize_sntp();
    mqtt_app_start();

    xTaskCreate(adc_task, "adc_task", 4096, NULL, 3, NULL);
    xTaskCreate(control_task, "ctrl_task", 4096, NULL, 3, NULL);

    // Display
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .flags.enable_internal_pullup = true,
        .glitch_ignore_cnt = 7,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = 0x3C,
        .scl_speed_hz = 400 * 1000,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 6,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));
    
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = { .bits_per_pixel = 1, .reset_gpio_num = -1 };
    esp_lcd_panel_ssd1306_config_t ssd_cfg = { .height = LCD_V_RES };
    panel_config.vendor_config = &ssd_cfg;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    // LVGL
    lv_init();
    lv_display_t *display = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_user_data(display, panel_handle);

    size_t draw_buffer_sz = LCD_H_RES * LCD_V_RES / 8 + LVGL_PALETTE_SIZE;
    void *buf = heap_caps_calloc(1, draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    
    if (buf == NULL) { ESP_LOGE(TAG, "Erro RAM Video"); abort(); }

    lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
    lv_display_set_buffers(display, buf, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(display, lvgl_flush_cb); 
    
    const esp_timer_create_args_t lvgl_tick_timer_args = { .callback = &increase_lvgl_tick, .name = "lvgl_tick" };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 5000));

    if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY) == pdTRUE) {
        create_gui(display);
        lv_timer_create(update_ui_callback, 500, NULL);
        xSemaphoreGive(xGuiSemaphore);
    }

    xTaskCreate(lvgl_port_task, "LVGL", 4096, NULL, 2, NULL);
    
    ESP_LOGI(TAG, "Estufa OK.");
    vTaskDelete(NULL);
}