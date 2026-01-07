/**
 * Projeto Final: Smart Greenhouse (Estufa de Orquídeas)
 * Autores: Brenon Lenes Bettcher, Patrick Gonçalves
 * * Funcionalidades:
 * - Leitura de Temperatura via ADC (Simulado por Potenciômetro).
 * - Controle PID Simples (PWM) para Aquecimento/Luminosidade.
 * - Troca de Espectro de Luz (Amarelo/Azul) via MQTT.
 * - Setpoint de Temperatura e Umidade via MQTT.
 * - Relógio em Tempo Real via Internet (SNTP).
 * - Display OLED com Interface LVGL.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
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
#include "esp_sntp.h"
#include "esp_timer.h"

// Hardware Drivers
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Display & LVGL
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "lvgl.h"

// --- Configurações e Defines ---
#define TAG "GREENHOUSE"

// MQTT
#define MQTT_BROKER_URI "mqtt://g2device:g2device@node02.myqtthub.com:1883"
#define MQTT_TOPIC_CONFIG "g2device/config"   // Enviar "T:25 H:60"
#define MQTT_TOPIC_COLOR  "g2device/color"    // Enviar "YELLOW" ou "BLUE"
#define MQTT_TOPIC_DATA   "g2device/monitor"  // Publica dados

// Pinos
#define PWM_PIN_YELLOW    26 // Luz Solar / Aquecimento
#define PWM_PIN_BLUE      33 // Luz UV
#define I2C_SDA           19
#define I2C_SCL           18

// Display
#define LCD_H_RES         128
#define LCD_V_RES         64

// Estrutura Global do Estado da Estufa
typedef enum {
    LIGHT_YELLOW = 0, // Simula Sol
    LIGHT_BLUE = 1    // Simula UV
} light_color_t;

typedef struct {
    int target_temp;    // Temperatura Desejada (recebida MQTT)
    int target_hum;     // Umidade Desejada (recebida MQTT)
    int current_temp;   // Temperatura Lida (ADC)
    int current_hum;    // Umidade Lida (Simulada para este exemplo)
    light_color_t color_mode;
    int pwm_intensity;  // 0 a 8191
} greenhouse_state_t;

// Variáveis Globais Protegidas
static greenhouse_state_t g_state = {
    .target_temp = 25, // Default Orquídea
    .target_hum = 60,
    .current_temp = 0,
    .current_hum = 55,
    .color_mode = LIGHT_YELLOW,
    .pwm_intensity = 0
};

static SemaphoreHandle_t xStateMutex = NULL;
esp_mqtt_client_handle_t mqtt_client = NULL;

// Variáveis LVGL
static _lock_t lvgl_api_lock;
static lv_obj_t *lbl_clock;
static lv_obj_t *lbl_temp;
static lv_obj_t *lbl_target;
static lv_obj_t *lbl_mode;

// --- Implementação SNTP (Tempo Real) ---
void initialize_sntp(void) {
    ESP_LOGI(TAG, "Inicializando SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    
    // Configura Fuso Horário (Brasília)
    setenv("TZ", "BRT3", 1);
    tzset();
}

void get_time_str(char *buffer, size_t size) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(buffer, size, "%H:%M:%S", &timeinfo);
}

// --- Callbacks MQTT ---
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
                // Comando de Cor
                if (strstr(payload, "BLUE")) g_state.color_mode = LIGHT_BLUE;
                else g_state.color_mode = LIGHT_YELLOW;
                ESP_LOGI(TAG, "Mudança de Cor: %s", payload);
            } 
            else if (strncmp(event->topic, MQTT_TOPIC_CONFIG, event->topic_len) == 0) {
                // Configuração "T:xx H:xx"
                int t, h;
                if (sscanf(payload, "T:%d H:%d", &t, &h) == 2) {
                    g_state.target_temp = t;
                    g_state.target_hum = h;
                    ESP_LOGI(TAG, "Novos Alvos -> T: %d, H: %d", t, h);
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

// --- Tarefa ADC (Leitura Sensores) ---
void adc_task(void *pvParameters) {
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
    // Canal 3 = Temperatura (Potenciômetro)
    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_11 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));

    // Calibração
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_line_fitting_config_t cali_config = { .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_11, .bitwidth = ADC_BITWIDTH_DEFAULT };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

    while (1) {
        int voltage;
        int raw;
        // Leitura da Temperatura (Mapeada de 0-3000mV para 0-50 Graus C para teste)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw, &voltage));
        
        int temp_celsius = voltage / 60; // Conversão rudimentar p/ teste (0 a ~50C)

        if (xSemaphoreTake(xStateMutex, portMAX_DELAY)) {
            g_state.current_temp = temp_celsius;
            
            // Simulação de variação de umidade baseada na temperatura (física básica)
            // Se esquenta muito, umidade cai um pouco
            g_state.current_hum = 80 - (temp_celsius / 2); 
            xSemaphoreGive(xStateMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --- Tarefa de Controle (PWM + Lógica Estufa) ---
void control_task(void *pvParameters) {
    // Configura PWM
    ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0, .duty_resolution = LEDC_TIMER_13_BIT, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t c_yellow = { .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE, .gpio_num = PWM_PIN_YELLOW, .duty = 0, .hpoint = 0 };
    ledc_channel_config(&c_yellow);

    ledc_channel_config_t c_blue = { .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE, .gpio_num = PWM_PIN_BLUE, .duty = 0, .hpoint = 0 };
    ledc_channel_config(&c_blue);

    while (1) {
        if (xSemaphoreTake(xStateMutex, portMAX_DELAY)) {
            int error = g_state.target_temp - g_state.current_temp;
            int new_duty = 0;

            // Lógica Proporcional: Se Temp < Alvo, Liga Aquecedor (Luz)
            // "Quanto maior o brilho, maior a temperatura"
            if (error > 0) {
                // Precisa aquecer. Erro grande = Duty Grande.
                new_duty = error * 1000; 
                if (new_duty > 8191) new_duty = 8191;
                if (new_duty < 500) new_duty = 500; // Mínimo para ver aceso
            } else {
                // Já está quente, desliga ou mantém mínimo
                new_duty = 0;
            }
            g_state.pwm_intensity = new_duty;
            
            // Aplica ao canal correto
            if (g_state.color_mode == LIGHT_YELLOW) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, new_duty);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0); // Desliga Azul
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            } else {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, new_duty);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // Desliga Amarelo
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            }
            
            xSemaphoreGive(xStateMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Atualização rápida do controle (10Hz)
    }
}

// --- Interface Gráfica LVGL ---
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int x1 = area->x1, x2 = area->x2, y1 = area->y1, y2 = area->y2;
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, px_map);
    lv_display_flush_ready(disp);
}

static void increase_lvgl_tick(void *arg) { lv_tick_inc(5); }

static void update_ui_callback(lv_timer_t *timer) {
    if (xSemaphoreTake(xStateMutex, 100)) {
        char time_buf[16];
        get_time_str(time_buf, sizeof(time_buf));
        
        lv_label_set_text_fmt(lbl_clock, "Hora: %s", time_buf);
        lv_label_set_text_fmt(lbl_temp, "Ambiente: %d C | %d%%", g_state.current_temp, g_state.current_hum);
        lv_label_set_text_fmt(lbl_target, "Meta: %d C (Duty:%d)", g_state.target_temp, g_state.pwm_intensity/82);
        
        if(g_state.color_mode == LIGHT_YELLOW) 
            lv_label_set_text(lbl_mode, "Luz: SOLAR (Amarelo)");
        else 
            lv_label_set_text(lbl_mode, "Luz: UV (Azul)");
            
        xSemaphoreGive(xStateMutex);
        
        // Publicar Telemetria a cada atualização de UI (aprox 500ms é muito rápido, melhor filtrar)
        static int count = 0;
        if(++count > 10) { // A cada 5 seg
            count = 0;
            char msg[64];
            sprintf(msg, "Temp:%d Hum:%d Duty:%d", g_state.current_temp, g_state.current_hum, g_state.pwm_intensity);
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DATA, msg, 0, 0, 0);
        }
    }
}

void create_gui(lv_display_t *disp) {
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    
    lbl_clock = lv_label_create(scr);
    lv_obj_align(lbl_clock, LV_ALIGN_TOP_MID, 0, 0);
    
    lbl_temp = lv_label_create(scr);
    lv_obj_align(lbl_temp, LV_ALIGN_TOP_MID, 0, 15);
    
    lbl_target = lv_label_create(scr);
    lv_obj_align(lbl_target, LV_ALIGN_TOP_MID, 0, 30);

    lbl_mode = lv_label_create(scr);
    lv_obj_align(lbl_mode, LV_ALIGN_TOP_MID, 0, 45);
}

static void lvgl_port_task(void *arg) {
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    // 1. Inicializações de Sistema
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    xStateMutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(example_connect());
    
    //3.Inicializa SNTP e MQTT
    initialize_sntp();
    mqtt_app_start();

    

    // 4. Tasks de Controle
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);
    xTaskCreate(control_task, "ctrl_task", 4096, NULL, 5, NULL);

    // 5. Configuração do Display (SSD1306/SH1107 via I2C)
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

    // 6. LVGL Init
    lv_init();
    lv_display_t *display = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_user_data(display, panel_handle);
    void *buf = heap_caps_malloc(LCD_H_RES * LCD_V_RES / 8, MALLOC_CAP_DMA);
    lv_display_set_buffers(display, buf, NULL, LCD_H_RES * LCD_V_RES / 8, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(display, lvgl_flush_cb);
    
    // Timer LVGL
    const esp_timer_create_args_t lvgl_tick_timer_args = { .callback = &increase_lvgl_tick, .name = "lvgl_tick" };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 5000));

    // GUI
    _lock_acquire(&lvgl_api_lock);
    create_gui(display);
    lv_timer_create(update_ui_callback, 500, NULL); // Atualiza tela a cada 500ms
    _lock_release(&lvgl_api_lock);

    xTaskCreate(lvgl_port_task, "LVGL", 4096, NULL, 2, NULL);
    
    ESP_LOGI(TAG, "Estufa Iniciada com Sucesso.");
}