/**
 * Nomes: Brenon Lenes Bettcher, Patrick Gonçalves
 * Projeto: Prática 06 - Multitarefa com Display LVGL (Sincronização Corrigida)
 * Descrição: Integração das práticas anteriores com exibição de Hora e Tensão.
 * Solução: Uso de variáveis volatile e correção do Deadlock no callback da UI.
 */

#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>      // Para usleep
#include <sys/lock.h>    // Para _lock_t do LVGL
#include <sys/param.h>   // Para MIN/MAX

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"   // Para o tick do LVGL

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"

// Includes do ADC
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Includes do Display e LVGL
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "lvgl.h"

// --- TAGs de Log ---
static const char *TAG1 = "PRATICA_01_LOG";
static const char *TAG2 = "PRATICA_02_BUTTON";
static const char *TAG3 = "PRATICA_03_TIMER";
static const char *TAG4 = "PRATICA_04_PWM";
static const char *TAG5 = "PRATICA_05_ADC";
static const char *TAG6 = "PRATICA_06_LVGL";

// --- Definições de Hardware ---
#define LED_AZUL_GPIO 2
#define PWM_LED_GPIO 26
#define PWM_OSC_GPIO 33
// ADC no GPIO 34 (Canal 3)

// --- Configuração Display / LVGL ---
#define I2C_BUS_PORT 0
#define EXAMPLE_PIN_NUM_SDA 19  // SEUS PINOS (ESTÁVEIS)
#define EXAMPLE_PIN_NUM_SCL 18  // SEUS PINOS (ESTÁVEIS)
#define EXAMPLE_PIN_NUM_RST -1
#define EXAMPLE_I2C_HW_ADDR 0x3C // Confirme endereço (0x3C ou 0x3D)

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (400 * 1000)
#define EXAMPLE_LCD_H_RES 128
#define EXAMPLE_LCD_V_RES 64
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

#define EXAMPLE_LVGL_TICK_PERIOD_MS 5
#define EXAMPLE_LVGL_TASK_STACK_SIZE (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY 2
#define EXAMPLE_LVGL_PALETTE_SIZE 8
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1000 / CONFIG_FREERTOS_HZ

// --- Estruturas e Variáveis Globais das Práticas Anteriores ---
#define DEBOUNCE_DELAY_MS 150
typedef struct {
    uint32_t gpio_num;
    TimerHandle_t timer_handle;
} button_config_t;

static button_config_t buttons[] = {
    { .gpio_num = 21, .timer_handle = NULL },
    { .gpio_num = 22, .timer_handle = NULL },
    { .gpio_num = 23, .timer_handle = NULL }
};
#define NUM_BUTTONS (sizeof(buttons) / sizeof(button_config_t))
static QueueHandle_t gpio_evt_queue = NULL;

typedef struct {
    uint64_t event_count;
    uint64_t alarm_value;
} timer_event_t;

typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} real_time_clock_t;
static QueueHandle_t gptimer_queue = NULL;

typedef struct {
    bool set_auto_mode;
    int16_t increment;
} PWM_elements_t;
static QueueHandle_t pwm_control_queue = NULL;
static SemaphoreHandle_t pwm_sync_semaphore = NULL;

typedef struct {
    int raw;
    int voltage;
} adc_data_t;
static QueueHandle_t adc_data_queue = NULL;
static SemaphoreHandle_t adc_sync_semaphore = NULL;

// --- Variáveis Globais Voláteis (Sincronização Simples) ---
// O uso de 'volatile' garante que a leitura na UI pegue o valor mais recente
// escrito pela task do Timer, sem precisar de mutex de dados.
volatile real_time_clock_t g_rtc = {0, 0, 0};
volatile adc_data_t g_adc_data = {0, 0};

// Buffer do Display e Lock da API LVGL
static uint8_t oled_buffer[EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8];
// Mantemos este lock APENAS para a estrutura do LVGL (exigido pelo exemplo), 
// mas não vamos usá-lo na lógica de dados.
static _lock_t lvgl_api_lock; 

// Labels da Interface
static lv_obj_t *label_time;
static lv_obj_t *label_voltage;


// ---------------------------------------------------------------------------
// --- LÓGICA DAS TAREFAS ANTERIORES (Mantida Integralmente) ---
// ---------------------------------------------------------------------------

static void button_timer_callback(TimerHandle_t xTimer) {
    uint32_t gpio_num = (uint32_t) pvTimerGetTimerID(xTimer);
    if (gpio_get_level(gpio_num) == 0) {
        xQueueSend(gpio_evt_queue, &gpio_num, portMAX_DELAY);
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (buttons[i].gpio_num == gpio_num) {
            if (buttons[i].timer_handle != NULL) {
                xTimerResetFromISR(buttons[i].timer_handle, &xHigherPriorityTaskWoken);
            }
            break;
        }
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void button_task(void* arg) {
    uint32_t io_num;
    while (true) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            PWM_elements_t pwm_cmd;
            bool command_valid = true;
            switch (io_num) {
                case 21: pwm_cmd.set_auto_mode = true; pwm_cmd.increment = 0; break;
                case 22: pwm_cmd.set_auto_mode = false; pwm_cmd.increment = -1; break;
                case 23: pwm_cmd.set_auto_mode = false; pwm_cmd.increment = 1; break;
                default: command_valid = false; break;
            }
            if (command_valid) {
                xQueueSend(pwm_control_queue, &pwm_cmd, portMAX_DELAY);
            }
        }
    }
}

static bool IRAM_ATTR timer_alarm_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    timer_event_t event_data = { .event_count = edata->count_value, .alarm_value = edata->alarm_value };
    xQueueSendFromISR(gptimer_queue, &event_data, &high_task_awoken);
    gptimer_alarm_config_t next_alarm_config = { .alarm_count = edata->alarm_value + 100000, .reload_count = 0, .flags.auto_reload_on_alarm = false };
    gptimer_set_alarm_action(timer, &next_alarm_config);
    return high_task_awoken == pdTRUE;
}

// Tarefa do Timer: Atualiza o Relógio e as Variáveis Globais
void gptimer_task(void *pvParameters) {
    ESP_LOGI(TAG3, "Iniciando GPTimer.");
    real_time_clock_t rtc = {0, 0, 0};
    int interrupt_counter = 0;
    adc_data_t last_adc_data = {0, 0};

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = { .clk_src = GPTIMER_CLK_SRC_DEFAULT, .direction = GPTIMER_COUNT_UP, .resolution_hz = 1000000 };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    gptimer_alarm_config_t alarm_config = { .alarm_count = 100000, .reload_count = 0, .flags.auto_reload_on_alarm = false };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    gptimer_event_callbacks_t cbs = { .on_alarm = timer_alarm_isr_callback };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    timer_event_t evt;

    while (1) {
        if (xQueueReceive(gptimer_queue, &evt, portMAX_DELAY)) {
            xSemaphoreGive(pwm_sync_semaphore);
            xSemaphoreGive(adc_sync_semaphore);

            if (xQueueReceive(adc_data_queue, &last_adc_data, 0) == pdPASS) {
                // Nova leitura ADC recebida
            }

            interrupt_counter++;
            if (interrupt_counter >= 10) {
                interrupt_counter = 0;
                rtc.second++;
                if (rtc.second >= 60) {
                    rtc.second = 0; rtc.minute++;
                    if (rtc.minute >= 60) {
                        rtc.minute = 0; rtc.hour++;
                        if (rtc.hour >= 24) rtc.hour = 0;
                    }
                }
                ESP_LOGI(TAG3, "Hora: %02d:%02d:%02d | ADC: %d mV", rtc.hour, rtc.minute, rtc.second, last_adc_data.voltage);
                
                // ATUALIZAÇÃO SEGURA (SEM MUTEX OS):
                // Apenas escrevemos nas variáveis volatile.
                // A interface gráfica (LVGL) vai ler isso no tempo dela.
                g_rtc = rtc;
                g_adc_data = last_adc_data;
            }
        }
    }
}

void pwm_task(void *pvParameters) {
    ESP_LOGI(TAG4, "Iniciando PWM.");
    ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0, .duty_resolution = LEDC_TIMER_13_BIT, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel_0 = { .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE, .gpio_num = PWM_LED_GPIO, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_0));
    ledc_channel_config_t ledc_channel_1 = { .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE, .gpio_num = PWM_OSC_GPIO, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));

    bool is_auto_mode = true;
    uint32_t duty_cycle = 0;
    int sawtooth_step = 82;
    PWM_elements_t pwm_cmd;

    while (1) {
        if (xQueueReceive(pwm_control_queue, &pwm_cmd, 0) == pdPASS) {
            if(pwm_cmd.increment == 0) is_auto_mode = pwm_cmd.set_auto_mode;
            else is_auto_mode = false;
            
            if (!is_auto_mode && pwm_cmd.increment != 0) {
                if (pwm_cmd.increment > 0) { duty_cycle += 410; if (duty_cycle > 8191) duty_cycle = 8191; }
                else if (pwm_cmd.increment < 0) { if (duty_cycle < 410) duty_cycle = 0; else duty_cycle -= 410; }
            }
        }

        if (is_auto_mode) {
            if (xSemaphoreTake(pwm_sync_semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                duty_cycle += sawtooth_step;
                if (duty_cycle > 8191) duty_cycle = 0;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle); ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_cycle); ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}

void adc_task(void *pvParameters) {
    ESP_LOGI(TAG5, "Iniciando ADC.");
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_11 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_line_fitting_config_t cali_config = { .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_11, .bitwidth = ADC_BITWIDTH_DEFAULT };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

    adc_data_t data_to_send;
    while (1) {
        if (xSemaphoreTake(adc_sync_semaphore, portMAX_DELAY) == pdTRUE) {
            int raw_value, voltage_value;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &raw_value));
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw_value, &voltage_value));
            data_to_send.raw = raw_value;
            data_to_send.voltage = voltage_value;
            xQueueSend(adc_data_queue, &data_to_send, 0);
        }
    }
}

// ---------------------------------------------------------------------------
// --- LVGL / DISPLAY (Correção de Deadlock) ---
// ---------------------------------------------------------------------------

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io_panel, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    px_map += EXAMPLE_LVGL_PALETTE_SIZE;
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
}

static void increase_lvgl_tick(void *arg) {
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void lvgl_port_task(void *arg) {
    ESP_LOGI(TAG6, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (1) {
        // [LOCK AQUI] Protege o handler do LVGL
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler(); // Executa o LVGL (e nosso callback)
        _lock_release(&lvgl_api_lock);
        
        time_till_next_ms = MAX(time_till_next_ms, EXAMPLE_LVGL_TASK_MIN_DELAY_MS);
        time_till_next_ms = MIN(time_till_next_ms, EXAMPLE_LVGL_TASK_MAX_DELAY_MS);
        usleep(1000 * time_till_next_ms);
    }
}

// !!! CORREÇÃO AQUI !!!
static void update_ui_callback(lv_timer_t *timer) {
    // 1. Lemos os valores voláteis (mais recentes)
    // Sem mutex, apenas leitura rápida.
    real_time_clock_t current_rtc = g_rtc;
    adc_data_t current_adc = g_adc_data;

    // 2. Atualizamos o texto.
    // NÃO COLOCAMOS _lock_acquire AQUI, POIS JÁ ESTAMOS DENTRO DO LOCK DA TAREFA LVGL.
    // (Isso evita o travamento que você viu na tela)
    lv_label_set_text_fmt(label_time, "Hora: %02d:%02d:%02d", current_rtc.hour, current_rtc.minute, current_rtc.second);
    lv_label_set_text_fmt(label_voltage, "ADC: %d mV", current_adc.voltage);
}

static void create_demo_ui(lv_display_t *disp) {
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    label_time = lv_label_create(scr);
    lv_label_set_text(label_time, "Hora: --:--:--");
    lv_obj_align(label_time, LV_ALIGN_TOP_MID, 0, 10);

    label_voltage = lv_label_create(scr);
    lv_label_set_text(label_voltage, "ADC: ---- mV");
    lv_obj_align(label_voltage, LV_ALIGN_TOP_MID, 0, 30);
}

void app_main(void) {
    ESP_LOGI(TAG1, "Iniciando Pratica 06 (Integrada)...");
    
    // Configura hardware
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    gpio_install_isr_service(0);
    uint64_t pin_bit_mask = 0;
    for (int i = 0; i < NUM_BUTTONS; i++) {
        pin_bit_mask |= (1ULL << buttons[i].gpio_num);
        buttons[i].timer_handle = xTimerCreate("btn", pdMS_TO_TICKS(DEBOUNCE_DELAY_MS), pdFALSE, (void*) buttons[i].gpio_num, button_timer_callback);
        gpio_isr_handler_add(buttons[i].gpio_num, gpio_isr_handler, (void*) buttons[i].gpio_num);
    }
    gpio_config_t io_conf = { .intr_type = GPIO_INTR_NEGEDGE, .pin_bit_mask = pin_bit_mask, .mode = GPIO_MODE_INPUT, .pull_up_en = 1 };
    gpio_config(&io_conf);
    
    gpio_reset_pin(LED_AZUL_GPIO); gpio_set_direction(LED_AZUL_GPIO, GPIO_MODE_OUTPUT);

    gptimer_queue = xQueueCreate(5, sizeof(timer_event_t));
    xTaskCreate(gptimer_task, "gptimer_task", 2048, NULL, 5, NULL);

    pwm_sync_semaphore = xSemaphoreCreateBinary();
    pwm_control_queue = xQueueCreate(5, sizeof(PWM_elements_t));
    xTaskCreate(pwm_task, "pwm_task", 2048, NULL, 5, NULL);

    adc_sync_semaphore = xSemaphoreCreateBinary();
    adc_data_queue = xQueueCreate(5, sizeof(adc_data_t));
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);

    // --- CONFIGURAÇÃO DISPLAY I2C / LVGL ---
    ESP_LOGI(TAG6, "Configurando Display...");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA, // PINO 19
        .scl_io_num = EXAMPLE_PIN_NUM_SCL, // PINO 18
        .flags.enable_internal_pullup = true,
        .glitch_ignore_cnt = 7,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS,
        .dc_bit_offset = 6,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = { .bits_per_pixel = 1, .reset_gpio_num = EXAMPLE_PIN_NUM_RST };
    esp_lcd_panel_ssd1306_config_t ssd1306_config = { .height = EXAMPLE_LCD_V_RES };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    lv_init();
    lv_display_t *display = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    lv_display_set_user_data(display, panel_handle);
    
    size_t draw_buffer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8 + EXAMPLE_LVGL_PALETTE_SIZE;
    void *buf = heap_caps_calloc(1, draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
    lv_display_set_buffers(display, buf, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    const esp_lcd_panel_io_callbacks_t cbs_io = { .on_color_trans_done = notify_lvgl_flush_ready };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs_io, display);

    const esp_timer_create_args_t lvgl_tick_timer_args = { .callback = &increase_lvgl_tick, .name = "lvgl_tick" };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    xTaskCreate(lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    _lock_acquire(&lvgl_api_lock);
    create_demo_ui(display);
    lv_timer_create(update_ui_callback, 500, NULL);
    _lock_release(&lvgl_api_lock);
    
    ESP_LOGI(TAG6, "Sistema Rodando.");
}