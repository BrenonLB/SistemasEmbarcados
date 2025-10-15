/**
 * Nomes: Brenon Lenes Bettcher, Patrick Gonçalves
 * Projeto: Prática 03 - GPTimer e Tarefas
 * Descrição:relógio de tempo real usando GPTimer.
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h" 
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/gptimer.h" 

static const char *TAG1 = "PRATICA_01_LOG";
static const char *TAG2 = "PRATICA_02_BUTTON";
static const char *TAG3 = "PRATICA_03_TIMER"; 

#define LED_AZUL_GPIO   2
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

/*Estruturas e Configurações da Tarefa do GPTimer (Relógio)*/
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

/*Estrutura da Tarefa dos Botões*/
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
    static bool led_state = false;
    while (true) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG2, "Botao no GPIO %" PRIu32 " acionado e validado!", io_num);
            switch (io_num) {
                case 21: // BOTAO_0_GPIO
                    ESP_LOGI(TAG2, "Alternando o estado do LED azul.");
                    led_state = !led_state;
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;
                case 22: // BOTAO_1_GPIO
                    ESP_LOGI(TAG2, "Ligando o LED azul.");
                    led_state = true;
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;
                case 23: // BOTAO_2_GPIO
                    ESP_LOGI(TAG2, "Desligando o LED azul.");
                    led_state = false;
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;
            }
        }
    }
}

static bool IRAM_ATTR timer_alarm_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *F_N)
{
    BaseType_t high_task_awoken = pdFALSE;
    
    // Prepara a estrutura de dados para enviar para a fila
    timer_event_t event_data = {
        .event_count = edata->count_value,
        .alarm_value = edata->alarm_value
    };

    xQueueSendFromISR(gptimer_queue, &event_data, &high_task_awoken);
    
    gptimer_alarm_config_t next_alarm_config = {
        .alarm_count = edata->alarm_value + 100000, // 100ms * 1MHz = 100.000 ticks
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(timer, &next_alarm_config);

    return high_task_awoken == pdTRUE;
}

void gptimer_task(void *pvParameters)
{
    ESP_LOGI(TAG3, "Iniciando a tarefa do GPTimer e configurando o relógio.");
    
    // Inicializa o relógio
    real_time_clock_t rtc = {0, 0, 0};
    int interrupt_counter = 0;

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, 
        .direction = GPTIMER_COUNT_UP,      
        .resolution_hz = 1 * 1000 * 1000,   
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    // Configuração do Alarme
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 100000, // Alarme inicial em 100ms (100.000 * 1µs)
        .reload_count = 0, 
        .flags.auto_reload_on_alarm = false,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_alarm_isr_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    timer_event_t evt;
    
    while (1) {
        // Aguarda por um evento da fila, enviado pela ISR do timer
        if (xQueueReceive(gptimer_queue, &evt, portMAX_DELAY)) {
            interrupt_counter++;

            // Como a interrupção ocorre a cada 100ms, 10 interrupções = 1 segundo
            if (interrupt_counter >= 10) {
                interrupt_counter = 0;

                rtc.second++;
                if (rtc.second >= 60) {
                    rtc.second = 0;
                    rtc.minute++;
                    if (rtc.minute >= 60) {
                        rtc.minute = 0;
                        rtc.hour++;
                        if (rtc.hour >= 24) {
                            rtc.hour = 0;
                        }
                    }
                }
                ESP_LOGI(TAG3, "Hora: %02d:%02d:%02d | Contagem Atual: %llu | Valor do Alarme: %llu",
                         rtc.hour, rtc.minute, rtc.second,
                         evt.event_count, evt.alarm_value);
            }
        }
    }
}

void app_main(void)
{
    //Bloco de informações do chip
    ESP_LOGI(TAG1, "Iniciando a aplicacao para obter informacoes do esp 32.");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG1, "Este e um %s com %d nucleo(s) de CPU.", CONFIG_IDF_TARGET, chip_info.cores);
    char features[100];
    snprintf(features, sizeof(features), "%s%s%s%s",
             (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
             (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");
    ESP_LOGI(TAG1, "Features: %s", features);
    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG1, "Revisao do esp: v%d.%d", major_rev, minor_rev);
    uint32_t flash_size;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG1, "Tamanho da flash: %" PRIu32 "MB (%s)", 
                 flash_size / (uint32_t)(1024 * 1024),
                 (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embarcada" : "externa");
    } else {
        ESP_LOGE(TAG1, "Falha ao obter o tamanho da memoria flash.");
    }
    ESP_LOGI(TAG1, "Minimo de heap livre: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG1, "Versao do ESP-IDF: %s", esp_get_idf_version());

    //Inicialização da Tarefa de Botões
    ESP_LOGI(TAG2, "Configurando a tarefa de botoes...");
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    gpio_install_isr_service(0);
    
    uint64_t pin_bit_mask = 0;
    for (int i = 0; i < NUM_BUTTONS; i++) {
        pin_bit_mask |= (1ULL << buttons[i].gpio_num);
        char timer_name[20];
        snprintf(timer_name, sizeof(timer_name), "timer_btn_%ld", buttons[i].gpio_num);
        buttons[i].timer_handle = xTimerCreate(timer_name, pdMS_TO_TICKS(DEBOUNCE_DELAY_MS), pdFALSE, (void*) buttons[i].gpio_num, button_timer_callback);
        gpio_isr_handler_add(buttons[i].gpio_num, gpio_isr_handler, (void*) buttons[i].gpio_num);
    }
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = pin_bit_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    
    gpio_reset_pin(LED_AZUL_GPIO);
    gpio_set_direction(LED_AZUL_GPIO, GPIO_MODE_OUTPUT);

    //Inicialização da Nova Tarefa do Timer
    ESP_LOGI(TAG3, "Configurando a tarefa do relogio...");
    gptimer_queue = xQueueCreate(5, sizeof(timer_event_t));
    xTaskCreate(gptimer_task, "gptimer_task", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG1, "Configuracao completa. Todas as tarefas estao em execucao.");
}