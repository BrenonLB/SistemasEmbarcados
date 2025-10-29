/**
 * Nomes: Brenon Lenes Bettcher, Patrick Gonçalves
 * Projeto: Prática 04 - PWM e Sincronização entre Tarefas
 * Descrição: Controle de PWM
 */

#include <stdio.h>
#include <inttypes.h>
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
#include "driver/gpio.h"
#include "driver/gptimer.h" 
#include "driver/ledc.h"

// --- TAGs de Log ---
static const char *TAG1 = "PRATICA_01_LOG";
static const char *TAG2 = "PRATICA_02_BUTTON";
static const char *TAG3 = "PRATICA_03_TIMER"; 
static const char *TAG4 = "PRATICA_04_PWM"; 

#define LED_AZUL_GPIO       2
#define PWM_LED_GPIO        26 //GPIO para o LED do PWM
#define PWM_OSC_GPIO        32 //GPIO para o osciloscópio

//Configurações da Tarefa de Botões
#define DEBOUNCE_DELAY_MS 150
typedef struct {
    uint32_t gpio_num;
    TimerHandle_t timer_handle;
} button_config_t;
static button_config_t buttons[] = {
    { .gpio_num = 21, .timer_handle = NULL }, // MODO AUTOMÁTICO
    { .gpio_num = 22, .timer_handle = NULL }, // MODO MANUAL
    { .gpio_num = 23, .timer_handle = NULL }  // INCREMENTO
};
#define NUM_BUTTONS (sizeof(buttons) / sizeof(button_config_t))
static QueueHandle_t gpio_evt_queue = NULL;

//Configurações da Tarefa do Relógio
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
    bool set_auto_mode; // true para automático, false para manual
    int16_t increment;    // 1 para incrementar, 0 para mudar o modo
} PWM_elements_t;
static QueueHandle_t pwm_control_queue = NULL; 

//Semáforo para sincronização Timer
static SemaphoreHandle_t pwm_sync_semaphore = NULL;

//Funções dos Botões
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
                case 21: //Botão para modo automático
                    pwm_cmd.set_auto_mode = true;
                    pwm_cmd.increment = 0;
                    break;
                case 22://Botão para modo manual
                    pwm_cmd.set_auto_mode = false;
                    pwm_cmd.increment = -1;
                    break;
                case 23: //Botão para incrementar duty cycle
                    pwm_cmd.set_auto_mode = false;
                    pwm_cmd.increment = 1;
                    break;
                default:
                    command_valid = false;
                    break;
            }

            if (command_valid) {
                xQueueSend(pwm_control_queue, &pwm_cmd, portMAX_DELAY);
            }
        }
    }
}

//Funções da Tarefa do Relógio
static bool IRAM_ATTR timer_alarm_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    timer_event_t event_data = {
        .event_count = edata->count_value,
        .alarm_value = edata->alarm_value
    };
    xQueueSendFromISR(gptimer_queue, &event_data, &high_task_awoken);
    gptimer_alarm_config_t next_alarm_config = {
        .alarm_count = edata->alarm_value + 100000,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(timer, &next_alarm_config);
    return high_task_awoken == pdTRUE;
}
void gptimer_task(void *pvParameters) {
    ESP_LOGI(TAG3, "Iniciando a tarefa do GPTimer e configurando o relógio.");
    real_time_clock_t rtc = {0, 0, 0};
    int interrupt_counter = 0;
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, 
        .direction = GPTIMER_COUNT_UP,      
        .resolution_hz = 1 * 1000 * 1000,   
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 100000,
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
        if (xQueueReceive(gptimer_queue, &evt, portMAX_DELAY)) {
            //Sincroniza com a task PWM a cada 100ms
            xSemaphoreGive(pwm_sync_semaphore);
            interrupt_counter++;
            if (interrupt_counter >= 10) {
                interrupt_counter = 0;
                rtc.second++;
                if (rtc.second >= 60) {
                    rtc.second = 0;
                    rtc.minute++;
                    if (rtc.minute >= 60) {
                        rtc.minute = 0;
                        rtc.hour++;
                    }
                }
                ESP_LOGI(TAG3, "Hora: %02d:%02d:%02d", rtc.hour, rtc.minute, rtc.second);
            }
        }
    }
}
void pwm_task(void *pvParameters) {
    ESP_LOGI(TAG4, "Iniciando a tarefa do PWM.");

    //Configuração do Timer do PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT, // Resolução de 13 bits (0-8191)
        .freq_hz          = 5000,              // Frequência de 5 KHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    //Configuração do Canal 0 para o LED
    ledc_channel_config_t ledc_channel_0 = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM_LED_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_0));

    //Configuração do Canal 1 para o Osciloscópio
    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_0, 
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM_OSC_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));

    // Variáveis de estado da task
    bool is_auto_mode = true; // Começa em modo automático
    uint32_t duty_cycle = 0;
    int sawtooth_step = 82; // Incremento para onda (8192 / 100 passos ≈ 82)

    PWM_elements_t pwm_cmd;

    while (1) {
        if (xQueueReceive(pwm_control_queue, &pwm_cmd, 0) == pdPASS) {
            if(pwm_cmd.increment == 0) {
                 is_auto_mode = pwm_cmd.set_auto_mode;
            } else {
                 is_auto_mode = false; // Ação manual sempre desliga o automático
            }
            if (!is_auto_mode && pwm_cmd.increment != 0) {
                if (pwm_cmd.increment > 0) {
                    duty_cycle += 410; // Incrementa ~5%
                    if (duty_cycle > 8191) {
                        duty_cycle = 8191; // Trava no máximo
                    }
                } else if (pwm_cmd.increment < 0) {
                    if (duty_cycle < 410) {
                        duty_cycle = 0; // Trava no mínimo (evita underflow)
                    } else {
                        duty_cycle -= 410; // Decrementa ~5%
                    }
                }
            }
            ESP_LOGI(TAG4, "Comando recebido! Modo: %s | Duty Cycle Atual: %ld", is_auto_mode ? "AUTOMATICO" : "MANUAL", duty_cycle);
        }

        // 2. Executa a ação baseada no modo atual
        if (is_auto_mode) {
            if (xSemaphoreTake(pwm_sync_semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                duty_cycle += sawtooth_step;
                if (duty_cycle > 8191) {
                    duty_cycle = 0;
                }
            }
        } else {
            // delay para não usar 100% da CPU.
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        //Atualização do Hardware
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_cycle);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}
void app_main(void)
{
    //Bloco de informações do chip
    ESP_LOGI(TAG1, "Iniciando a aplicacao para obter informacoes do esp 32.");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG1, "Este e um %s com %d nucleo(s) de CPU.", CONFIG_IDF_TARGET, chip_info.cores);
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

    //Inicialização da Tarefa do Relógio
    ESP_LOGI(TAG3, "Configurando a tarefa do relogio...");
    gptimer_queue = xQueueCreate(5, sizeof(timer_event_t));
    xTaskCreate(gptimer_task, "gptimer_task", 2048, NULL, 5, NULL);

    //Inicialização da Tarefa PWM
    ESP_LOGI(TAG4, "Configurando a tarefa do PWM...");
    pwm_sync_semaphore = xSemaphoreCreateBinary();
    pwm_control_queue = xQueueCreate(5, sizeof(PWM_elements_t));
    xTaskCreate(pwm_task, "pwm_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG1, "Configuracao completa. Todas as tarefas estao em execucao.");
}