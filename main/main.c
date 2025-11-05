/**
 * Nomes: Brenon Lenes Bettcher, Patrick Gonçalves
 * Projeto: Prática 05 - Leitura de ADC
 * Descrição: Controle de PWM e leitura de ADC sincronizada por tarefas.
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
//Biblioteca para calibragem do ADC
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG1 = "PRATICA_01_LOG";
static const char *TAG2 = "PRATICA_02_BUTTON";
static const char *TAG3 = "PRATICA_03_TIMER"; 
static const char *TAG4 = "PRATICA_04_PWM"; 
static const char *TAG5 = "PRATICA_05_ADC"; 

#define LED_AZUL_GPIO       2
#define PWM_LED_GPIO        26 
#define PWM_OSC_GPIO        33 
// (O ADC_CHANNEL_3 é o GPIO34)

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

// Configurações do relogio
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

//Configurações do PWM
typedef struct {
    bool set_auto_mode; 
    int16_t increment;    
} PWM_elements_t;
static QueueHandle_t pwm_control_queue = NULL; 
static SemaphoreHandle_t pwm_sync_semaphore = NULL;

//Configurações do ADC
// Estrutura para passar dados do ADC para o Timer
typedef struct {
    int raw;
    int voltage; // Tensão em mV
} adc_data_t;

//Fila para comunicação ADC -> Timer
static QueueHandle_t adc_data_queue = NULL;
//Semáforo para sincronização Timer -> ADC
static SemaphoreHandle_t adc_sync_semaphore = NULL;

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
                case 21: 
                    pwm_cmd.set_auto_mode = true;
                    pwm_cmd.increment = 0;
                    break;
                case 22: 
                    pwm_cmd.set_auto_mode = false;
                    pwm_cmd.increment = -1; 
                    break;
                case 23: 
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
//Task do timer agora também recebe dados do ADC e loga
void gptimer_task(void *pvParameters) {
    ESP_LOGI(TAG3, "Iniciando a tarefa do GPTimer e configurando o relógio.");
    real_time_clock_t rtc = {0, 0, 0};
    int interrupt_counter = 0;
    
    //Variável para armazenar a última leitura do ADC
    adc_data_t last_adc_data = {0, 0};

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
    gptimer_event_callbacks_t cbs = { .on_alarm = timer_alarm_isr_callback };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    timer_event_t evt;
    
    while (1) {
        if (xQueueReceive(gptimer_queue, &evt, portMAX_DELAY)) {
            
            //Sincroniza com a task PWM a cada 100ms
            xSemaphoreGive(pwm_sync_semaphore);
            //Sincroniza com a task ADC a cada 100ms
            xSemaphoreGive(adc_sync_semaphore);

            if (xQueueReceive(adc_data_queue, &last_adc_data, 0) == pdPASS) {
            }

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
                        if (rtc.hour >= 24) {
                            rtc.hour = 0;
                        }
                    }
                }
                ESP_LOGI(TAG3, "Hora: %02d:%02d:%02d", rtc.hour, rtc.minute, rtc.second);
                ESP_LOGI(TAG5, "Leitura ADC: Raw= %d | Tensao= %d mV", last_adc_data.raw, last_adc_data.voltage);
            }
        }
    }
}

void pwm_task(void *pvParameters) {
    ESP_LOGI(TAG4, "Iniciando a tarefa do PWM.");
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT, // Resolução de 13 bits (0-8191)
        .freq_hz          = 5000,              // Frequência de 5 KHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

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

    bool is_auto_mode = true; 
    uint32_t duty_cycle = 0;
    int sawtooth_step = 82;
    PWM_elements_t pwm_cmd;

    while (1) {
        if (xQueueReceive(pwm_control_queue, &pwm_cmd, 0) == pdPASS) {
            if(pwm_cmd.increment == 0) {
                 is_auto_mode = pwm_cmd.set_auto_mode;
            } else {
                 is_auto_mode = false; 
            }
            if (!is_auto_mode && pwm_cmd.increment != 0) {
                if (pwm_cmd.increment > 0) {
                    duty_cycle += 410; 
                    if (duty_cycle > 8191) {
                        duty_cycle = 8191; 
                    }
                } else if (pwm_cmd.increment < 0) {
                    if (duty_cycle < 410) {
                        duty_cycle = 0; 
                    } else {
                        duty_cycle -= 410; 
                    }
                }
            }
            ESP_LOGI(TAG4, "Comando recebido! Modo: %s | Duty Cycle Atual: %ld", is_auto_mode ? "AUTOMATICO" : "MANUAL", duty_cycle);
        }l
        if (is_auto_mode) {
            if (xSemaphoreTake(pwm_sync_semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                duty_cycle += sawtooth_step;
                if (duty_cycle > 8191) {
                    duty_cycle = 0;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_cycle);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}


//Tarefa e Funções do ADC
void adc_task(void *pvParameters) {
    ESP_LOGI(TAG5, "Iniciando a tarefa do ADC.");
    //Configuração do ADC 
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //Configuração do Canal
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, //Resolução máxima (default é 12 bits)
        .atten = ADC_ATTEN_DB_11 //Atenuação para ler a faixa completa (0-3.3V)
    };
    //Associa ao ADC_CHANNEL_3
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
    //Configuração da Calibração (para obter Tensão em mV)
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

    adc_data_t data_to_send;

    while (1) {
        if (xSemaphoreTake(adc_sync_semaphore, portMAX_DELAY) == pdTRUE) {
            
            //Realiza a leitura do ADC
            int raw_value;
            int voltage_value;
            
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &raw_value));
            //Converte o valor bruto (raw) para tensão (mV)
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw_value, &voltage_value));
            data_to_send.raw = raw_value;
            data_to_send.voltage = voltage_value;
            //Envia os dados lidos para a fila da gptimer_task
            xQueueSend(adc_data_queue, &data_to_send, 0);
        }
    }
}

void app_main(void){
    ESP_LOGI(TAG1, "Iniciando a aplicacao para obter informacoes do esp 32.");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG1, "Este e um %s com %d nucleo(s) de CPU.", CONFIG_IDF_TARGET, chip_info.cores);
    ESP_LOGI(TAG1, "Versao do ESP-IDF: %s", esp_get_idf_version());

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

    ESP_LOGI(TAG3, "Configurando a tarefa do relogio...");
    gptimer_queue = xQueueCreate(5, sizeof(timer_event_t));
    xTaskCreate(gptimer_task, "gptimer_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG4, "Configurando a tarefa do PWM...");
    pwm_sync_semaphore = xSemaphoreCreateBinary();
    pwm_control_queue = xQueueCreate(5, sizeof(PWM_elements_t));
    xTaskCreate(pwm_task, "pwm_task", 2048, NULL, 5, NULL);

    //Inicialização da Tarefa ADC
    ESP_LOGI(TAG5, "Configurando a tarefa do ADC...");
    adc_sync_semaphore = xSemaphoreCreateBinary();
    adc_data_queue = xQueueCreate(5, sizeof(adc_data_t));
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG1, "Configuracao completa. Todas as tarefas estao em execucao.");
}