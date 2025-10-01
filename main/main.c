/*
 * Nomes: Brenon Lenes Bettcher, Patrick Gonçalves
 * Projeto: Prática 02 - Interrupções e GPIO (Com correção de Debounce por Timer)
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

static const char *TAG = "PRATICA_02_TIMER";

#define LED_AZUL_GPIO   2
#define BOTAO_0_GPIO    21
#define BOTAO_1_GPIO    22
#define BOTAO_2_GPIO    23

#define DEBOUNCE_DELAY_MS 150 

static QueueHandle_t gpio_evt_queue = NULL;

static TimerHandle_t button_timers[3];


 // Função de callback que é executada quando um timer de debounce expira.

static void button_timer_callback(TimerHandle_t xTimer)
{
    // Pega o ID do timer, que configuramos para ser o número do pino GPIO
    uint32_t gpio_num = (uint32_t) pvTimerGetTimerID(xTimer);

    // Verifica o estado atual do pino.
    
    if (gpio_get_level(gpio_num) == 0) {
  
        xQueueSend(gpio_evt_queue, &gpio_num, portMAX_DELAY);
    }
}



static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    TimerHandle_t timer_to_reset = NULL;

    // Identifica qual timer deve ser reiniciado
    if (gpio_num == BOTAO_0_GPIO) {
        timer_to_reset = button_timers[0];
    } else if (gpio_num == BOTAO_1_GPIO) {
        timer_to_reset = button_timers[1];
    } else if (gpio_num == BOTAO_2_GPIO) {
        timer_to_reset = button_timers[2];
    }

    if (timer_to_reset != NULL) {
        // Reinicia o timer. Se o timer não estiver ativo, ele o inicia.
        // Isso garante que a callback só será chamada após o sinal ficar estável.
        xTimerResetFromISR(timer_to_reset, NULL);
    }
}


static void button_task(void* arg)
{
    uint32_t io_num;
    static bool led_state = 0;

    while (true) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Botao no GPIO %" PRIu32 " acionado e validado!", io_num);

            switch (io_num) {
                case BOTAO_1_GPIO:
                    ESP_LOGI(TAG, "Ligando o LED azul.");
                    led_state = 1;
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;

                case BOTAO_2_GPIO:
                    ESP_LOGI(TAG, "Desligando o LED azul.");
                    led_state = 0;
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;

                case BOTAO_0_GPIO:
                    ESP_LOGI(TAG, "Alternando o estado do LED azul.");
                    led_state = !led_state;
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando a aplicacao...");

  
    gpio_reset_pin(LED_AZUL_GPIO);
    gpio_set_direction(LED_AZUL_GPIO, GPIO_MODE_OUTPUT);

 
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL<<BOTAO_0_GPIO) | (1ULL<<BOTAO_1_GPIO) | (1ULL<<BOTAO_2_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    // Timer para o Botão 0
    button_timers[0] = xTimerCreate("timer_btn0", pdMS_TO_TICKS(DEBOUNCE_DELAY_MS), pdFALSE, (void*) BOTAO_0_GPIO, button_timer_callback);
    // Timer para o Botão 1
    button_timers[1] = xTimerCreate("timer_btn1", pdMS_TO_TICKS(DEBOUNCE_DELAY_MS), pdFALSE, (void*) BOTAO_1_GPIO, button_timer_callback);
    // Timer para o Botão 2
    button_timers[2] = xTimerCreate("timer_btn2", pdMS_TO_TICKS(DEBOUNCE_DELAY_MS), pdFALSE, (void*) BOTAO_2_GPIO, button_timer_callback);



    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO_0_GPIO, gpio_isr_handler, (void*) BOTAO_0_GPIO);
    gpio_isr_handler_add(BOTAO_1_GPIO, gpio_isr_handler, (void*) BOTAO_1_GPIO);
    gpio_isr_handler_add(BOTAO_2_GPIO, gpio_isr_handler, (void*) BOTAO_2_GPIO);

    ESP_LOGI(TAG, "Configuracao completa. O sistema esta pronto.");
}