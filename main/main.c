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

static const char *TAG1 = "PRATICA_01_LOG";
static const char *TAG2 = "PRATICA_02_TIMER";

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
            ESP_LOGI(TAG2, "Botao no GPIO %" PRIu32 " acionado e validado!", io_num);

            switch (io_num) {
                case BOTAO_1_GPIO:
                    ESP_LOGI(TAG2, "Ligando o LED azul.");
                    led_state = 1;
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;

                case BOTAO_2_GPIO:
                    ESP_LOGI(TAG2, "Desligando o LED azul.");
                    led_state = 0;
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;

                case BOTAO_0_GPIO:
                    ESP_LOGI(TAG2, "Alternando o estado do LED azul.");
                    led_state = !led_state;
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;
            }
        }
    }
}

void app_main(void)
{

    ESP_LOGI(TAG1, "Iniciando a aplicacao para obter informacoes do esp 32.");

    /* Estrutura para armazenar as informações do esp*/
    esp_chip_info_t chip_info;

    /* Mostra a estrutura chip_info com os dados do hardware */
    esp_chip_info(&chip_info);

    // Exibe as informações do esp utilizando ESP_LOGI
    ESP_LOGI(TAG1, "Este e um esp %s com %d nucleo(s) de CPU.", CONFIG_IDF_TARGET, chip_info.cores);
    
    // Constrói uma string com as features do esp para ficar de forma organizada
    char features[100];
    snprintf(features, sizeof(features), "%s%s%s%s",
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");
    ESP_LOGI(TAG1, "Features: %s", features);


    // Calcula a revisão do esp 
    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG1, "Revisao do esp: v%d.%d", major_rev, minor_rev);

    // Variável para armazenar o tamanho da flash
    uint32_t flash_size;
    // Tenta obter o tamanho da memória flash
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG1, "Tamanho da flash: %" PRIu32 "MB (%s)", 
                 flash_size / (uint32_t)(1024 * 1024),
                 (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embarcada" : "externa");
    } else {
        // Log de erro caso não consiga obter o tamanho da flash
        ESP_LOGE(TAG1, "Falha ao obter o tamanho da memoria flash.");
    }

    // Exibe o mínimo de memória heap livre que o sistema já atingiu
    ESP_LOGI(TAG1, "Minimo de heap livre: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());

    // Exibe a versão do ESP-IDF que está sendo utilizada
    ESP_LOGI(TAG1, "Versao do ESP-IDF: %s", esp_get_idf_version());

    ESP_LOGI(TAG2, "Iniciando a aplicacao...");

  
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

    ESP_LOGI(TAG2, "Configuracao completa. O sistema esta pronto.");
}