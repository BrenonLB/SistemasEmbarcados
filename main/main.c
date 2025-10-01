/*
 * Nomes: Brenon Lenes Bettcher, Patrick Gonçalves
 * Projeto: Prática 02 - Interrupções e GPIO
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" // Adicionado para a fila de eventos da ISR
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h" // Adicionado para controle de GPIO e interrupções

/* TAG para o sistema de logging, facilitando a identificação da origem das mensagens. */
static const char *TAG = "PRATICA_02";

/* Definições dos pinos GPIO para melhor legibilidade */
#define LED_AZUL_GPIO   2
#define BOTAO_0_GPIO    21 // Botão que alterna o estado do LED
#define BOTAO_1_GPIO    22 // Botão que liga o LED
#define BOTAO_2_GPIO    23 // Botão que desliga o LED

/* Fila para comunicação entre a rotina de interrupção (ISR) e a tarefa principal */
static QueueHandle_t gpio_evt_queue = NULL;

/*
 * Rotina de Tratamento da Interrupção (ISR) de GPIO.
 * Esta função é executada sempre que uma interrupção de um dos botões é detectada.
 * É importante que o código aqui seja o mais rápido possível.
 * Usamos IRAM_ATTR para garantir que a função seja carregada na RAM e execute rapidamente.
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    // Obtém o número do pino que gerou a interrupção
    uint32_t gpio_num = (uint32_t) arg;
    // Envia o número do pino para a fila para ser processado por uma tarefa
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/*
 * Tarefa para processar os eventos dos botões.
 * Esta tarefa fica bloqueada, aguardando por eventos na fila.
 * Quando um evento chega, ela o processa (controla o LED e gera o log).
 */
static void button_task(void* arg)
{
    uint32_t io_num;
    static bool led_state = 0; // Variável para armazenar o estado atual do LED

    while (true) {
        // Aguarda indefinidamente até que um item chegue na fila
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // Gera o log informando qual botão foi pressionado e seu estado atual (0 = pressionado)
            ESP_LOGI(TAG, "Botao no GPIO %" PRIu32 " foi apertado! Nivel: %d", io_num, gpio_get_level(io_num));

            // Lógica de controle do LED baseada no botão pressionado
            switch (io_num) {
                case BOTAO_1_GPIO: // Se o Botão 1 (GPIO22) foi pressionado
                    ESP_LOGI(TAG, "Ligando o LED azul.");
                    led_state = 1; // Define o estado como LIGADO
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;

                case BOTAO_2_GPIO: // Se o Botão 2 (GPIO23) foi pressionado
                    ESP_LOGI(TAG, "Desligando o LED azul.");
                    led_state = 0; // Define o estado como DESLIGADO
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;

                case BOTAO_0_GPIO: // Se o Botão 0 (GPIO21) foi pressionado
                    ESP_LOGI(TAG, "Alternando o estado do LED azul.");
                    led_state = !led_state; // Inverte o estado atual do LED
                    gpio_set_level(LED_AZUL_GPIO, led_state);
                    break;
            }
        }
    }
}


void app_main(void)
{
    /* Bloco de código da Prática 01 para exibir informações do chip - Mantido */
    ESP_LOGI(TAG, "Iniciando a aplicacao para obter informacoes do esp 32.");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Este e um esp %s com %d nucleo(s) de CPU.", CONFIG_IDF_TARGET, chip_info.cores);
    ESP_LOGI(TAG, "Versao do ESP-IDF: %s", esp_get_idf_version());
    /* Fim do bloco da Prática 01 */

    ESP_LOGI(TAG, "Configurando GPIOs para botoes e LED.");

    // Configuração do LED (GPIO2)
    gpio_reset_pin(LED_AZUL_GPIO);
    gpio_set_direction(LED_AZUL_GPIO, GPIO_MODE_OUTPUT);

    // Configuração dos botões
    const uint64_t pin_mask = (1ULL << BOTAO_0_GPIO) | (1ULL << BOTAO_1_GPIO) | (1ULL << BOTAO_2_GPIO);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Interrupção na borda de descida (quando o botão é pressionado)
    io_conf.pin_bit_mask = pin_mask;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1; // Habilita o resistor de PULL-UP interno
    gpio_config(&io_conf);

    // Cria a fila para armazenar os eventos de GPIO
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Cria a tarefa que irá processar os eventos dos botões
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    // Instala o serviço global de ISR de GPIO
    gpio_install_isr_service(0);

    // Adiciona o handler de ISR para cada um dos pinos dos botões
    gpio_isr_handler_add(BOTAO_0_GPIO, gpio_isr_handler, (void*) BOTAO_0_GPIO);
    gpio_isr_handler_add(BOTAO_1_GPIO, gpio_isr_handler, (void*) BOTAO_1_GPIO);
    gpio_isr_handler_add(BOTAO_2_GPIO, gpio_isr_handler, (void*) BOTAO_2_GPIO);

    ESP_LOGI(TAG, "Configuracao concluida. O programa esta em execucao e aguardando eventos dos botoes.");

    // O loop while(1) da prática anterior não é mais necessário aqui,
    // pois a tarefa button_task e o sistema FreeRTOS manterão o programa em execução.
}