/*      Brenon Lenes Bettcher 
        Patrick Gonçalves
        Pratica 1
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h" 

/* * TAG para o sistema de logging serve para facilitar a identificação da origem das mensagens de log durante o debug.
 */
static const char *TAG = "SYS_INFO";

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando a aplicacao para obter informacoes do esp 32.");

    /* Estrutura para armazenar as informações do esp*/
    esp_chip_info_t chip_info;

    /* Popula a estrutura chip_info com os dados do hardware */
    esp_chip_info(&chip_info);

    // Exibe as informações do esp utilizando ESP_LOGI
    ESP_LOGI(TAG, "Este e um esp %s com %d nucleo(s) de CPU.", CONFIG_IDF_TARGET, chip_info.cores);
    
    // Constrói uma string com as features do esp para ficar de forma organizada
    char features[100];
    snprintf(features, sizeof(features), "%s%s%s%s",
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");
    ESP_LOGI(TAG, "Features: %s", features);


    // Calcula a revisão do silício (major e minor)
    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "Revisao do silicio: v%d.%d", major_rev, minor_rev);

    // Variável para armazenar o tamanho da flash
    uint32_t flash_size;
    // Tenta obter o tamanho da memória flash
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Tamanho da flash: %" PRIu32 "MB (%s)", 
                 flash_size / (uint32_t)(1024 * 1024),
                 (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embarcada" : "externa");
    } else {
        // Log de erro caso não consiga obter o tamanho da flash
        ESP_LOGE(TAG, "Falha ao obter o tamanho da memoria flash.");
    }

    // Exibe o mínimo de memória heap livre que o sistema já atingiu
    ESP_LOGI(TAG, "Minimo de heap livre: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());

    // Exibe a versão do ESP-IDF que está sendo utilizada
    ESP_LOGI(TAG, "Versao do ESP-IDF: %s", esp_get_idf_version());
    /*
     * Loop infinito para manter a task app_main em execução.
     * No FreeRTOS, se a função app_main retornar ela chega ao "fim",
     * a task associada a ela será pausada, o que pode levar a um
     * comportamento inesperado ou a um reset por "idle task".
     * O vTaskDelay impede que o loop consuma 100% da CPU, cedendo
     * tempo de processamento para outras tasks,
     */
    while(1) {
        ESP_LOGI(TAG, "Programa em execucao. Aguardando...");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Pausa a task por 1 segundos
    }
}