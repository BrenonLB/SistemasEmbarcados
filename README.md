🌿 Smart Greenhouse IoT (Estufa Inteligente para Orquídeas)

> **Trabalho Final de Sistemas Embarcados** > **Instituição:** CENTRO FEDERAL DE EDUCAÇÃO TECNOLÓGICA DE MINAS GERAIS(CEFET-MG)
> **Autores:** Brenon Lenes Bettcher e Patrick Vinicius Gonçalves

## 📖 Descrição do Projeto

Este projeto consiste no desenvolvimento de uma **Estufa Inteligente IoT** baseada no microcontrolador **ESP32**. O sistema foi projetado para o cultivo de orquídeas e.g, monitorando condições climáticas locais e remotas para controlar automaticamente a iluminação e temperatura sob a planta.

O diferencial deste projeto é a integração de **dados meteorológicos reais (via API HTTP)**, atuando através de **PWM** para simular aquecimento e resfriamento, além de fornecer telemetria via **MQTT**.

---

## 🚀 Funcionalidades Principais

1. **Monitoramento IoT;**
* **Temperatura Local:** Leitura via ADC;
* **Temperatura Externa (Internet):** Consulta automática à API **Open-Meteo** para obter a temperatura real de Belo Horizonte (-19.91, -43.93).


2. **Controle Inteligente "Smart Mode":**
* **Modo FLORESCER (Orquídea):** Prioriza luz do sol((representada pelo LED na cor vermelha)-Meta: 25°C.
* **Modo CRESCER (Folhagem):** Prioriza luz UV(representada pelo LED na cor azul)-Meta: 18°C.


3. **Atuação PWM Híbrida:**
* O LED (Vermelho ou Azul) atua com **brilho base (~20%)** para fornecer o modo de monitoramento, sendo azul á representação da luz UV e o vermelho á representação da luz do sol.
* Caso a temperatura externa seja desfavorável, o LED sobe para **brilho máximo (100%)** atuando como Aquecedor(red) ou Refrigerador(blue).


4. **Interface Homem-Máquina (IHM):**
* **Display OLED (LVGL):** Exibe hora certa (SNTP), temperatura da internet, meta e status da atuação.
* **Botão Físico:** Alterna instantaneamente entre os modos de cultivo.


5. **Conectividade:**
* Sincronização de relógio via **NTP**.
* Envio de dados de telemetria via **MQTT** para monitoramento remoto.



---

## 🛠️ Hardware e Pinagem

O projeto utiliza um ESP32 DevKit V1 conectado aos seguintes periféricos:

| Componente | Função | GPIO (ESP32) | Observação |
| --- | --- | --- | --- |
| **LED Vermelho** | Aquecimento / Luz / Flor | **GPIO 33** | Controle PWM (Canal 0) |
| **LED Azul** | Resfriamento / Luz / Folha | **GPIO 26** | Controle PWM (Canal 1) |
| **Botão (Push)** | Alternar Modos | **GPIO 23** | Pull-up Interno + Interrupção |
| **Display OLED** | Interface Gráfica | **SDA: 19 / SCL: 18** | Comunicação I2C |

---

## 💻 Arquitetura de Software

O código foi desenvolvido em C utilizando o framework **ESP-IDF (v5.5.1)** e **FreeRTOS**. O sistema é multitarefa e utiliza os seguintes recursos avançados:

* **FreeRTOS Tasks:**
* `http_weather_task`: Realiza requisições GET periódicas (memória stack otimizada para JSON/TLS).
* `control_task`: Executa a lógica PID/Hysteresis e controle PWM.
* `lvgl_port_task`: Gerencia a atualização gráfica do display (Thread-safe).
* `adc_task`: Leitura de sensores locais.


* **Sincronização:** Uso de **Mutexes** e **Mutexes Recursivos** para evitar *Race Conditions* no acesso às variáveis globais e ao barramento do display.
* **Watchdog Protection:** Estrutura não-bloqueante para evitar reset do sistema durante conexões Wi-Fi.
* **Bibliotecas:** `esp_http_client`, `mqtt_client`, `esp_netif_sntp`, `lvgl`.

---

## 📊 Lógica de Controle

O sistema opera com uma máquina de estados baseada no modo escolhido:

### 1. Modo FLORESCER (LED Vermelho Ativo)

* **Objetivo:** Simular ambiente quente para indução de flores (solar-Infravermelho).
* **Regra:**
* Se `Temp_Internet < 25°C`: LED Vermelho em **100%** (Aquecendo).
* Se `Temp_Internet >= 25°C`: LED Vermelho em **20%** (Apenas luz solar necessária).



### 2. Modo CRESCER (LED Azul Ativo)

* **Objetivo:** Simular ambiente fresco para crescimento vegetativo(UV).
* **Regra:**
* Se `Temp_Internet > 18°C`: LED Azul em **100%** (Ventilando/Resfriando).
* Se `Temp_Internet <= 18°C`: LED Azul em **20%** (Apenas luz de crescimento).



---

## 🔧 Como Executar

### Pré-requisitos

* VS Code com extensão **ESP-IDF** instalada.
* Placa ESP32.

### Passos

1. **Clonar o Repositório:**
```bash
git clone https://github.com/BrenonLB/SistemasEmbarcados

```


2. **Configurar Wi-Fi:**
* O projeto utiliza o componente `protocol_examples_common`.
* No VS Code, abra o monitor e digite `idf.py menuconfig`.
* Vá em `Example Connection Configuration` e insira o SSID e Senha do seu Wi-Fi.


3. **Compilar e Flash:**
* Selecione o target (`Set Target`) para `esp32`.
* Clique em `Build` e depois em `Flash` ou clique no icone do foguinho.
* Acompanhe o funcionamento pelo `Monitor`.



---

## 📡 Tópicos MQTT

Para visualizar os dados, configure um cliente MQTT (como MQTT Explorer ou Painel Web) com as credenciais do **MyQttHub**:

* **Broker:** `node02.myqtthub.com`
* **Porta:** `1883`
* **Tópico de Monitoramento:** `g2device/monitor`
* **Tópico de Comando (Cor):** `g2device/color`

**Exemplo de Payload recebido:**

```text
BH_Temp:25.4 Mode:FLOR Action:Temp OK. Luz On

```

---

*Desenvolvido para a disciplina de Sistemas Embarcados - 2025*
