/*
 * ESP32-S3 MCPWM Inversor trifasico
 * Hardware Alvo: ESP32-S3
 * Módulo Inversor: FSBB30CH60C
 * Framework: Arduino
 * Autor: José Fonseca
 *
 *
 * Características:
 * - Geração de SPWM de 3 fases
 * - Monitoramento de 6 sensores de corrente SCT013
 * - Monitoramento de 3 sensores de tensão ZMPT
 * - Proteção contra sobrecorrente (>5A disparo)
 * - Monitoramento do pino de segurança VFO
 * - Interface serial para usuário
 * - Monitoramento de sensores em tempo real
 *
 * Pinagem:
 * Fase U, Lado Alto:  GPIO 18    |  Sensores de Corrente (ADC1):
 * Fase U, Lado Baixo: GPIO 35    |  - AmpSens_L1_AF: GPIO 1  (ADC1_CH0)
 * Fase V, Lado Alto:  GPIO 13    |  - AmpSens_L1_DF: GPIO 2  (ADC1_CH1)
 * Fase W, Lado Alto:  GPIO 14    |  - AmpSens_L2_AF: GPIO 3  (ADC1_CH2)
 * Fase W, Lado Baixo: GPIO 17    |  - AmpSens_L2_DF: GPIO 4  (ADC1_CH3)
 * Fase V, Lado Baixo: GPIO 21    |  - AmpSens_L3_AF: GPIO 5  (ADC1_CH4)
 *                                |  - AmpSens_L3_DF: GPIO 6  (ADC1_CH5)
 * Sensores de Tensão (ADC1):     |
 * - VoltSens_L1: GPIO 7  (ADC1_CH6)  |  Segurança:
 * - VoltSens_L2: GPIO 8  (ADC1_CH7)  |  - Pino VFO: GPIO 36 (entrada)
 * - VoltSens_L3: GPIO 9  (ADC1_CH8)  |
 *                                    |  Sensor Vbus (ADC1):
 *                                    |  - VoltSens_Link: GPIO 10 (ADC1_CH9)
 *                                    |    Divisor: Ra=1.2MΩ, Rb=10kΩ
 */

#include "driver/mcpwm_prelude.h"
#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "math.h"

//============================
// Definições de Pinos
//============================

// Pinos PWM
#define PWM_UH_PIN 18 // Fase U, Lado Alto
#define PWM_UL_PIN 35 // Fase U, Lado Baixo
#define PWM_VH_PIN 13 // Fase V, Lado Alto
#define PWM_VL_PIN 21 // Fase V, Lado Baixo
#define PWM_WH_PIN 14 // Fase W, Lado Alto
#define PWM_WL_PIN 17 // Fase W, Lado Baixo

// Pinos dos Sensores de Corrente (SCT013)
#define SCT013_U_IN_PIN ADC_CHANNEL_0  // GPIO 1 - AmpSens_L1_AF
#define SCT013_U_OUT_PIN ADC_CHANNEL_1 // GPIO 2 - AmpSens_L1_DF
#define SCT013_V_IN_PIN ADC_CHANNEL_2  // GPIO 3 - AmpSens_L2_AF
#define SCT013_V_OUT_PIN ADC_CHANNEL_3 // GPIO 4 - AmpSens_L2_DF
#define SCT013_W_IN_PIN ADC_CHANNEL_4  // GPIO 5 - AmpSens_L3_AF
#define SCT013_W_OUT_PIN ADC_CHANNEL_5 // GPIO 6 - AmpSens_L3_DF

// Pinos dos Sensores de Tensão (ZMPT)
#define ZMPT_U_PIN ADC_CHANNEL_6 // GPIO 7 - VoltSens_L1
#define ZMPT_V_PIN ADC_CHANNEL_7 // GPIO 8 - VoltSens_L2
#define ZMPT_W_PIN ADC_CHANNEL_8 // GPIO 9 - VoltSens_L3

// Sensor de Tensão Vbus (ADC1)
#define VBUS_SENSE_PIN ADC_CHANNEL_9 // GPIO 10 - VoltSens_Link

// Pino de Segurança
#define VFO_PIN 36 // Status VFO do módulo inversor

//=========================
// Configurações SPWM
//=========================

#define PWM_FREQUENCY 10000             // Frequência portadora PWM em Hz
#define PWM_TIMER_RESOLUTION_HZ 1000000 // Resolução do timer MCPWM (1MHz -> 1us tick)
#define PWM_PERIOD_TICKS (PWM_TIMER_RESOLUTION_HZ / PWM_FREQUENCY)
#define DEAD_TIME_NS 2000 // Tempo morto em nanossegundos

// Parâmetros SPWM
#define MODULATION_FREQUENCY 50 // Frequência de saída em Hz (50 Hz para AC de 50Hz)
#define SPWM_UPDATE_FREQ 1000   // Frequência de atualização SPWM em Hz
#define SPWM_UPDATE_PERIOD_MS (1000 / SPWM_UPDATE_FREQ)

// Deslocamentos de fase (em radianos)
#define PHASE_U_OFFSET 0.0
#define PHASE_V_OFFSET (2.0 * M_PI / 3.0) // 120°
#define PHASE_W_OFFSET (4.0 * M_PI / 3.0) // 240°

//=========================================
// Configurações de Segurança e Controle
//=========================================

#define MAX_CURRENT_AMPS 5.0 // Corrente máxima permitida por fase
#define SENSOR_UPDATE_MS 10  // Período de atualização da leitura dos sensores

// Configuração do Modo Contínuo ADC
#define ADC_SAMPLE_FREQ_HZ 20000 // Frequência de amostragem de 20 kHz (400x 50Hz)
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type1.data)

// Tamanhos dos buffers para DMA
#define ADC_READ_LEN 1024   // Tamanho do buffer DMA
#define ADC_BUFFER_COUNT 4  // Número de buffers DMA
#define RMS_WINDOW_SIZE 400 // Amostras para cálculo RMS (20ms em 20kHz)
#define NUM_ADC_CHANNELS 10 // Total de canais ADC (incluindo Vbus)

// Constantes de calibração dos sensores
#define SCT013_SENSITIVITY 0.066 // Sensibilidade SCT013 (V/A) para versão 30A
#define ZMPT_SENSITIVITY 0.0055  // Sensibilidade ZMPT (V/V) típica
#define ADC_VREF 3300            // Tensão de referência ADC em mV

// Constantes do divisor de tensão Vbus
#define VBUS_R1 1200000.0                                    // Ra = 1.2MΩ (resistor superior)
#define VBUS_R2 10000.0                                      // Rb = 10kΩ (resistor inferior)
#define VBUS_DIVIDER_RATIO ((VBUS_R2) / (VBUS_R1 + VBUS_R2)) // Fator do divisor
#define MAX_VBUS_VOLTAGE 400.0                               // Tensão máxima permitida do barramento (V)

//==================
// Handles Globais
//==================

mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper_u = NULL, oper_v = NULL, oper_w = NULL;
mcpwm_cmpr_handle_t comparator_u = NULL, comparator_v = NULL, comparator_w = NULL;
mcpwm_gen_handle_t generator_uh = NULL, generator_ul = NULL;
mcpwm_gen_handle_t generator_vh = NULL, generator_vl = NULL;
mcpwm_gen_handle_t generator_wh = NULL, generator_wl = NULL;

// Handles ADC
adc_continuous_handle_t adc_handle = NULL;
adc_cali_handle_t adc_cali_handle = NULL;

// Variáveis DMA e amostragem
static uint8_t adc_result[ADC_READ_LEN] = {0};
static SemaphoreHandle_t adc_semaphore = NULL;
static TaskHandle_t adc_task_handle = NULL;

// Buffers circulares para cálculo RMS
static float adc_buffers[NUM_ADC_CHANNELS][RMS_WINDOW_SIZE];
static uint16_t buffer_indices[NUM_ADC_CHANNELS] = {0};
static bool buffers_full[NUM_ADC_CHANNELS] = {false};

//====================
// Variáveis Globais
//====================

// Variáveis SPWM
volatile float spwm_angle = 0.0;
volatile unsigned long last_spwm_update = 0;
volatile bool spwm_enabled = false;
volatile float modulation_index = 0.8; // Índice de modulação fixo (sem controle)

// Variáveis dos Sensores
struct SensorData
{
    float current_u_in, current_u_out;
    float current_v_in, current_v_out;
    float current_w_in, current_w_out;
    float voltage_u, voltage_v, voltage_w;
    float vbus_voltage; // Tensão do barramento DC
    bool vfo_status;
} sensors;

// Variáveis de Segurança
volatile bool overcurrent_trip = false;
volatile bool vfo_trip = false;
volatile bool system_fault = false;
unsigned long last_sensor_update = 0;

//=========================
// Protótipos das Funções
//=========================

void setup_mcpwm(void);
void setup_adc_continuous(void);
void setup_safety_pins(void);
void update_spwm(void);
void update_sensors(void);
void check_safety_conditions(void);
void set_duty_cycle(mcpwm_cmpr_handle_t comparator, float duty_percent);
void enable_spwm(void);
void disable_spwm(void);
void emergency_stop(const char *reason);
float read_adc_channel(adc_channel_t channel);
float read_vbus_voltage(void);
float calculate_current_rms(float adc_voltage);
float calculate_voltage_rms(float adc_voltage);
void process_serial_commands(void);
void print_sensor_data(void);
void adc_processing_task(void *pvParameters);
void process_adc_data(uint8_t *data, uint32_t size);
void add_sample_to_buffer(uint8_t channel_idx, float sample);
float calculate_rms_from_buffer(uint8_t channel_idx);
float adc_raw_to_voltage(uint32_t adc_raw);
static bool IRAM_ATTR adc_continuous_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);

//===============
// Função Setup
//===============

void setup()
{
    Serial.begin(115200);

    // Inicializar todos os subsistemas
    setup_safety_pins();
    setup_adc_continuous();
    setup_mcpwm();

    // Criar tarefa de processamento ADC e semáforo
    adc_semaphore = xSemaphoreCreateBinary();
    xTaskCreate(adc_processing_task, "adc_task", 4096, NULL, 5, &adc_task_handle);

    // Iniciar conversão contínua ADC
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    Serial.println("Sistema inicializado - Pronto para operação");
    Serial.println("Comandos: start, stop, status, sensors, reset, set_mod <0.1-1.0>");
    Serial.println("AVISO: Verifique todos os sistemas de segurança antes de iniciar!");
}

//======================================
// Configuração dos Pinos de Segurança
//======================================

void setup_safety_pins(void)
{
    pinMode(VFO_PIN, INPUT_PULLUP);
    Serial.println("Pinos de segurança configurados");
}

//=====================================
// Função de Configuração ADC Contínuo
//=====================================

void setup_adc_continuous(void)
{
    Serial.println("Inicializando Modo Contínuo ADC com DMA...");

    // Configurar modo contínuo ADC
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_READ_LEN * ADC_BUFFER_COUNT,
        .conv_frame_size = ADC_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    // Configurar padrões de conversão
    adc_digi_pattern_config_t adc_pattern[NUM_ADC_CHANNELS] = {};

    // Sensores de corrente (canais 0-5)
    adc_pattern[0] = {.atten = ADC_ATTEN_DB_11, .channel = SCT013_U_IN_PIN, .unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH};
    adc_pattern[1] = {.atten = ADC_ATTEN_DB_11, .channel = SCT013_U_OUT_PIN, .unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH};
    adc_pattern[2] = {.atten = ADC_ATTEN_DB_11, .channel = SCT013_V_IN_PIN, .unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH};
    adc_pattern[3] = {.atten = ADC_ATTEN_DB_11, .channel = SCT013_V_OUT_PIN, .unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH};
    adc_pattern[4] = {.atten = ADC_ATTEN_DB_11, .channel = SCT013_W_IN_PIN, .unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH};
    adc_pattern[5] = {.atten = ADC_ATTEN_DB_11, .channel = SCT013_W_OUT_PIN, .unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH};

    // Sensores de tensão (canais 6-8)
    adc_pattern[6] = {.atten = ADC_ATTEN_DB_11, .channel = ZMPT_U_PIN, .unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH};
    adc_pattern[7] = {.atten = ADC_ATTEN_DB_11, .channel = ZMPT_V_PIN, .unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH};
    adc_pattern[8] = {.atten = ADC_ATTEN_DB_11, .channel = ZMPT_W_PIN, .unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH};

    // Sensor Vbus (canal 9)
    adc_pattern[9] = {.atten = ADC_ATTEN_DB_11, .channel = VBUS_SENSE_PIN, .unit = ADC_UNIT_1, .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH};

    adc_continuous_config_t dig_cfg = {
        .pattern_num = NUM_ADC_CHANNELS,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));

    // Registrar callback
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_continuous_callback,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));

    // Inicializar calibração
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle));

    // Inicializar buffers circulares
    for (int i = 0; i < NUM_ADC_CHANNELS; i++)
    {
        buffer_indices[i] = 0;
        buffers_full[i] = false;
        memset(adc_buffers[i], 0, sizeof(adc_buffers[i]));
    }

    Serial.printf("Modo Contínuo ADC inicializado:\n");
    Serial.printf("- Taxa de Amostragem: %d Hz\n", ADC_SAMPLE_FREQ_HZ);
    Serial.printf("- Canais: %d\n", NUM_ADC_CHANNELS);
    Serial.printf("- Janela RMS: %d amostras (%.1f ms)\n", RMS_WINDOW_SIZE, (float)RMS_WINDOW_SIZE * 1000.0 / ADC_SAMPLE_FREQ_HZ);
}

//========================
// Callback ADC Contínuo
//========================

static bool IRAM_ATTR adc_continuous_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Sinalizar tarefa de processamento
    vTaskNotifyGiveFromISR(adc_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

//===============================
// Tarefa de Processamento ADC
//================================

void adc_processing_task(void *pvParameters)
{
    uint32_t ret_num = 0;

    while (1)
    {
        // Aguardar notificação do callback
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Ler dados disponíveis
        esp_err_t ret = adc_continuous_read(adc_handle, adc_result, ADC_READ_LEN, &ret_num, 0);
        if (ret == ESP_OK)
        {
            process_adc_data(adc_result, ret_num);
        }
    }
}

//=======================
// Processar Dados ADC
//=======================

void process_adc_data(uint8_t *data, uint32_t size)
{
    adc_digi_output_data_t *p = NULL;

    for (int i = 0; i < size; i += SOC_ADC_DIGI_RESULT_BYTES)
    {
        p = (adc_digi_output_data_t *)&data[i];

        if (ADC_GET_CHANNEL(p) < NUM_ADC_CHANNELS)
        {
            uint32_t adc_raw = ADC_GET_DATA(p);
            float voltage = adc_raw_to_voltage(adc_raw);
            add_sample_to_buffer(ADC_GET_CHANNEL(p), voltage);
        }
    }
}

//=======================================
// Adicionar Amostra ao Buffer Circular
//=======================================

void add_sample_to_buffer(uint8_t channel_idx, float sample)
{
    if (channel_idx >= NUM_ADC_CHANNELS)
        return;

    adc_buffers[channel_idx][buffer_indices[channel_idx]] = sample;
    buffer_indices[channel_idx]++;

    if (buffer_indices[channel_idx] >= RMS_WINDOW_SIZE)
    {
        buffer_indices[channel_idx] = 0;
        buffers_full[channel_idx] = true;
    }
}

//=========================
// Calcular RMS do Buffer
//=========================

float calculate_rms_from_buffer(uint8_t channel_idx)
{
    if (channel_idx >= NUM_ADC_CHANNELS)
        return 0.0;

    // Aguardar buffer ser preenchido pelo menos uma vez
    if (!buffers_full[channel_idx])
        return 0.0;

    float sum_squares = 0.0;
    uint16_t samples_to_use = RMS_WINDOW_SIZE;

    for (int i = 0; i < samples_to_use; i++)
    {
        float sample = adc_buffers[channel_idx][i];
        sum_squares += sample * sample;
    }

    return sqrt(sum_squares / samples_to_use);
}

//=================================
// Conversão ADC Raw para Tensão
//=================================

float adc_raw_to_voltage(uint32_t adc_raw)
{
    int voltage_mv = 0;
    esp_err_t ret = adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv);
    if (ret != ESP_OK)
    {
        return 0.0;
    }
    return (float)voltage_mv / 1000.0; // Converter mV para V
}

//================================
// Função de Configuração MCPWM
//================================

void setup_mcpwm(void)
{
    Serial.println("Inicializando MCPWM para SPWM...");

    // Criar timer
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = PWM_TIMER_RESOLUTION_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = PWM_PERIOD_TICKS;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // Criar operadores
    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_u));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_v));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_w));

    // Conectar operadores ao timer
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_u, timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_v, timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_w, timer));

    // Criar comparadores
    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_u, &comparator_config, &comparator_u));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_v, &comparator_config, &comparator_v));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_w, &comparator_config, &comparator_w));

    // Definir duty cycle inicial
    uint32_t initial_duty_ticks = PWM_PERIOD_TICKS / 2;
    if (initial_duty_ticks >= PWM_PERIOD_TICKS)
    {
        initial_duty_ticks = PWM_PERIOD_TICKS - 1;
    }

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_u, initial_duty_ticks));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_v, initial_duty_ticks));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_w, initial_duty_ticks));

    // Criar geradores
    mcpwm_generator_config_t gen_config = {};

    // Geradores Fase U
    gen_config.gen_gpio_num = PWM_UH_PIN;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_u, &gen_config, &generator_uh));
    gen_config.gen_gpio_num = PWM_UL_PIN;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_u, &gen_config, &generator_ul));

    // Geradores Fase V
    gen_config.gen_gpio_num = PWM_VH_PIN;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_v, &gen_config, &generator_vh));
    gen_config.gen_gpio_num = PWM_VL_PIN;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_v, &gen_config, &generator_vl));

    // Geradores Fase W
    gen_config.gen_gpio_num = PWM_WH_PIN;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_w, &gen_config, &generator_wh));
    gen_config.gen_gpio_num = PWM_WL_PIN;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_w, &gen_config, &generator_wl));

    // Configurar ações dos geradores
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_uh,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_uh,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_u, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_vh,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_vh,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_v, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_wh,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_wh,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_w, MCPWM_GEN_ACTION_LOW)));

    // Configurar tempo morto
    const uint32_t dead_time_ticks = (uint32_t)(DEAD_TIME_NS * PWM_TIMER_RESOLUTION_HZ / 1000000000);

    auto configure_dead_time = [dead_time_ticks](mcpwm_gen_handle_t gena, mcpwm_gen_handle_t genb)
    {
        mcpwm_dead_time_config_t dead_time_config = {
            .posedge_delay_ticks = dead_time_ticks,
            .negedge_delay_ticks = 0};
        ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gena, gena, &dead_time_config));

        dead_time_config.posedge_delay_ticks = 0;
        dead_time_config.negedge_delay_ticks = dead_time_ticks;
        dead_time_config.flags.invert_output = true;
        ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gena, genb, &dead_time_config));
    };

    configure_dead_time(generator_uh, generator_ul);
    configure_dead_time(generator_vh, generator_vl);
    configure_dead_time(generator_wh, generator_wl);

    // Habilitar e iniciar timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    Serial.println("Inicialização MCPWM completa!");
}

//=================================================================
// Função de Leitura ADC (Agora usa RMS da amostragem contínua)
//=================================================================

float read_adc_channel(adc_channel_t channel)
{
    // Mapear canal ADC para índice do buffer
    uint8_t channel_idx = 0;

    switch (channel)
    {
    case SCT013_U_IN_PIN:
        channel_idx = 0;
        break;
    case SCT013_U_OUT_PIN:
        channel_idx = 1;
        break;
    case SCT013_V_IN_PIN:
        channel_idx = 2;
        break;
    case SCT013_V_OUT_PIN:
        channel_idx = 3;
        break;
    case SCT013_W_IN_PIN:
        channel_idx = 4;
        break;
    case SCT013_W_OUT_PIN:
        channel_idx = 5;
        break;
    case ZMPT_U_PIN:
        channel_idx = 6;
        break;
    case ZMPT_V_PIN:
        channel_idx = 7;
        break;
    case ZMPT_W_PIN:
        channel_idx = 8;
        break;
    case VBUS_SENSE_PIN:
        channel_idx = 9;
        break;
    default:
        return 0.0;
    }

    return calculate_rms_from_buffer(channel_idx);
}

//===========================================
// Funções de Cálculo de Corrente e Tensão
//===========================================

float calculate_current_rms(float adc_voltage)
{
    // SCT013 produz tensão AC proporcional à corrente AC
    // Assumindo que o ADC lê a tensão RMS após retificação e filtragem
    // Corrente (A) = Tensão_ADC (V) / Sensibilidade (V/A)
    return adc_voltage / SCT013_SENSITIVITY;
}

float calculate_voltage_rms(float adc_voltage)
{
    // ZMPT produz tensão AC proporcional à tensão AC
    // Tensão (V) = Tensão_ADC (V) / Sensibilidade (V/V)
    return adc_voltage / ZMPT_SENSITIVITY;
}

//=======================================
// Função de Leitura da Tensão Vbus
//=======================================

float read_vbus_voltage(void)
{
    // Ler tensão RMS do sensor Vbus usando ADC contínuo
    float adc_voltage = read_adc_channel(VBUS_SENSE_PIN);

    // Calcular tensão real considerando o divisor de tensão
    float vbus_voltage = adc_voltage / VBUS_DIVIDER_RATIO;

    return vbus_voltage;
}

//=======================================
// Função de Atualização dos Sensores
//========================================

void update_sensors(void)
{
    // Ler sensores de corrente
    sensors.current_u_in = calculate_current_rms(read_adc_channel(SCT013_U_IN_PIN));
    sensors.current_u_out = calculate_current_rms(read_adc_channel(SCT013_U_OUT_PIN));
    sensors.current_v_in = calculate_current_rms(read_adc_channel(SCT013_V_IN_PIN));
    sensors.current_v_out = calculate_current_rms(read_adc_channel(SCT013_V_OUT_PIN));
    sensors.current_w_in = calculate_current_rms(read_adc_channel(SCT013_W_IN_PIN));
    sensors.current_w_out = calculate_current_rms(read_adc_channel(SCT013_W_OUT_PIN));

    // Ler sensores de tensão
    sensors.voltage_u = calculate_voltage_rms(read_adc_channel(ZMPT_U_PIN));
    sensors.voltage_v = calculate_voltage_rms(read_adc_channel(ZMPT_V_PIN));
    sensors.voltage_w = calculate_voltage_rms(read_adc_channel(ZMPT_W_PIN));

    // Ler tensão do barramento Vbus
    sensors.vbus_voltage = read_vbus_voltage();

    // Ler status VFO
    sensors.vfo_status = digitalRead(VFO_PIN);
}

//======================================
// Função de Verificação de Segurança
//======================================

void check_safety_conditions(void)
{
    // Verificar condições de sobrecorrente
    if (sensors.current_u_in > MAX_CURRENT_AMPS || sensors.current_u_out > MAX_CURRENT_AMPS ||
        sensors.current_v_in > MAX_CURRENT_AMPS || sensors.current_v_out > MAX_CURRENT_AMPS ||
        sensors.current_w_in > MAX_CURRENT_AMPS || sensors.current_w_out > MAX_CURRENT_AMPS)
    {

        if (!overcurrent_trip)
        {
            overcurrent_trip = true;
            emergency_stop("SOBRECORRENTE DETECTADA");
        }
    }

    // Verificar tensão do barramento Vbus
    if (sensors.vbus_voltage > MAX_VBUS_VOLTAGE)
    {
        if (!overcurrent_trip) // Usar mesmo flag para simplificar
        {
            overcurrent_trip = true;
            emergency_stop("SOBRETENSAO VBUS DETECTADA");
        }
    }

    // Verificar status VFO (LOW = falha)
    if (!sensors.vfo_status)
    {
        if (!vfo_trip)
        {
            vfo_trip = true;
            emergency_stop("FALHA VFO DETECTADA");
        }
    }
}

//=================================
// Função de Parada de Emergência
//=================================

void emergency_stop(const char *reason)
{
    disable_spwm();
    system_fault = true;

    Serial.println("=== PARADA DE EMERGÊNCIA ===");
    Serial.printf("Motivo: %s\n", reason);
    Serial.println("Sistema DESABILITADO - Envie 'reset' para limpar falhas");
    Serial.println("=============================");
}

//====================================
// Função de Definição do Duty Cycle
//====================================

void set_duty_cycle(mcpwm_cmpr_handle_t comparator, float duty_percent)
{
    duty_percent = constrain(duty_percent, 0.0, 1.0);
    uint32_t duty_ticks = (uint32_t)(duty_percent * PWM_PERIOD_TICKS);

    if (duty_ticks >= PWM_PERIOD_TICKS)
    {
        duty_ticks = PWM_PERIOD_TICKS - 1;
    }

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty_ticks));
}

//=============================
// Função de Atualização SPWM
//=============================

void update_spwm(void)
{
    if (!spwm_enabled || system_fault)
        return;

    // Calcular duty cycles senoidais para cada fase
    float duty_u = 0.5 + (modulation_index * 0.5 * sin(spwm_angle + PHASE_U_OFFSET));
    float duty_v = 0.5 + (modulation_index * 0.5 * sin(spwm_angle + PHASE_V_OFFSET));
    float duty_w = 0.5 + (modulation_index * 0.5 * sin(spwm_angle + PHASE_W_OFFSET));

    // Garantir que os duty cycles estejam dentro da faixa válida
    duty_u = constrain(duty_u, 0.05, 0.95);
    duty_v = constrain(duty_v, 0.05, 0.95);
    duty_w = constrain(duty_w, 0.05, 0.95);

    // Atualizar duty cycles PWM
    set_duty_cycle(comparator_u, duty_u);
    set_duty_cycle(comparator_v, duty_v);
    set_duty_cycle(comparator_w, duty_w);

    // Incrementar ângulo para próxima atualização
    spwm_angle += (2.0 * M_PI * MODULATION_FREQUENCY) / SPWM_UPDATE_FREQ;

    // Manter ângulo dentro de 0 a 2\pi
    if (spwm_angle >= 2.0 * M_PI)
    {
        spwm_angle -= 2.0 * M_PI;
    }
}

//===============================
// Habilitar/Desabilitar SPWM
//===============================

void enable_spwm(void)
{
    if (system_fault)
    {
        Serial.println("Não é possível iniciar - Falha de sistema ativa. Envie 'reset' primeiro.");
        return;
    }

    spwm_enabled = true;
    spwm_angle = 0.0;
    Serial.println("SPWM habilitado - Inversor iniciado");
}

void disable_spwm(void)
{
    spwm_enabled = false;
    // Definir todas as fases para 50% duty cycle
    set_duty_cycle(comparator_u, 0.5);
    set_duty_cycle(comparator_v, 0.5);
    set_duty_cycle(comparator_w, 0.5);
    Serial.println("SPWM desabilitado - Inversor parado");
}

//==============================
// Funções da Interface Serial
//==============================

void print_sensor_data(void)
{
    Serial.println("=== LEITURAS DOS SENSORES ===");
    Serial.printf("Correntes (A):\n");
    Serial.printf("  U_IN: %.2f  U_OUT: %.2f\n", sensors.current_u_in, sensors.current_u_out);
    Serial.printf("  V_IN: %.2f  V_OUT: %.2f\n", sensors.current_v_in, sensors.current_v_out);
    Serial.printf("  W_IN: %.2f  W_OUT: %.2f\n", sensors.current_w_in, sensors.current_w_out);
    Serial.printf("Tensões (V):\n");
    Serial.printf("  U: %.1f  V: %.1f  W: %.1f\n", sensors.voltage_u, sensors.voltage_v, sensors.voltage_w);
    Serial.printf("Tensão Vbus: %.1f V\n", sensors.vbus_voltage);
    Serial.printf("Status VFO: %s\n", sensors.vfo_status ? "OK" : "FALHA");
    Serial.printf("Índice de Modulação: %.3f\n", modulation_index);
    Serial.println("=============================");
}

void process_serial_commands(void)
{
    if (!Serial.available())
        return;

    String command = Serial.readString();
    command.trim();
    command.toLowerCase();

    if (command == "start")
    {
        enable_spwm();
    }
    else if (command == "stop")
    {
        disable_spwm();
    }
    else if (command == "status")
    {
        Serial.printf("=== STATUS DO SISTEMA ===\n");
        Serial.printf("SPWM: %s\n", spwm_enabled ? "EXECUTANDO" : "PARADO");
        Serial.printf("Falha de Sistema: %s\n", system_fault ? "SIM" : "NÃO");
        Serial.printf("Disparo Sobrecorrente: %s\n", overcurrent_trip ? "SIM" : "NÃO");
        Serial.printf("Disparo VFO: %s\n", vfo_trip ? "SIM" : "NÃO");
        Serial.printf("Ângulo Atual: %.2f rad (%.1f graus)\n", spwm_angle, spwm_angle * 180.0 / M_PI);
        Serial.printf("Índice de Modulação: %.3f\n", modulation_index);
        Serial.printf("Frequência de Saída: %d Hz\n", MODULATION_FREQUENCY);
        Serial.println("========================");
    }
    else if (command == "sensors")
    {
        print_sensor_data();
    }
    else if (command == "reset")
    {
        // Resetar todas as condições de falha
        overcurrent_trip = false;
        vfo_trip = false;
        system_fault = false;
        Serial.println("Falhas do sistema limpas - Pronto para operação");
    }
    else if (command.startsWith("set_mod"))
    {
        // Analisar valor do índice de modulação
        int space_index = command.indexOf(' ');
        if (space_index > 0)
        {
            float new_mod_index = command.substring(space_index + 1).toFloat();
            if (new_mod_index >= 0.1 && new_mod_index <= 1.0)
            {
                modulation_index = new_mod_index;
                Serial.printf("Índice de modulação definido para %.3f\n", new_mod_index);
            }
            else
            {
                Serial.printf("Índice de modulação inválido. Faixa: 0.1 - 1.0\n");
            }
        }
        else
        {
            Serial.println("Uso: set_mod <valor>");
            Serial.printf("Exemplo: set_mod 0.8\n");
        }
    }
    else if (command == "help")
    {
        Serial.println("=== COMANDOS DISPONÍVEIS ===");
        Serial.println("start              - Iniciar o inversor");
        Serial.println("stop               - Parar o inversor");
        Serial.println("status             - Mostrar status do sistema");
        Serial.println("sensors            - Mostrar leituras dos sensores");
        Serial.println("reset              - Limpar falhas do sistema");
        Serial.println("set_mod <valor>    - Definir índice de modulação (0.1-1.0)");
        Serial.println("help               - Mostrar esta ajuda");
        Serial.println("============================");
    }
    else
    {
        Serial.println("Comando desconhecido. Digite 'help' para comandos disponíveis.");
    }
}

//==================
// Loop Principal
//==================

void loop()
{
    unsigned long current_time = millis();

    // Atualizar SPWM se habilitado
    if (current_time - last_spwm_update >= SPWM_UPDATE_PERIOD_MS)
    {
        update_spwm();
        last_spwm_update = current_time;
    }

    // Atualizar sensores e verificar segurança
    if (current_time - last_sensor_update >= SENSOR_UPDATE_MS)
    {
        update_sensors();
        check_safety_conditions();
        last_sensor_update = current_time;
    }

    // Processar comandos seriais
    process_serial_commands();

    // Pequeno delay para não sobrecarregar o processador
    delay(1);
}