/*
 * Inversor trifásico de bancada — firmware v2
 * ESP32-S3 + FSBB30CH60C (Motion SPM 3)
 * Arduino-ESP32 core 3.x (ESP-IDF 5.1 ou mais novo). NÃO compila no core 2.x.
 * Autor: José Fonseca
 *
 * O que mudou em relação à v1, e por quê:
 *  - O seno é calculado na interrupção do próprio PWM (10 kHz, um ponto por período,
 *    PWM centralizado). A v1 atualizava a 1 kHz pelo loop() com delay(1): 20 degraus por
 *    ciclo, com tremor, e o loop inteiro travava 1 s a cada comando (Serial.readString()),
 *    deixando o motor com tensão CC aplicada nesse segundo.
 *  - Parar e falhar DESLIGAM AS SEIS CHAVES pelo "brake" do MCPWM (estágio de saída, depois
 *    do tempo morto). A v1 "parava" pondo 50% de razão cíclica: a ponte continuava chaveando,
 *    inclusive depois de uma falha, e já chaveava desde o boot, antes do "start".
 *  - O VFO do módulo derruba o PWM no HARDWARE (falha por GPIO do MCPWM), em nanossegundos.
 *  - Pré-carga dos capacitores de bootstrap antes de partir (chaves de baixo ligadas ~20 ms).
 *  - Rampa de frequência e V/f (com boost); frequência negativa inverte o sentido.
 *  - ADC: formato TYPE2 (o único que o ESP32-S3 aceita; com TYPE1 a v1 abortava no boot),
 *    sem o driver legado "driver/adc.h" (misturar os dois aborta no boot no core 3.x), e o
 *    RMS tira a média (o nível de 1,65 V dos sensores). Na v1 o RMS incluía esse nível:
 *    1,65 V / 0,066 = 25 A lidos com o motor parado -> "sobrecorrente" na hora.
 *  - Sensibilidade do SCT013 corrigida: 0,066 V/A era a do ACS712-30A. SCT013-030 = 1 V / 30 A.
 *
 * ⚠️ ISOLAÇÃO: no esquemático V1 o GND do ESP32 é o NEGATIVO DO BARRAMENTO (ponte retificadora
 * direto da rede). Com a placa ligada na tomada, o ESP32 e o cabo USB estão no potencial da
 * rede. NUNCA ligue o USB num PC aterrado com a placa na rede: use transformador isolador na
 * entrada, um isolador USB (ADuM3160 e afins) ou alimente o barramento por fonte CC isolada.
 *
 * Comandos (115200 baud, terminar com Enter):
 *   start            parte (pré-carga + rampa até a frequência alvo)
 *   stop             rampa até 0 Hz e desliga
 *   off              desliga na hora
 *   f <Hz>           frequência alvo (negativa = sentido inverso)
 *   rampa <Hz/s>     aceleração/desaceleração
 *   m <0-1>          índice de modulação fixo (desliga o V/f)
 *   vf               volta ao V/f
 *   status           estado e medidas
 *   mon <ms>         imprime uma linha de medidas a cada <ms> (0 desliga)
 *   reset            limpa a falha (só se o VFO já voltou)
 *   help
 */

#include <Arduino.h>
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ============================ Pinos (esquemático V1) ============================
// ⚠️ GPIO35/36/37 são da PSRAM octal nos módulos N8R8/N16R8 (o YD-ESP32-S3 N16R8 é um
// deles). Nesses módulos, religue UL e VFO a pinos livres (ex.: 38 e 39) e troque aqui.
#define PIN_UH 18
#define PIN_UL 35
#define PIN_VH 13
#define PIN_VL 21
#define PIN_WH 14
#define PIN_WL 17
#define PIN_VFO 36 // ativo em nível BAIXO (open-collector do módulo)

// ============================ PWM ============================
#define PWM_RES_HZ 10000000                     // 10 MHz -> 0,1 us por tick
#define PWM_FREQ_HZ 10000                       // portadora
#define PWM_PERIODO (PWM_RES_HZ / PWM_FREQ_HZ)  // 1000 ticks, contagem sobe-desce
#define PWM_PICO (PWM_PERIODO / 2)              // comparador de 0 a 500 => razão = cmp / 500
#define TEMPO_MORTO_TICKS 20                    // 2,0 us (AJUSTE: datasheet do FSBB)
#define CMP_MIN ((TEMPO_MORTO_TICKS + 10) / 2)  // pulso mínimo ~1 us depois do tempo morto
#define PRECARGA_MS 20                          // bootstrap: chaves de baixo ligadas antes de partir

// ============================ Controle ============================
#define F_MAX 100.0f      // Hz, limite do comando
#define F_NOMINAL 50.0f   // Hz em que o V/f chega a M_MAX
#define M_MAX 1.0f        // índice máximo (1,0 = seno sem sobremodulação)
#define BOOST 0.05f       // índice em 0 Hz no V/f (AJUSTE: motor parado sem torque? aumente)
#define RAMPA_PADRAO 10.0f // Hz/s

// ============================ Medidas ============================
// ADC1_CH0..CH9 = GPIO1..GPIO10, na ordem do esquemático. GPIO11/12 (L12/L23) são ADC2: fora.
#define N_CANAIS 10
#define ADC_TAXA_HZ 20000   // total -> 2 kHz por canal
#define JANELA 400          // amostras por canal = 200 ms (10 ciclos de 50 Hz)
#define CH_VBUS 9

static const char *NOME[N_CANAIS] = {"I_U_AF", "I_U_DF", "I_V_AF", "I_V_DF", "I_W_AF",
                                     "I_W_DF", "V_U", "V_V", "V_W", "Vbus"};
// Ganho de cada canal (AJUSTE/CALIBRE com multímetro):
//  correntes: A por V  — SCT013-030 = 30 A / 1 V. (SCT013-005: 5.0)
//  tensões:   V por V  — depende do trimpot do ZMPT. 250 é só um ponto de partida.
//  Vbus:      (1,2 M + 10 k) / 10 k = 121
static float GANHO[N_CANAIS] = {30, 30, 30, 30, 30, 30, 250, 250, 250, 121};

// ============================ Proteções ============================
#define I_RMS_MAX 5.0f      // A rms, qualquer sensor (lento: janela de 200 ms)
#define I_PICO_MAX 12.0f    // A instantâneo (rápido: ~0,5 ms). O curto é com o CSC do módulo.
#define VBUS_MAX 360.0f     // V. O ADC satura perto de 375 V com este divisor.
#define VBUS_MIN 20.0f      // V, mínimo para partir
#define VBUS_FREIO 340.0f   // V: acima disso a desaceleração para (regeneração do motor)

// ============================ Estado ============================
static mcpwm_timer_handle_t timer;
static mcpwm_oper_handle_t oper[3];
static mcpwm_cmpr_handle_t cmpr[3];
static mcpwm_gen_handle_t gen_alto[3], gen_baixo[3];
static mcpwm_fault_handle_t falha_sw, falha_vfo;
static adc_continuous_handle_t adc;
static adc_cali_handle_t cali;

DRAM_ATTR static int16_t seno[1024];       // Q15
static volatile uint32_t fase = 0;         // acumulador de fase (2^32 = 1 volta)
static volatile int32_t passo_fase = 0;    // por período de PWM; negativo inverte o sentido
static volatile int32_t mod_q15 = 0;       // índice de modulação em Q15

static volatile float medida[N_CANAIS];    // rms AC (0-8) ou média (Vbus)
static volatile const char *motivo = nullptr;

enum Estado { PARADO, RODANDO, PARANDO, FALHA };
static Estado estado = PARADO;
static float freq = 0, alvo = 0, rampa = RAMPA_PADRAO, m_manual = 0.5f;
static bool vf = true;
static uint32_t mon_ms = 0;

// ============================ PWM ============================
static inline uint32_t IRAM_ATTR comparador(uint32_t f, int32_t m) {
  int32_t d = (m * seno[f >> 22]) >> 15;                        // m·sen, Q15
  int32_t c = PWM_PICO / 2 + ((d * (PWM_PICO / 2)) >> 15);
  if (c < CMP_MIN) c = CMP_MIN;
  if (c > PWM_PICO - CMP_MIN) c = PWM_PICO - CMP_MIN;
  return c;
}

// Roda a cada período (contador em zero); o valor novo entra no zero seguinte.
static bool IRAM_ATTR isr_pwm(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t *, void *) {
  uint32_t f = fase + (uint32_t)passo_fase;
  fase = f;
  int32_t m = mod_q15;
  mcpwm_comparator_set_compare_value(cmpr[0], comparador(f, m));
  mcpwm_comparator_set_compare_value(cmpr[1], comparador(f - 0x55555555u, m)); // -120°
  mcpwm_comparator_set_compare_value(cmpr[2], comparador(f - 0xAAAAAAAAu, m)); // -240°
  return false;
}

static void disparar(const char *porque) {
  if (!motivo) motivo = porque;
  mcpwm_soft_fault_activate(falha_sw); // brake one-shot: as seis saídas em BAIXO
}

static void setup_pwm() {
  for (int i = 0; i < 1024; i++) seno[i] = (int16_t)lroundf(32767.0f * sinf(i * 2.0f * PI / 1024));

  mcpwm_timer_config_t tc = {};
  tc.group_id = 0;
  tc.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  tc.resolution_hz = PWM_RES_HZ;
  tc.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN; // PWM centralizado
  tc.period_ticks = PWM_PERIODO;
  ESP_ERROR_CHECK(mcpwm_new_timer(&tc, &timer));

  mcpwm_soft_fault_config_t sfc = {};
  ESP_ERROR_CHECK(mcpwm_new_soft_fault(&sfc, &falha_sw));
  mcpwm_gpio_fault_config_t gfc = {};
  gfc.group_id = 0;
  gfc.gpio_num = PIN_VFO;
  gfc.flags.active_level = 0;
  gfc.flags.pull_up = true;
  ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&gfc, &falha_vfo));

  const int alto[3] = {PIN_UH, PIN_VH, PIN_WH}, baixo[3] = {PIN_UL, PIN_VL, PIN_WL};
  for (int i = 0; i < 3; i++) {
    mcpwm_operator_config_t oc = {};
    oc.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oc, &oper[i]));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper[i], timer));

    mcpwm_comparator_config_t cc = {};
    cc.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper[i], &cc, &cmpr[i]));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr[i], PWM_PICO / 2));

    mcpwm_generator_config_t gc = {};
    gc.gen_gpio_num = alto[i];
    ESP_ERROR_CHECK(mcpwm_new_generator(oper[i], &gc, &gen_alto[i]));
    gc.gen_gpio_num = baixo[i];
    ESP_ERROR_CHECK(mcpwm_new_generator(oper[i], &gc, &gen_baixo[i]));

    // Alto enquanto o contador está abaixo do comparador: razão = cmp / PWM_PICO.
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_alto[i],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr[i], MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_alto[i],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, cmpr[i], MCPWM_GEN_ACTION_HIGH)));

    // Baixo = alto invertido, com tempo morto nas duas bordas.
    mcpwm_dead_time_config_t dt = {};
    dt.posedge_delay_ticks = TEMPO_MORTO_TICKS;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_alto[i], gen_alto[i], &dt));
    dt = {};
    dt.negedge_delay_ticks = TEMPO_MORTO_TICKS;
    dt.flags.invert_output = true;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_alto[i], gen_baixo[i], &dt));

    // Falha (software ou VFO) = one-shot: as duas saídas em BAIXO até o reset.
    mcpwm_brake_config_t bc = {};
    bc.brake_mode = MCPWM_OPER_BRAKE_MODE_OST;
    bc.fault = falha_sw;
    ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(oper[i], &bc));
    bc.fault = falha_vfo;
    ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(oper[i], &bc));
    for (mcpwm_gen_handle_t g : {gen_alto[i], gen_baixo[i]}) {
      ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(g,
          MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));
      ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(g,
          MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));
    }
  }

  mcpwm_timer_event_callbacks_t cbs = {};
  cbs.on_empty = isr_pwm;
  ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer, &cbs, nullptr));
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
  mcpwm_soft_fault_activate(falha_sw); // nasce desligado
}

// Pré-carga do bootstrap e liberação das saídas. Devolve false se o VFO ainda está em falha.
static bool ligar_saidas() {
  for (int i = 0; i < 3; i++) mcpwm_generator_set_force_level(gen_alto[i], 0, true); // alto 0 => baixo 1
  for (int i = 0; i < 3; i++) {
    if (mcpwm_operator_recover_from_fault(oper[i], falha_vfo) != ESP_OK) return false;
    mcpwm_operator_recover_from_fault(oper[i], falha_sw);
  }
  delay(PRECARGA_MS);
  fase = 0;
  for (int i = 0; i < 3; i++) mcpwm_generator_set_force_level(gen_alto[i], -1, true);
  return true;
}

// ============================ ADC ============================
static void tarefa_adc(void *) {
  static uint8_t buf[1024];
  float soma[N_CANAIS] = {}, soma2[N_CANAIS] = {}, media[N_CANAIS];
  uint16_t n[N_CANAIS] = {};
  for (int i = 0; i < N_CANAIS; i++) media[i] = NAN; // sem média ainda: nada de pico
  for (;;) {
    uint32_t lidos = 0;
    if (adc_continuous_read(adc, buf, sizeof buf, &lidos, 1000) != ESP_OK) continue;
    for (uint32_t k = 0; k + SOC_ADC_DIGI_RESULT_BYTES <= lidos; k += SOC_ADC_DIGI_RESULT_BYTES) {
      auto *p = (adc_digi_output_data_t *)&buf[k];
      uint32_t ch = p->type2.channel;
      if (p->type2.unit != 0 || ch >= N_CANAIS) continue;
      int mv = 0;
      adc_cali_raw_to_voltage(cali, p->type2.data, &mv);
      float v = mv * 0.001f;

      if (ch < 6 && fabsf(v - media[ch]) * GANHO[ch] > I_PICO_MAX) disparar("pico de corrente");

      soma[ch] += v;
      soma2[ch] += v * v;
      if (++n[ch] == JANELA) {
        float mu = soma[ch] / JANELA, var = soma2[ch] / JANELA - mu * mu;
        media[ch] = mu;
        medida[ch] = GANHO[ch] * (ch == CH_VBUS ? mu : sqrtf(var > 0 ? var : 0));
        soma[ch] = soma2[ch] = 0;
        n[ch] = 0;
      }
    }
  }
}

static void setup_adc() {
  adc_continuous_handle_cfg_t hc = {};
  hc.max_store_buf_size = 4096;
  hc.conv_frame_size = 1024;
  ESP_ERROR_CHECK(adc_continuous_new_handle(&hc, &adc));

  adc_digi_pattern_config_t pat[N_CANAIS] = {};
  for (int i = 0; i < N_CANAIS; i++) {
    pat[i].atten = ADC_ATTEN_DB_12;
    pat[i].channel = i;
    pat[i].unit = ADC_UNIT_1;
    pat[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  }
  adc_continuous_config_t cfg = {};
  cfg.pattern_num = N_CANAIS;
  cfg.adc_pattern = pat;
  cfg.sample_freq_hz = ADC_TAXA_HZ;
  cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2; // o ESP32-S3 só tem o TYPE2
  ESP_ERROR_CHECK(adc_continuous_config(adc, &cfg));

  adc_cali_curve_fitting_config_t cc = {};
  cc.unit_id = ADC_UNIT_1;
  cc.atten = ADC_ATTEN_DB_12;
  cc.bitwidth = ADC_BITWIDTH_12;
  ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cc, &cali));

  xTaskCreatePinnedToCore(tarefa_adc, "adc", 4096, nullptr, 5, nullptr, 0);
  ESP_ERROR_CHECK(adc_continuous_start(adc));
}

// ============================ Controle (1 kHz, no loop) ============================
static void parar_agora() {
  mcpwm_soft_fault_activate(falha_sw);
  passo_fase = 0;
  mod_q15 = 0;
  freq = 0;
}

static void controle(float dt) {
  // Falhas
  if (estado == RODANDO || estado == PARANDO) {
    if (digitalRead(PIN_VFO) == LOW) disparar("VFO do modulo (curto no shunt ou subtensao 15 V)");
    for (int i = 0; i < 6; i++)
      if (medida[i] > I_RMS_MAX) disparar("sobrecorrente (rms)");
  }
  if (medida[CH_VBUS] > VBUS_MAX) disparar("sobretensao no barramento");
  if (motivo && estado != FALHA) {
    parar_agora();
    estado = FALHA;
    Serial.printf("\n*** FALHA: %s. Saidas desligadas. Use 'reset'.\n", (const char *)motivo);
  }
  if (estado != RODANDO && estado != PARANDO) return;

  // Rampa (não desacelera com o barramento alto: o motor está devolvendo energia)
  float objetivo = estado == PARANDO ? 0 : alvo;
  float d = objetivo - freq, max_d = rampa * dt;
  bool desacelerando = fabsf(objetivo) < fabsf(freq) || objetivo * freq < 0;
  if (!(desacelerando && medida[CH_VBUS] > VBUS_FREIO)) freq += constrain(d, -max_d, max_d);

  if (estado == PARANDO && freq == 0) {
    parar_agora();
    estado = PARADO;
    Serial.println("Parado.");
    return;
  }

  float m = vf ? constrain(BOOST + (1 - BOOST) * fabsf(freq) / F_NOMINAL, 0.0f, 1.0f) * M_MAX : m_manual;
  passo_fase = (int32_t)lroundf(freq * 4294967296.0f / PWM_FREQ_HZ);
  mod_q15 = (int32_t)(m * 32767);
}

// ============================ Serial ============================
static void imprimir_status() {
  const char *ESTADOS[] = {"PARADO", "RODANDO", "PARANDO", "FALHA"};
  Serial.printf("Estado %s | f %.2f Hz (alvo %.2f, rampa %.1f Hz/s) | m %.3f %s | VFO %s\n",
                ESTADOS[estado], freq, alvo, rampa, mod_q15 / 32767.0f, vf ? "(V/f)" : "(fixo)",
                digitalRead(PIN_VFO) ? "ok" : "FALHA");
  for (int i = 0; i < N_CANAIS; i++)
    Serial.printf("%s %.2f%s", NOME[i], medida[i], i < 6 ? " A  " : " V  ");
  Serial.println();
}

static void comando(String c) {
  c.trim();
  c.toLowerCase();
  int sp = c.indexOf(' ');
  String cmd = sp < 0 ? c : c.substring(0, sp);
  float arg = sp < 0 ? NAN : c.substring(sp + 1).toFloat();

  if (cmd == "start") {
    if (estado == FALHA) Serial.println("Em falha. Use 'reset'.");
    else if (estado == RODANDO) Serial.println("Ja esta rodando.");
    else if (medida[CH_VBUS] < VBUS_MIN) Serial.printf("Barramento baixo (%.1f V).\n", medida[CH_VBUS]);
    else if (estado == PARANDO) estado = RODANDO;
    else if (!ligar_saidas()) Serial.println("VFO em falha: nao partiu.");
    else {
      freq = 0;
      estado = RODANDO;
      Serial.printf("Partiu. Alvo %.2f Hz%s\n", alvo, alvo == 0 ? " (use 'f <Hz>')" : "");
    }
  } else if (cmd == "stop") {
    if (estado == RODANDO) estado = PARANDO;
  } else if (cmd == "off") {
    parar_agora();
    if (estado != FALHA) estado = PARADO;
    Serial.println("Desligado.");
  } else if (cmd == "f" && !isnan(arg)) {
    alvo = constrain(arg, -F_MAX, F_MAX);
  } else if (cmd == "rampa" && arg > 0) {
    rampa = arg;
  } else if (cmd == "m" && arg >= 0 && arg <= 1) {
    m_manual = arg;
    vf = false;
  } else if (cmd == "vf") {
    vf = true;
  } else if (cmd == "status") {
    imprimir_status();
  } else if (cmd == "mon" && !isnan(arg)) {
    mon_ms = (uint32_t)arg;
  } else if (cmd == "reset") {
    if (digitalRead(PIN_VFO) == LOW) Serial.println("VFO ainda em falha.");
    else {
      parar_agora();
      motivo = nullptr;
      estado = PARADO;
      Serial.println("Falha limpa.");
    }
  } else {
    Serial.println("start | stop | off | f <Hz> | rampa <Hz/s> | m <0-1> | vf | status | mon <ms> | reset");
  }
}

// ============================ Arduino ============================
void setup() {
  Serial.begin(115200);
  setup_pwm(); // primeiro: as saídas ficam desligadas desde já
  setup_adc();
  Serial.println("\nInversor v2 pronto. Saidas desligadas. 'help' para os comandos.");
}

void loop() {
  static String linha;
  static uint32_t t_ctrl = micros(), t_mon = 0;

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (linha.length()) comando(linha);
      linha = "";
    } else if (linha.length() < 64) {
      linha += ch;
    }
  }

  uint32_t agora = micros();
  if (agora - t_ctrl >= 1000) {
    controle((agora - t_ctrl) * 1e-6f);
    t_ctrl = agora;
  }

  if (mon_ms && millis() - t_mon >= mon_ms) {
    t_mon = millis();
    Serial.printf("%lu;%.2f;%.3f", (unsigned long)t_mon,freq, mod_q15 / 32767.0f);
    for (int i = 0; i < N_CANAIS; i++) Serial.printf(";%.2f", medida[i]);
    Serial.println();
  }
}
