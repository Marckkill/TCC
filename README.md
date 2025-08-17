# TCC - Sistema de Controle de Motor Trifásico com ESP32

Este repositório contém o projeto de TCC (Trabalho de Conclusão de Curso) que implementa um sistema de controle de motor trifásico utilizando ESP32 com modulação SPWM (Sinusoidal Pulse Width Modulation).

## 📋 Sobre o Projeto

O sistema desenvolvido é um inversor trifásico baseado no microcontrolador ESP32-S3, capaz de gerar sinais SPWM para acionamento de motores AC trifásicos. O projeto inclui monitoramento em tempo real de corrente e tensão, além de sistemas de proteção e segurança.

### Características Principais

- **Controle SPWM**: Modulação senoidal PWM trifásica com frequência configurável
- **Monitoramento em Tempo Real**: Leitura contínua de tensão e corrente RMS
- **Sistema de Proteção**: Detecção de sobrecorrente, sobretensão e falhas do sistema
- **Interface Serial**: Controle e monitoramento via comandos seriais
- **Hardware Personalizado**: PCB desenvolvida especificamente para o projeto

## 📁 Estrutura do Projeto

```
TCC/
├── 📄 Esquemáticos KiCad
│   ├── TCC.kicad_sch           # Esquemático principal
│   ├── filtro.kicad_sch        # Circuito de filtros
│   ├── fonte.kicad_sch         # Sistema de alimentação
│   ├── mcu.kicad_sch           # Microcontrolador ESP32
│   ├── sdf.kicad_sch           # Sistema de detecção de falhas
│   └── smp.kicad_sch           # Sistema de medição de potência
│
├── 🔧 Hardware
│   ├── TCC.kicad_pcb           # Layout da PCB
│   ├── TCC.kicad_pro           # Projeto KiCad
│   └── Componentes_Custom/     # Componentes personalizados
│       ├── ESP32/              # Footprint ESP32-S3
│       ├── FSBB30CH60/         # Módulo de potência
│       └── PJ320A/             # Conector de áudio
│
├── 💻 Firmware
│   └── Firmware.ino            # Código principal do ESP32
│
├── 📚 Documentação
│   ├── TCC.pdf                 # Documentação completa
│   ├── TCC-Fonte.pdf           # Esquemático da fonte
│   ├── TCC-MCU.pdf             # Esquemático do MCU
│   └── TCC-SMP.pdf             # Sistema de medição
│
└── 🔄 Backups
    └── TCC-backups/            # Backups automáticos do projeto
```

## 🔧 Hardware

### Componentes Principais

| Componente               | Modelo/Tipo | Função                         |
| ------------------------ | ----------- | ------------------------------ |
| **Microcontrolador**     | ESP32-S3    | Controle principal do sistema  |
| **Módulo de Potência**   | FSBB30CH60C | Inversor trifásico de potência |
| **Sensores de Corrente** | SCT013 (6x) | Monitoramento de corrente AC   |
| **Sensores de Tensão**   | ZMPT (3x)   | Monitoramento de tensão AC     |
| **Conector**             | PJ320A      | Interface de entrada de áudio  |

### Especificações Técnicas

- **Tensão de Entrada**: Barramento DC variável
- **Frequência de Saída**: 50 Hz (configurável)
- **Frequência PWM**: 10 kHz
- **Corrente Máxima**: 5A por fase (com proteção)
- **Tempo Morto**: 2μs entre chaves complementares
- **Taxa de Amostragem ADC**: 20 kHz

### Pinagem do ESP32-S3

#### Sinais PWM

```
Fase U: GPIO 18 (Alto), GPIO 35 (Baixo)
Fase V: GPIO 13 (Alto), GPIO 21 (Baixo)
Fase W: GPIO 14 (Alto), GPIO 17 (Baixo)
```

#### Sensores de Corrente (ADC1)

```
U_IN:  GPIO 1  (ADC1_CH0)    U_OUT: GPIO 2  (ADC1_CH1)
V_IN:  GPIO 3  (ADC1_CH2)    V_OUT: GPIO 4  (ADC1_CH3)
W_IN:  GPIO 5  (ADC1_CH4)    W_OUT: GPIO 6  (ADC1_CH5)
```

#### Sensores de Tensão (ADC1)

```
Tensão U: GPIO 7  (ADC1_CH6)
Tensão V: GPIO 8  (ADC1_CH7)
Tensão W: GPIO 9  (ADC1_CH8)
Vbus:     GPIO 10 (ADC1_CH9)
```

#### Segurança

```
VFO (Status): GPIO 36 (INPUT_PULLUP)
```

## 💻 Firmware

### Funcionalidades Implementadas

#### 🌊 Geração SPWM

- Modulação senoidal PWM trifásica
- Defasagem de 120° entre fases
- Índice de modulação configurável (0.1 - 1.0)
- Atualização a 1 kHz para suavidade do sinal

#### 📊 Monitoramento em Tempo Real

- **ADC Contínuo com DMA**: 20 kHz de amostragem
- **Cálculo RMS**: Janela deslizante de 400 amostras (20ms)
- **Buffer Circular**: Armazenamento eficiente de dados
- **Multitarefa**: Processamento paralelo com FreeRTOS

#### 🛡️ Sistema de Proteção

- **Sobrecorrente**: Trip em >5A por fase
- **Sobretensão**: Monitoramento do barramento DC
- **Status VFO**: Verificação do módulo de potência
- **Parada de Emergência**: Desabilitação imediata em falhas

#### 📡 Interface Serial

Comandos disponíveis:

```
start              - Iniciar o inversor
stop               - Parar o inversor
status             - Mostrar status do sistema
sensors            - Mostrar leituras dos sensores
reset              - Limpar falhas do sistema
set_mod <valor>    - Definir índice de modulação (0.1-1.0)
help               - Mostrar ajuda
```

### Principais Funções do Código

| Função                        | Descrição                              |
| ----------------------------- | -------------------------------------- |
| `setup_mcpwm()`               | Configuração do módulo MCPWM para SPWM |
| `setup_adc_continuous()`      | Inicialização do ADC contínuo com DMA  |
| `update_spwm()`               | Atualização dos duty cycles senoidais  |
| `calculate_rms_from_buffer()` | Cálculo RMS das amostras               |
| `check_safety_conditions()`   | Verificação das condições de segurança |
| `emergency_stop()`            | Parada de emergência do sistema        |

## 🚀 Como Usar

### Pré-requisitos

#### Software

- [Arduino IDE](https://www.arduino.cc/en/software) 2.0+
- [KiCad](https://www.kicad.org/) 7.0+ (para visualizar esquemáticos)
- Bibliotecas ESP32 (esp32 by Espressif Systems)

#### Hardware

- ESP32-S3 Development Board
- Módulo FSBB30CH60C
- Sensores SCT013 e ZMPT
- Fonte de alimentação adequada

### Instalação e Configuração

1. **Clone o repositório**:

   ```bash
   git clone https://github.com/usuario/TCC.git
   cd TCC
   ```

2. **Configure o Arduino IDE**:

   - Instale o pacote ESP32
   - Selecione "ESP32S3 Dev Module"
   - Configure a porta serial correta

3. **Compile e envie o firmware**:
   - Abra `Firmware/Firmware.ino`
   - Compile (Ctrl+R)
   - Envie para o ESP32 (Ctrl+U)

### Operação do Sistema

1. **Inicialização**:

   ```
   Sistema inicializado - Pronto para operação
   Comandos: start, stop, status, sensors, reset, set_mod <0.1-1.0>
   AVISO: Verifique todos os sistemas de segurança antes de iniciar!
   ```

2. **Verificar sensores**:

   ```
   sensors
   ```

3. **Iniciar o inversor**:

   ```
   start
   ```

4. **Monitorar operação**:
   ```
   status
   ```

### Exemplo de Uso

```bash
# Monitor serial a 115200 baud
sensors          # Verificar leituras
set_mod 0.5      # Configurar índice de modulação para 50%
start            # Iniciar operação
status           # Verificar status
stop             # Parar operação
```

## 📊 Monitoramento

### Dados Disponíveis

O sistema monitora continuamente:

- **Correntes RMS**: 6 canais (entrada e saída por fase)
- **Tensões RMS**: 3 fases + barramento DC
- **Status de Segurança**: VFO, sobrecorrente, sobretensão
- **Parâmetros SPWM**: Ângulo atual, índice de modulação

### Exemplo de Saída do Monitor

```
=== LEITURAS DOS SENSORES ===
Correntes (A):
  U_IN: 2.34  U_OUT: 2.31
  V_IN: 2.35  V_OUT: 2.33
  W_IN: 2.36  W_OUT: 2.34
Tensões (V):
  U: 230.1  V: 229.8  W: 230.3
Tensão Vbus: 325.4 V
Status VFO: OK
Índice de Modulação: 0.800
=============================
```

## 🛡️ Segurança

### Sistemas de Proteção Implementados

1. **Sobrecorrente**: Monitoramento contínuo com trip em 5A
2. **Sobretensão**: Proteção do barramento DC
3. **VFO**: Monitoramento do status do módulo de potência
4. **Tempo Morto**: Prevenção de curto-circuito entre chaves

### ⚠️ Avisos de Segurança

- **ALTA TENSÃO**: Este projeto trabalha com tensões perigosas
- **Sempre desligue a alimentação** antes de fazer conexões
- **Verifique todas as conexões** antes de energizar
- **Use EPIs adequados** durante testes
- **Tenha um sistema de parada de emergência** sempre acessível

## 🔄 Desenvolvimento

### Estrutura do Código

```cpp
// Configurações principais
#define PWM_FREQUENCY 10000        // 10 kHz
#define MODULATION_FREQUENCY 50    // 50 Hz
#define ADC_SAMPLE_FREQ_HZ 20000   // 20 kHz
#define MAX_CURRENT_AMPS 5.0       // 5A máximo

// Loop principal
void loop() {
    update_spwm();              // Atualiza SPWM (1 kHz)
    update_sensors();           // Lê sensores (100 Hz)
    check_safety_conditions();  // Verifica segurança
    process_serial_commands();  // Processa comandos
}
```

### Tecnologias Utilizadas

- **ESP-IDF**: Framework nativo do ESP32
- **FreeRTOS**: Sistema operacional em tempo real
- **MCPWM**: Módulo PWM dedicado do ESP32
- **ADC Contínuo**: Amostragem de alta velocidade com DMA
- **KiCad**: Design de circuitos e PCB

## 📈 Melhorias Futuras

- [ ] Controle de velocidade com feedback
- [ ] Interface web para controle remoto
- [ ] Controle de torque
- [ ] Algoritmos de controle avançados (FOC)
- [ ] Análise de espectro harmônico
- [ ] Comunicação CAN/Modbus
- [ ] Sistema de logs em cartão SD

## 📄 Documentação

### Arquivos de Documentação

- **[TCC.pdf](TCC.pdf)**: Documentação completa do projeto
- **[TCC-Fonte.pdf](TCC-Fonte.pdf)**: Detalhes do sistema de alimentação
- **[TCC-MCU.pdf](TCC-MCU.pdf)**: Circuito do microcontrolador
- **[TCC-SMP.pdf](TCC-SMP.pdf)**: Sistema de medição de potência

### Referências Técnicas

- [ESP32-S3 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [FSBB30CH60C Datasheet](https://www.onsemi.com/pdf/datasheet/fsbb30ch60c-d.pdf)
- Normas de segurança para inversores de frequência

## 👨‍💻 Autor

**José Fonseca**

- Projeto de TCC - Engenharia Elétrica
- Especialização em Eletrônica de Potência
- Controle de Motores AC

## 📝 Licença

Este projeto foi desenvolvido como Trabalho de Conclusão de Curso (TCC) e está disponível para fins educacionais e de pesquisa.

---

## 🆘 Suporte

Para dúvidas ou problemas:

1. Verifique a documentação técnica incluída
2. Consulte os comentários no código fonte
3. Revise as configurações de hardware
4. Verifique as conexões elétricas

**⚠️ LEMBRE-SE: Segurança sempre em primeiro lugar ao trabalhar com sistemas de potência!**
