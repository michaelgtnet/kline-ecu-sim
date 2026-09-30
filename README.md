# kline-ecu-sim 🚗⚡

> **High-Performance, Standalone Automotive K-Line ECU Simulator for ISO 9141-2, ISO 14230 (KWP2000) and VW KWP1281**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Rust: 2021](https://img.shields.io/badge/Rust-2021_Edition-orange.svg)](https://www.rust-lang.org)
[![Automotive: K-Line](https://img.shields.io/badge/OBD2-K--Line%20%7C%20KWP2000-green.svg)]()

---

## 1. O que é o `kline-ecu-sim`?

O **`kline-ecu-sim`** é um simulador de ECU automotiva em tempo real desenvolvido em Rust nativo para rodar diretamente em computadores de placa única (SBCs como **Orange Pi One**, Raspberry Pi) ou no PC via cabo USB-Serial / KKL.

Ele se conecta diretamente ao transceptor físico da Linha K (**ST L9613**, **ST L9637D**, **TI SN65HVDA195-Q1**) através da porta serial de hardware (ex: `/dev/ttyS3`) e responde a qualquer scanner automotivo profissional (**ThinkDiag**, **Autel**, **VCDS**, **Launch**, **Rasther** ou **ELM327**) como se fosse uma injeção eletrônica real de um veículo.

---

## 2. Recursos Principais

* **Suporte Completo a Slow Init (5-Baud):**
  * Detecta o byte de endereço em 5 bauds (`0x01` para motor, `0x33` para OBD2 padrão).
  * Emite o byte de sincronismo `0x55` e os KeyBytes (`KB1`, `KB2`) a 10.400 baud.
  * Valida o handshake $W4$ ($\sim\text{KB2}$) e emite o endereço invertido ($\sim\text{Address}$) para abrir a sessão de diagnóstico.
* **Suporte a Fast Init (ISO 14230):**
  * Detecta o frame `StartCommunication` ($81$) e responde positivamente com $C1$ e KeyBytes.
* **Perfis de Centrais Mapeadas (`--profile`):**
  * `bosch-me75`: Injeção **Bosch Motronic ME7.5** (VW/Audi 1.8T, 2.0, 1.6 - Golf, Bora, Passat, A3).
  * `vag-kwp1281`: Protocolo clássico VAG **KWP1281** (painel de instrumentos e injeções antigas).
  * `generic-obd`: Padrão universal **SAE J1979 / OBD-II** (compatível com qualquer scanner genérico).
  * `marelli-iaw`: Centrais **Magneti Marelli IAW** (Fiat / VW).
* **Parâmetros em Tempo Real (Live Data - Modo 01):**
  * **RPM com Varredura Automática (`--rpm-sweep`):** O RPM sobe e desce dinamicamente (850 $\leftrightarrow$ 3200 RPM) para você ver o ponteiro ou gráfico do seu scanner se movendo ao vivo na bancada!
  * **Temperatura do Motor (`--coolant-temp`):** Padrão 90°C.
  * **Velocidade (`--speed`):** Configurável em km/h.
  * **Tensão da Central:** Reporta tensão real no PID 42.
* **Diagnóstico de Falhas (Modos 03 e 04):**
  * Retorna códigos de falha ativos (DTCs como `P0300`, `P0171`).
  * **Aceita o comando de Apagar Falhas (Mode 04):** Ao mandar limpar pelo scanner, a central limpa a memória e responde 0 falhas!
* **Identificação do Veículo (Modo 09):**
  * Retorna o número de chassi (VIN) codificado em múltiplos quadros ISO.
* **Proteção de Hardware Integrada (EchoGuard):**
  * Filtra automaticamente o eco elétrico local refletido pelos transceptores de fio único (L9613/L9637D), evitando que a central confunda a própria resposta com comandos do scanner.

---

## 3. Esquema de Ligação no Orange Pi One (com ST L9613 / L9637D)

O chip transceptor da Linha K é alimentado diretamente pelos **3.3V** e conectado na **UART3 (`/dev/ttyS3`)** do Orange Pi One:

```text
       LADO 12V (Carro / Scanner)                LADO 3.3V (Orange Pi One)
               
  FONTE 12V ───> Pino 7 (VS)            Pino 3 (VCC) <─── Pino 1 do Orange Pi (3.3V)
                    │                        │
               [RESISTOR]                    [RESISTOR PULL-UP]
              (1k a 4.7k)                    (4.7k a 10k)
                    │                        │
  LINHA K   ───> Pino 6 (K)             Pino 1 (RX)  ───> Pino 10 do Orange Pi (UART3_RX)
  DO SCANNER                                 │
  (Pino 7 OBD)                               ▼
                 Pino 4 (TX)  <────────────────────────── Pino 8 do Orange Pi (UART3_TX)
                    │
  TERRA 12V ───> Pino 5 (GND) ◄────────────────────────── Pino 6 do Orange Pi (GND COMUM)
```

---

## 4. Instalação e Execução

### Opção 1: Executar no Orange Pi One

1. Habilite a UART3 no Armbian (`/boot/armbianEnv.txt` adicionando `overlays=uart3`).
2. Rode o simulador na UART3:
```bash
# Simular uma Bosch ME7.5 (Golf/Audi 1.8T) com RPM oscilando na tela do scanner
kline-ecu-sim --device /dev/ttyS3 --profile bosch-me75 --rpm-sweep
```

### Opção 2: Simular com Parâmetros Customizados

```bash
kline-ecu-sim \
  --device /dev/ttyS3 \
  --baud 10400 \
  --profile bosch-me75 \
  --rpm 2500 \
  --coolant-temp 92 \
  --dtcs "P0300,P0171,P0420" \
  --vin "9BWCA05X12P123456"
```

---

## 5. Opções de Linha de Comando (CLI)

| Opção | Padrão | Descrição |
|---|---|---|
| `-d, --device <path>` | `/dev/ttyS3` | Porta serial UART do hardware |
| `-b, --baud <rate>` | `10400` | Taxa de transmissão (10400 ou 9600) |
| `-p, --profile <profile>` | `bosch-me75` | Perfil da central (`bosch-me75`, `vag-kwp1281`, `generic-obd`, `marelli-iaw`) |
| `--rpm <rpm>` | `850` | Rotação inicial do motor em RPM |
| `--rpm-sweep` | `false` | Varia o RPM automaticamente (850 a 3200) para testar ponteiros |
| `--coolant-temp <temp>` | `90` | Temperatura da água em °C |
| `--speed <kmh>` | `0` | Velocidade do veículo em km/h |
| `--dtcs <codes>` | `P0300,P0171` | Códigos de falha na memória da central |
| `--vin <vin>` | `9BWCA05X12P123456` | Chassi retornado no Modo 09 |

---

## 6. Como Testar com o Scanner

1. Conecte o cabo do seu scanner (**ThinkDiag**, **Autel**, **VCDS**, etc.) na tomada fêmea da bancada.
2. Inicie o `kline-ecu-sim`.
3. No aplicativo do scanner:
   * Escolha a montadora (ex: **Volkswagen** $\rightarrow$ **Golf/Bora** $\rightarrow$ **1.8T** $\rightarrow$ **Engine** ou **Generic OBD-II**).
   * Clique em **Ler Dados em Tempo Real (Live Data)** para ver o RPM e a temperatura.
   * Clique em **Ler Códigos de Falha** para ver `P0300` e `P0171`.
   * Clique em **Apagar Códigos de Falha** para ver a central confirmar a limpeza!

O terminal do `kline-ecu-sim` mostrará cada requisição em tempo real:
```text
⚡ [HANDSHAKE] RX 0x01 -> TX [0x55, 0x08, 0x08]
⚡ [HANDSHAKE] RX 0xF7 -> TX [0xFE]
✅ [SESSION ACTIVE] Diagnostic session fully opened with scanner!
📊 [REQ] Mode 01 PID 0C (Engine RPM) -> Responding 2400 RPM
🌡️ [REQ] Mode 01 PID 05 (Coolant Temp) -> Responding 90 °C
🔍 [REQ] Mode 03 (Read Trouble Codes) -> Responding DTCs
🧹 [REQ] Mode 04 (Clear Trouble Codes) -> DTCs Cleared!
```

---

## 7. Compilação e Cross-Compilação

### Compilação Nativa (Linux / SBC)
```bash
cargo build --release
```
Binário gerado: `target/release/kline-ecu-sim`

### Cross-Compilação Estática para Orange Pi One (ARMv7 32-bit musl)
```bash
cross build --target armv7-unknown-linux-musleabihf --release
```

---

## 8. Licença

Distribuído sob licença **MIT**. Consulte `LICENSE` para mais detalhes.
