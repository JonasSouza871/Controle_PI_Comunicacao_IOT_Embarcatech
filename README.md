# 🚦 **SemaTask RT** – Semáforo Inteligente com Display OLED & Matriz WS2812

## 📘 *Descrição Breve*  
 **SemaTask RT** é um firmware para o Raspberry Pi Pico (RP2040) que implementa um semáforo inteligente completo:  
> • Dois modos de operação (Normal / Noturno)  
> • Exibição gráfica em OLED 128×64 (SSD1306)  
> • Símbolos animados em matriz 5×5 de LEDs WS2812  
> • Sinalização sonora em buzzer PWM  
> • Arquitetura multitarefa com **FreeRTOS**.  

O projeto serve como base para laboratórios de sistemas embarcados, aulas de RTOS ou demonstrações de periféricos I²C, PIO + DMA e PWM.

---

## ✨ Funcionalidades Principais
| # | Recurso | Detalhes |
|---|---------|----------|
| 1 | **Modo Normal** | Sequência Verde → Amarelo → Vermelho com tempos configuráveis |
| 2 | **Modo Noturno** | Pisca amarelo em duty-cycle reduzido |
| 3 | **Buzzer Acessível** | Padrões de beep diferentes por cor, facilitando uso por PCD|
| 4 | **Display OLED** | Título, modo atual, cor ativa e mensagens contextuais |
| 5 | **Matriz 5 × 5** | Animação “✓”, “!” ou “✕” sincronizada com o semáforo |
| 6 | **Hot-Swap UF2** | Botão B reinicia diretamente no bootloader para atualização |
| 7 | **Código Modular** | Bibliotecas independentes (`ssd1306`, `matriz_led`) |
| 8 | **Adaptável** | Placeholders permitem portar para outros pinos / módulos |

---

## ⚙️ Hardware Necessário
- 🔴 **Raspberry Pi Pico / Pico W**  
- 🟢 **Matriz WS2812 (5 × 5)** – 25 LEDs endereçáveis  
- 🔵 **Display OLED SSD1306** – 128×64, I²C  
- 🟡 **LED RGB discreto** (ou 2 LEDs) nos pinos 11-13  
- 🟣 **Buzzer passivo** (40 Ω – 5 V) no pino 10  
- ⚫ **Botão de Modo** (GPIO 5)  
- ⚪ **Botão UF2** (GPIO 6)  
- 🔌 Fonte 5 V / USB-C  
- 🛠️ Cabos, solda e protoboard conforme necessidade  

> 💡 *Substitua por seus próprios componentes, tensões ou quantidades caso altere o projeto.*

---

## 🔌 Esquema de Conexões

### 1. OLED SSD1306
| Sinal | Pico GPIO | Observações |
|-------|-----------|-------------|
| SDA   | **14**    | PUX-UP interno habilitado |
| SCL   | **15**    | I²C1 @ 400 kHz |
| VCC   | 3V3(OUT)  | 3,3 V |
| GND   | GND       | — |

### 2. Matriz WS2812
| Sinal | Pico GPIO | Observações |
|-------|-----------|-------------|
| DIN   | **7**     | PIO0 / SM0 – 800 kHz |
| VCC   | 5 V ou 3V3| - |
| GND   | GND       | — |

### 3. LED RGB Discreto
| Cor | Pico GPIO | Resistor |
|-----|-----------|----------|
| Verde   | **11** | 330 Ω |
| Azul(*) | **12** | 330 Ω *(debug opcional)* |
| Vermelho| **13** | 330 Ω |

### 4. Buzzer PWM
| Sinal | Pico GPIO | PWM Slice |
|-------|-----------|-----------|
| +5 V  | **10**    | configurado no código |

### 5. Botões
| Função          | Pico GPIO | Pull-up |
|-----------------|-----------|---------|
| Botão A (Modo)  | **5**     | **Sim** |
| Botão B (UF2)   | **6**     | **Sim** |

---

## 💻 Configuração do Ambiente
1. **Pico-SDK <vX.Y.Z>**  
   ```bash
   git clone -b <versão> https://github.com/raspberrypi/pico-sdk
   export PICO_SDK_PATH=<path>


2. **FreeRTOS-Kernel** (fork RP2040)

   ```bash
   git clone https://github.com/FreeRTOS/FreeRTOS-Kernel
   ```
3. **Toolchain ARM-GNU** >= 13.2

   * `arm-none-eabi-gcc`, `cmake >= 3.13`, `ninja`
4. **VS Code + Extensão “Pico-CMake”** (opcional)
5. **Picotool / picoprobe** para flash e debug

> 📝 *Adapte caminhos `<…>` aos diretórios da sua máquina.*

---

## ▶️ Compilação e Upload


# 1. Crie diretório de build isolado
```bash
mkdir build && cd build
 ```

# 2. Gere arquivos via CMake
```bash
cmake -G "Ninja" ..
 ```
# 3. Compile
```bash
ninja
 ```
# 4. Conecte o Pico em BOOTSEL (ou use Botão B) e faça upload
cp semaforo.uf2 /media/<username>/RPI-RP2


> 🔄 *Para atualização OTA no Pico W, substitua o passo 4 pelo seu script de OTA.*

---

## 📁 Estrutura do Código

```
PicoSignaLED/
├─ lib/
│  ├─ Display_Bibliotecas/
│  │  ├─ font.h           # Bitmaps 8×8 e 5×5
│  │  ├─ ssd1306.c/.h     # Driver OLED I²C
│  ├─ Matriz_Bibliotecas/
│  │  ├─ matriz_led.c/.h  # Driver WS2812 via PIO
│  │  └─ generated/ws2812.pio.h
├─ FreeRTOSConfig.h       # Ajustado para RP2040
├─ main.c                 # Lógica de alto nível / tasks
├─ CMakeLists.txt         # Build script
└─ README.md              # Este arquivo ✍️
```

---

## 👉 Como Usar

1. **Ligue o Pico** - o modo inicial é **Normal**.
2. **Pressione Botão A** para alternar entre Normal ↔ Noturno.
3. **Observe**:

   * OLED exibe título, modo e cor atual.
   * Matriz WS2812 mostra animação (“✓” / “!” / “✕”).
   * Buzzer emite padrão de beeps condizente.
4. **Pressione Botão B** para entrar no bootloader UF2 e atualizar o firmware.


## 🐛 Debugging & Logs

* `printf()` via `stdio_usb` → visualize no *serial monitor* (baud irrelevante).
* Use `pico_enable_stdio_uart()` se precisar de UART em paralelo.

## 👤 Autor e Contato

* **Autor**: `Jonas Souza Pinto / CEPEDI`
* **E-mail / GitHub**: `<Jonassouza871@hotmail.com>` — *Contribuições são bem-vindas!*
