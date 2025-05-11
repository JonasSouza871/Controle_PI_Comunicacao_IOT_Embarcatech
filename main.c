#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lib/DS18b20/ds18b20.h"
#include "lib/Display_Bibliotecas/ssd1306.h"

// Pinos
#define PIN_DS18B20   20
#define PIN_VRY       26     // ADC0 no GPIO26
#define PIN_BTN_A     5
#define I2C_SDA       14
#define I2C_SCL       15
#define LED_RGB_PIN   16     // Pino do LED RGB para saída PWM

// Variáveis do controlador PI
#define SAMPLING_PERIOD 0.5f  // tempo de amostragem em segundos
static float kp = 120.0f;     // ganho proporcional
static float ki = 8.0f;       // ganho integral (k/ti)
static float integral = 0.0f; // termo integral
static uint32_t pwm_duty = 0; // valor do PWM (0-1000)

// Variáveis compartilhadas
static ssd1306_t oled;
static float current_temp;
static int setpoint = 20;
static bool is_selecting = true;  // Modo de seleção ativo inicialmente

// Função para mapear valor de um intervalo para outro
static uint32_t map_value(float x, float in_min, float in_max, uint32_t out_min, uint32_t out_max) {
    if (x <= in_min) return out_min;
    if (x >= in_max) return out_max;
    return (uint32_t)((x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min);
}

// Task: lê temperatura a cada 1s
void Task_Sensor(void *pv) {
    ds18b20_init(PIN_DS18B20);
    while (1) {
        current_temp = ds18b20_get_temperature();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task: controlador PI para saída PWM
void Task_Controller(void *pv) {
    // Configuração do PWM
    gpio_set_function(LED_RGB_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LED_RGB_PIN);
    uint chan = pwm_gpio_to_channel(LED_RGB_PIN);
    
    // Configura PWM para 1kHz
    pwm_set_clkdiv(slice, 125.0f);
    pwm_set_wrap(slice, 1000);
    pwm_set_enabled(slice, true);
    
    while (1) {
        // Se estiver no modo de seleção, não calcula o controle
        if (!is_selecting) {
            // Calcula erro
            float error = setpoint - current_temp;
            
            // Ação proporcional
            float p_term = kp * error;
            
            // Ação integral
            integral += ki * error * SAMPLING_PERIOD;
            
            // Anti-windup
            if (integral > 4096.0f) integral = 4096.0f;
            else if (integral < -4096.0f) integral = -4096.0f;
            
            // Sinal de controle total
            float control = p_term + integral;
            
            // Mapeia para valor PWM (0-1000)
            pwm_duty = map_value(control, -4096.0f, 4096.0f, 0, 1000);
            
            // Atualiza saída PWM
            pwm_set_chan_level(slice, chan, pwm_duty);
        }
        
        vTaskDelay(pdMS_TO_TICKS((uint32_t)(SAMPLING_PERIOD * 1000)));
    }
}

// Task: ajusta setpoint com joystick e confirma com botão
void Task_Input(void *pv) {
    // inicializa ADC no canal 0 (GPIO26)
    adc_init();
    adc_gpio_init(PIN_VRY);
    adc_select_input(0);

    // configura botão A com pull-up
    gpio_init(PIN_BTN_A);
    gpio_set_dir(PIN_BTN_A, GPIO_IN);
    gpio_pull_up(PIN_BTN_A);

    bool last_btn = false;   // para detecção de borda do botão
    int last_dir = 0;       // -1, 0 ou +1 para joystick
    char buf[32];

    for (;;) {
        // lê ADC (0–4095) e botão
        uint16_t raw = adc_read();
        bool btn_now = (gpio_get(PIN_BTN_A) == 0);

        // determina direção atual do joystick
        int dir;
        if (raw > 3000)       dir =  1;  // pra cima
        else if (raw < 1000)  dir = -1;  // pra baixo
        else                  dir =  0;  // neutro

        if (is_selecting) {
            // só atua quando sair de 0 para +1 ou de 0 para -1
            if (dir == 1 && last_dir == 0 && setpoint < 30) {
                setpoint++;
            }
            else if (dir == -1 && last_dir == 0 && setpoint > 10) {
                setpoint--;
            }

            // confirma com botão A
            if (btn_now && !last_btn) {
                is_selecting = false;
                // Reinicia integral quando confirma novo setpoint
                integral = 0.0f;
            }

            // desenha tela de ajuste
            ssd1306_fill(&oled, false);
            ssd1306_draw_string(&oled, "Ajuste Setpoint:", 0, 0, false);
            snprintf(buf, sizeof(buf), "   %2d C", setpoint);
            ssd1306_draw_string(&oled, buf, 0, 16, false);
            ssd1306_draw_string(&oled, "[A] Confirma", 0, 32, false);
            ssd1306_send_data(&oled);
        }
        else {
            // volta ao ajuste se apertar A novamente
            if (btn_now && !last_btn) {
                is_selecting = true;
            }

            // desenha tela de monitoramento
            ssd1306_fill(&oled, false);
            snprintf(buf, sizeof(buf), "Temp: %4.1f C", current_temp);
            ssd1306_draw_string(&oled, buf, 0, 0, false);
            snprintf(buf, sizeof(buf), "Set:  %3d C", setpoint);
            ssd1306_draw_string(&oled, buf, 0, 16, false);
            snprintf(buf, sizeof(buf), "Erro: %4.1f C", setpoint - current_temp);
            ssd1306_draw_string(&oled, buf, 0, 32, false);
            snprintf(buf, sizeof(buf), "PWM:  %3u/1000", pwm_duty);
            ssd1306_draw_string(&oled, buf, 0, 48, false);
            ssd1306_send_data(&oled);
        }

        // atualiza estados para próxima iteração
        last_btn = btn_now;
        last_dir = dir;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main() {
    stdio_init_all();

    // inicializa I2C e OLED
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&oled, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&oled);

    // inicializa PWM
    pwm_config config = pwm_get_default_config();
    
    // cria tasks
    xTaskCreate(Task_Sensor, "Sensor", 256, NULL, 2, NULL);
    xTaskCreate(Task_Input, "Input", 512, NULL, 1, NULL);
    xTaskCreate(Task_Controller, "Controller", 512, NULL, 3, NULL);

    // inicia scheduler
    vTaskStartScheduler();

    // não deve chegar aqui
    while (1) tight_loop_contents();
    return 0;
}
