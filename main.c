#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lib/DS18b20/ds18b20.h"
#include "lib/Display_Bibliotecas/ssd1306.h"
#include "lib/Matriz_Bibliotecas/matriz_led.h"  // Inclui o cabeçalho da matriz de LED

// pinos
#define PIN_DS18B20   20
#define PIN_VRY       26     // joystick vertical → ADC0
#define PIN_BTN_A     5
#define PIN_PWM_OUT   11    // pino PWM para motor/ventoinha
#define I2C_SDA       14
#define I2C_SCL       15

static ssd1306_t oled;
static float current_temp;
static int   setpoint = 20;
// guarda o duty gerado pelo PI para exibir
static volatile uint16_t pwm_duty = 0;

// ===== Task_Sensor =====
// Lê DS18B20 a cada 1s e atualiza current_temp
void Task_Sensor(void *pv) {
    ds18b20_init(PIN_DS18B20);
    for (;;) {
        current_temp = ds18b20_get_temperature();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ===== Task_Input =====
// Ajusta setpoint via joystick → ADC0 e confirma/retorna com botão A
void Task_Input(void *pv) {
    // ADC
    adc_init();
    adc_gpio_init(PIN_VRY);
    adc_select_input(0);

    // Botão
    gpio_init(PIN_BTN_A);
    gpio_set_dir(PIN_BTN_A, GPIO_IN);
    gpio_pull_up(PIN_BTN_A);

    bool selecting = true;
    bool last_btn   = false;
    int  last_dir   = 0;
    char buf[32];

    for (;;) {
        uint16_t raw     = adc_read();                   // 0–4095
        bool     btn_now = (gpio_get(PIN_BTN_A) == 0);   // 0=pressionado

        // direção joystick
        int dir = (raw > 3000 ?  1
               : (raw < 1000 ? -1 : 0));

        if (selecting) {
            // flanco 0→±1 para um passo único
            if (dir ==  1 && last_dir == 0 && setpoint < 30) setpoint++;
            if (dir == -1 && last_dir == 0 && setpoint > 10) setpoint--;

            // confirma setpoint
            if (btn_now && !last_btn) selecting = false;

            // desenha UI ajuste
            ssd1306_fill(&oled, false);
            ssd1306_draw_string(&oled, "Ajuste Setpoint:", 0,  0, false);
            snprintf(buf, sizeof(buf), "   %2d C", setpoint);
            ssd1306_draw_string(&oled, buf,              0, 16, false);
            ssd1306_draw_string(&oled, "[A] Confirma",      0, 32, false);
            ssd1306_send_data(&oled);
        }
        else {
            // volta ao ajuste se apertar A novamente
            if (btn_now && !last_btn) selecting = true;

            // desenha UI monitoramento
            ssd1306_fill(&oled, false);
            snprintf(buf, sizeof(buf), "Temp: %4.1f C", current_temp);
            ssd1306_draw_string(&oled, buf,               0,  0, false);
            snprintf(buf, sizeof(buf), "Set:  %3d C",    setpoint);
            ssd1306_draw_string(&oled, buf,               0, 16, false);
            snprintf(buf, sizeof(buf), "Erro: %4.1f C",  setpoint - current_temp);
            ssd1306_draw_string(&oled, buf,               0, 32, false);
            // aqui exibimos o PWM gerado pelo PI
            snprintf(buf, sizeof(buf), "PWM: %5u",       pwm_duty);
            ssd1306_draw_string(&oled, buf,               0, 48, false);
            ssd1306_send_data(&oled);

            // Calcula o erro e arredonda para o inteiro mais próximo
            float error = setpoint - current_temp;
            int digit = (int)(error + 0.5);  // Arredonda para o inteiro mais próximo

            // Garante que o dígito esteja entre 0 e 9
            if (digit < 0) digit = 0;
            if (digit > 9) digit = 9;

            // Seleciona o padrão correspondente ao dígito
            const uint8_t *digit_pattern;
            switch (digit) {
                case 0: digit_pattern = PAD_0; break;
                case 1: digit_pattern = PAD_1; break;
                case 2: digit_pattern = PAD_2; break;
                case 3: digit_pattern = PAD_3; break;
                case 4: digit_pattern = PAD_4; break;
                case 5: digit_pattern = PAD_5; break;
                case 6: digit_pattern = PAD_6; break;
                case 7: digit_pattern = PAD_7; break;
                case 8: digit_pattern = PAD_8; break;
                case 9: digit_pattern = PAD_9; break;
                default: digit_pattern = PAD_0; break;
            }

            // Desenha o padrão na matriz de LED
            matriz_draw_pattern(digit_pattern, COR_VERDE);
        }

        last_btn = btn_now;
        last_dir = dir;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ===== Task_Control =====
// Algoritmo PI rodando a cada 1s e gerando duty-cycle PWM
void Task_Control(void *pv) {
    const float kp = 120.0f;
    const float ki = 120.0f / 15.0f;  // ti = 15s
    const float h  = 1.0f;            // período de amostragem (s)
    static float integral = 0.0f;

    // configura PWM
    gpio_set_function(PIN_PWM_OUT, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_PWM_OUT);
    uint chan  = pwm_gpio_to_channel(PIN_PWM_OUT);
    pwm_set_wrap(slice, 65535);
    pwm_set_chan_level(slice, chan, 0);
    pwm_set_enabled(slice, true);

    for (;;) {
        // erro = setpoint – medido
        float error = (float)setpoint - current_temp;
        float P = kp * error;
        integral += ki * error * h;

        // anti-windup
        if (integral >  4096.0f) integral =  4096.0f;
        if (integral < -4096.0f) integral = -4096.0f;

        float U = P + integral;  // sinal contínuo

        // mapeia U∈[-4096,4096] → duty∈[0,65535]
        int32_t duty = (int32_t)((U + 4096.0f) * (65535.0f / 8192.0f));
        if (duty <     0)   duty = 0;
        if (duty > 65535)   duty = 65535;

        // aplica e salva para exibir
        pwm_set_chan_level(slice, chan, duty);
        pwm_duty = duty;

        vTaskDelay(pdMS_TO_TICKS((uint32_t)(h*1000)));
    }
}

int main() {
    stdio_init_all();

    // Inicializa a matriz de LED
    inicializar_matriz_led();

    // inicializa I2C e OLED
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&oled, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&oled);

    // cria tasks
    xTaskCreate(Task_Sensor,  "Sensor",  256, NULL, 2, NULL);
    xTaskCreate(Task_Input,   "Input",   512, NULL, 1, NULL);
    xTaskCreate(Task_Control, "Control", 512, NULL, 1, NULL);

    // inicia scheduler
    vTaskStartScheduler();

    while (1) tight_loop_contents();
    return 0;
}
