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
#include "lib/Matriz_Bibliotecas/matriz_led.h"

// Pinos
#define PIN_DS18B20   20
#define PIN_VRY       26     // Joystick vertical → ADC0
#define PIN_BTN_A     5
#define PIN_PWM_OUT   11    // Pino PWM para motor/ventoinha
#define I2C_SDA       14
#define I2C_SCL       15

static ssd1306_t oled;
static float current_temp = 0.0f;
static int setpoint = 20;
static volatile uint16_t pwm_duty = 0;
static bool show_main_screen = true;
static uint32_t last_screen_switch_time = 0;
static float simulated_rpm = 300.0f;

// Task_Sensor: Lê DS18B20 a cada 1s e atualiza current_temp
void Task_Sensor(void *pv) {
    ds18b20_init(PIN_DS18B20);
    for (;;) {
        current_temp = ds18b20_get_temperature();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task_Input: Ajusta setpoint via joystick e confirma com botão A
void Task_Input(void *pv) {
    // Inicializa ADC
    adc_init();
    adc_gpio_init(PIN_VRY);
    adc_select_input(0);

    // Inicializa botão
    gpio_init(PIN_BTN_A);
    gpio_set_dir(PIN_BTN_A, GPIO_IN);
    gpio_pull_up(PIN_BTN_A);

    bool selecting = true;
    bool last_btn = false;
    int last_dir = 0;
    char buf[32];

    for (;;) {
        uint16_t raw = adc_read();
        bool btn_now = (gpio_get(PIN_BTN_A) == 0);

        // Direção do joystick
        int dir = (raw > 3000 ? 1 : (raw < 1000 ? -1 : 0));

        if (selecting) {
            // Ajusta setpoint
            if (dir == 1 && last_dir == 0 && setpoint < 30) setpoint++;
            if (dir == -1 && last_dir == 0 && setpoint > 10) setpoint--;

            // Confirma setpoint
            if (btn_now && !last_btn) selecting = false;

            // Desenha UI de ajuste
            ssd1306_fill(&oled, false);
            ssd1306_draw_string(&oled, "Ajuste Setpoint:", 0, 0, false);
            snprintf(buf, sizeof(buf), "   %2d C", setpoint);
            ssd1306_draw_string(&oled, buf, 0, 16, false);
            ssd1306_draw_string(&oled, "[A] Confirma", 0, 32, false);
            ssd1306_send_data(&oled);
        } else {
            // Volta ao ajuste
            if (btn_now && !last_btn) selecting = true;

            // Alterna telas a cada 5s
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            if (current_time - last_screen_switch_time > 5000) {
                show_main_screen = !show_main_screen;
                last_screen_switch_time = current_time;
            }

            if (show_main_screen) {
                // Desenha UI de monitoramento
                ssd1306_fill(&oled, false);
                snprintf(buf, sizeof(buf), "Temp: %4.1f C", current_temp);
                ssd1306_draw_string(&oled, buf, 0, 0, false);
                snprintf(buf, sizeof(buf), "Set:  %3d C", setpoint);
                ssd1306_draw_string(&oled, buf, 0, 16, false);
                snprintf(buf, sizeof(buf), "Erro: %4.1f C", setpoint - current_temp);
                ssd1306_draw_string(&oled, buf, 0, 32, false);
                snprintf(buf, sizeof(buf), "PWM: %5u", pwm_duty);
                ssd1306_draw_string(&oled, buf, 0, 48, false);
                ssd1306_send_data(&oled);

                // Calcula erro e arredonda
                float error = fabsf(setpoint - current_temp);
                int digit = (int)(error + 0.5);

                // Define cor com base no dígito
                uint32_t cor_on;
                switch (digit) {
                    case 0: cor_on = COR_BRANCO; break;
                    case 1: cor_on = COR_PRATA; break;
                    case 2: cor_on = COR_CINZA; break;
                    case 3: cor_on = COR_VIOLETA; break;
                    case 4: cor_on = COR_AZUL; break;
                    case 5: cor_on = COR_MARROM; break;
                    case 6: cor_on = COR_VERDE; break;
                    case 7: cor_on = COR_OURO; break;
                    case 8: cor_on = COR_LARANJA; break;
                    case 9: cor_on = COR_AMARELO; break;
                    default: cor_on = COR_OFF; break;
                }

                // Desenha na matriz de LED
                if (error > 9.6) {
                    matriz_draw_pattern(PAD_X, COR_VERMELHO);
                } else {
                    matriz_draw_number(digit, cor_on);
                }
            } else {
                // Desenha UI de RPM
                ssd1306_fill(&oled, false);
                snprintf(buf, sizeof(buf), "RPM: %4.0f", simulated_rpm);
                ssd1306_draw_string(&oled, buf, 0, 0, false);

                // Calcula barra de progresso
                float rpm_range = 2000.0 - 300.0;
                float rpm_position = (simulated_rpm - 300.0) / rpm_range;
                int bar_length = (int)(rpm_position * 128);

                // Desenha barra
                for (int i = 0; i < bar_length; i++) {
                    ssd1306_draw_line(&oled, i, 20, i, 25, true);
                }

                ssd1306_send_data(&oled);
            }
        }

        last_btn = btn_now;
        last_dir = dir;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task_Control: Algoritmo PI a cada 1s
void Task_Control(void *pv) {
    const float kp = 120.0f;
    const float ki = 120.0f / 15.0f;
    const float h = 1.0f;
    static float integral = 0.0f;

    // Configura PWM
    gpio_set_function(PIN_PWM_OUT, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_PWM_OUT);
    uint chan = pwm_gpio_to_channel(PIN_PWM_OUT);
    pwm_set_wrap(slice, 65535);
    pwm_set_chan_level(slice, chan, 0);
    pwm_set_enabled(slice, true);

    for (;;) {
        float error = current_temp - (float)setpoint;
        float P = kp * error;
        integral += ki * error * h;

        // Anti-windup
        if (integral > 4096.0f) integral = 4096.0f;
        if (integral < -4096.0f) integral = -4096.0f;

        float U = P + integral;
        int32_t duty = (int32_t)((U + 4096.0f) * (65535.0f / 8192.0f));
        if (duty < 0) duty = 0;
        if (duty > 65535) duty = 65535;

        // Simula RPM
        simulated_rpm = 300.0 + (2000.0 - 300.0) * (duty / 65535.0);

        // Aplica PWM
        pwm_set_chan_level(slice, chan, duty);
        pwm_duty = duty;

        vTaskDelay(pdMS_TO_TICKS((uint32_t)(h * 1000)));
    }
}

int main() {
    stdio_init_all();

    // Inicializa matriz de LED
    inicializar_matriz_led();

    // Inicializa I2C e OLED
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&oled, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&oled);

    // Cria tasks
    xTaskCreate(Task_Sensor, "Sensor", 256, NULL, 2, NULL);
    xTaskCreate(Task_Input, "Input", 512, NULL, 1, NULL);
    xTaskCreate(Task_Control, "Control", 512, NULL, 1, NULL);

    // Inicia scheduler
    vTaskStartScheduler();

    while (1) tight_loop_contents();
    return 0;
}