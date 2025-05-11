#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lib/DS18b20/ds18b20.h"
#include "lib/Display_Bibliotecas/ssd1306.h"

// pinos
#define PIN_DS18B20   20
#define PIN_VRY       26     // ADC0 no GPIO26
#define PIN_BTN_A     5
#define I2C_SDA       14
#define I2C_SCL       15

static ssd1306_t oled;
static float current_temp;
static int setpoint = 20;

// Task: lê temperatura a cada 1s
void Task_Sensor(void *pv) {
    ds18b20_init(PIN_DS18B20);
    while (1) {
        current_temp = ds18b20_get_temperature();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task: ajusta setpoint com joystick e confirma com botão
void Task_Input(void *pv) {
    // 1) inicializa ADC no canal 0 (GPIO26)
    adc_init();
    adc_gpio_init(PIN_VRY);
    adc_select_input(0);  // <-- ADC0, pois PIN_VRY = 26

    // 2) configura botão A com pull-up
    gpio_init(PIN_BTN_A);
    gpio_set_dir(PIN_BTN_A, GPIO_IN);
    gpio_pull_up(PIN_BTN_A);

    bool selecting = true;     // modo ajuste = true, modo monitor = false
    bool last_btn   = false;   // para detectar borda
    char buf[32];

    while (1) {
        // lê valor bruto do ADC (0–4095)
        uint16_t raw = adc_read();

        // lê botão (0 = pressionado, 1 = solto)
        bool btn_now = (gpio_get(PIN_BTN_A) == 0);

        if (selecting) {
            // joystick para ajustar setpoint
            if (raw > 3000 && setpoint < 30) {
                setpoint++;
                vTaskDelay(pdMS_TO_TICKS(200));  // debounce simplificado
            }
            else if (raw < 1000 && setpoint > 10) {
                setpoint--;
                vTaskDelay(pdMS_TO_TICKS(200));
            }

            // detecta pressionar do botão A para confirmar
            if (btn_now && !last_btn) {
                selecting = false;
            }

            // desenha tela de ajuste
            ssd1306_fill(&oled, false);
            ssd1306_draw_string(&oled, "Ajuste Setpoint:",  0,  0, false);
            snprintf(buf, sizeof(buf), "   %2d C", setpoint);
            ssd1306_draw_string(&oled, buf,               0, 16, false);
            ssd1306_draw_string(&oled, "[A] Confirma",      0, 32, false);
            // (opcional) mostrar raw pra debug:
            // snprintf(buf, sizeof(buf), "ADC: %4u", raw);
            // ssd1306_draw_string(&oled, buf, 0, 48, false);
            ssd1306_send_data(&oled);
        }
        else {
            // depois de confirmar, monitora temperatura
            // detecta pressionar do botão A para voltar ao ajuste
            if (btn_now && !last_btn) {
                selecting = true;
            }

            ssd1306_fill(&oled, false);
            snprintf(buf, sizeof(buf), "Temp: %4.1f C", current_temp);
            ssd1306_draw_string(&oled, buf, 0,  0, false);
            snprintf(buf, sizeof(buf), "Set:  %3d C",  setpoint);
            ssd1306_draw_string(&oled, buf, 0, 16, false);
            snprintf(buf, sizeof(buf), "Erro: %4.1f C", setpoint - current_temp);
            ssd1306_draw_string(&oled, buf, 0, 32, false);
            ssd1306_draw_string(&oled, "Press A ajustar", 0, 48, false);
            ssd1306_send_data(&oled);
        }

        last_btn = btn_now;
        vTaskDelay(pdMS_TO_TICKS(50));
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

    // cria tasks
    xTaskCreate(Task_Sensor, "Sensor", 256, NULL, 2, NULL);
    xTaskCreate(Task_Input,  "Input",  512, NULL, 1, NULL);

    // inicia scheduler
    vTaskStartScheduler();

    // não deve chegar aqui
    while (1) tight_loop_contents();
    return 0;
}
