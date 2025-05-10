// main.c
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "lib/DS18b20/ds18b20.h"
#include "lib/Display_Bibliotecas/ssd1306.h"

#define DS_PIN        17      // GP2 onde está o DS18B20
#define I2C_PORT      i2c1
#define SDA_PIN       14      // GP4 (ajuste conforme seu wiring)
#define SCL_PIN       15     // GP5 (ajuste conforme seu wiring)
#define SSD1306_ADDR  0x3C   // endereço I²C padrão

int main() {
    stdio_init_all();

    // --- 1. Inicializa sensor de temperatura ---
    ds18b20_init(DS_PIN);

    // --- 2. Inicializa I²C para o display ---
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // --- 3. Inicializa SSD1306 ---
    ssd1306_t disp;
    ssd1306_init(&disp, 128, 64, false, SSD1306_ADDR, I2C_PORT);
    ssd1306_config(&disp);
    ssd1306_fill(&disp, false);
    ssd1306_send_data(&disp);

    // --- 4. Loop principal ---
    while (true) {
        // tenta resetar e converter
        if (ds18b20_reset()) {
            float temp = ds18b20_get_temperature();

            // monta string com a temperatura
            char buf[16];
            snprintf(buf, sizeof(buf), "%.2f°C", temp);

            // limpa tela e desenha
            ssd1306_fill(&disp, false);
            ssd1306_draw_string(&disp, "Temp:",  0,  0, false);
            ssd1306_draw_string(&disp, buf,     0, 10, false);  // usa numerais pequenos
            ssd1306_send_data(&disp);
        } else {
            // sensor não encontrado
            ssd1306_fill(&disp, false);
            ssd1306_draw_string(&disp, "Sem sensor", 0, 0, false);
            ssd1306_send_data(&disp);
        }

        sleep_ms(1000);
    }

    return 0;
}
