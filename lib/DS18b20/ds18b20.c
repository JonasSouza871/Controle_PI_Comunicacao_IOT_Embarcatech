#include "ds18b20.h"

static uint ds18b20_pin;  // o GPIO usado pelo sensor

// ————— Funções “privadas” —————
static void write_bit(int bit) {
    gpio_set_dir(ds18b20_pin, GPIO_OUT);
    gpio_put(ds18b20_pin, 0);
    sleep_us(bit ? 6 : 60);
    gpio_set_dir(ds18b20_pin, GPIO_IN);
    sleep_us(bit ? 64 : 10);
}

static int read_bit(void) {
    int v;
    gpio_set_dir(ds18b20_pin, GPIO_OUT);
    gpio_put(ds18b20_pin, 0);
    sleep_us(6);
    gpio_set_dir(ds18b20_pin, GPIO_IN);
    sleep_us(9);
    v = gpio_get(ds18b20_pin);
    sleep_us(55);
    return v;
}

static void write_byte(uint8_t b) {
    for (int i = 0; i < 8; i++)
        write_bit((b >> i) & 1);
}

static uint8_t read_byte(void) {
    uint8_t b = 0;
    for (int i = 0; i < 8; i++)
        if (read_bit())
            b |= 1 << i;
    return b;
}
// ————————————————————————

void ds18b20_init(uint pin) {
    ds18b20_pin = pin;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

bool ds18b20_reset(void) {
    gpio_set_dir(ds18b20_pin, GPIO_OUT);
    gpio_put(ds18b20_pin, 0);
    sleep_us(480);
    gpio_set_dir(ds18b20_pin, GPIO_IN);
    sleep_us(70);
    bool present = !gpio_get(ds18b20_pin);
    sleep_us(410);
    return present;
}

float ds18b20_get_temperature(void) {
    ds18b20_reset();
    write_byte(0xCC);  // Skip ROM
    write_byte(0x44);  // Convert T
    sleep_ms(750);

    ds18b20_reset();
    write_byte(0xCC);
    write_byte(0xBE);  // Read Scratchpad
    uint8_t lsb = read_byte();
    uint8_t msb = read_byte();
    int16_t raw = (msb << 8) | lsb;
    return raw * 0.0625f;
}
