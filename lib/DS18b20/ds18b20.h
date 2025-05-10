#ifndef DS18B20_H
#define DS18B20_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"

/// Inicializa o barramento 1-Wire no pino especificado (ex: GP2)
void ds18b20_init(uint pin);

/// Emite reset/presença. Retorna true se o sensor respondeu.
bool ds18b20_reset(void);

/// Dispara conversão e retorna temperatura em °C (resolução 12-bit).
float ds18b20_get_temperature(void);

#endif  // DS18B20_H
