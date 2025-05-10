#include "ssd1306.h"
#include "font.h"
#include <stdlib.h>
#include "hardware/i2c.h"

// Inicializa o display SSD1306
void ssd1306_init(ssd1306_t *ssd, uint8_t width, uint8_t height, bool external_vcc, uint8_t address, i2c_inst_t *i2c) {
    ssd->width = width;
    ssd->height = height;
    ssd->pages = height / 8;
    ssd->address = address;
    ssd->i2c_port = i2c;
    ssd->bufsize = ssd->pages * ssd->width + 1;
    ssd->ram_buffer = calloc(ssd->bufsize, sizeof(uint8_t));
    if (ssd->ram_buffer != NULL) {
        ssd->ram_buffer[0] = 0x40; // Co = 0, D/C = 1 (Data Continuation)
    }
    ssd->port_buffer[0] = 0x80; // Prefixo de comando
}

// Configuração inicial do display
void ssd1306_config(ssd1306_t *ssd) {
    ssd1306_command(ssd, 0xAE); // Desliga display
    ssd1306_command(ssd, 0x20); // Modo de memória
    ssd1306_command(ssd, 0x00); // Modo de endereçamento horizontal
    ssd1306_command(ssd, 0x40); // Linha inicial
    ssd1306_command(ssd, 0xA1); // Remapeamento de segmentos
    ssd1306_command(ssd, 0xA8); // Razão de multiplexação
    ssd1306_command(ssd, ssd->height - 1);
    ssd1306_command(ssd, 0xC8); // Direção de varredura COM
    ssd1306_command(ssd, 0xD3); // Deslocamento do display
    ssd1306_command(ssd, 0x00);
    ssd1306_command(ssd, 0xDA); // Configuração de pinos COM
    ssd1306_command(ssd, 0x12);
    ssd1306_command(ssd, 0xD5); // Divisor de clock
    ssd1306_command(ssd, 0x80);
    ssd1306_command(ssd, 0xD9); // Período de pré-carga
    ssd1306_command(ssd, 0xF1);
    ssd1306_command(ssd, 0xDB); // Nível VCOMH
    ssd1306_command(ssd, 0x30);
    ssd1306_command(ssd, 0x81); // Controle de contraste
    ssd1306_command(ssd, 0xFF);
    ssd1306_command(ssd, 0xA4); // Display normal (não forçar todos pixels)
    ssd1306_command(ssd, 0xA6); // Sem inversão
    ssd1306_command(ssd, 0x8D); // Configuração da charge pump
    ssd1306_command(ssd, 0x14);
    ssd1306_command(ssd, 0xAF); // Liga display
}

// Envia um comando para o display
void ssd1306_command(ssd1306_t *ssd, uint8_t command) {
    ssd->port_buffer[1] = command;
    i2c_write_blocking(ssd->i2c_port, ssd->address, ssd->port_buffer, 2, false);
}

// Envia o buffer de dados para o display
void ssd1306_send_data(ssd1306_t *ssd) {
    ssd1306_command(ssd, 0x21); // Endereço de coluna
    ssd1306_command(ssd, 0);
    ssd1306_command(ssd, ssd->width - 1);
    ssd1306_command(ssd, 0x22); // Endereço de página
    ssd1306_command(ssd, 0);
    ssd1306_command(ssd, ssd->pages - 1);
    i2c_write_blocking(ssd->i2c_port, ssd->address, ssd->ram_buffer, ssd->bufsize, false);
}

// Desenha um pixel
void ssd1306_pixel(ssd1306_t *ssd, uint8_t x, uint8_t y, bool value) {
    uint16_t index = (y / 8) * ssd->width + x + 1;
    uint8_t pixel = y % 8;
    if (value) {
        ssd->ram_buffer[index] |= (1 << pixel);
    } else {
        ssd->ram_buffer[index] &= ~(1 << pixel);
    }
}

// Preenche a tela (tudo ligado ou desligado)
void ssd1306_fill(ssd1306_t *ssd, bool value) {
    for (uint8_t y = 0; y < ssd->height; ++y) {
        for (uint8_t x = 0; x < ssd->width; ++x) {
            ssd1306_pixel(ssd, x, y, value);
        }
    }
}

// Desenha números pequenos (5x5 pixels)
void ssd1306_draw_small_number(ssd1306_t *ssd, char c, uint8_t x, uint8_t y) {
    if (c >= '0' && c <= '9') {
        uint16_t index = ((c - '0') * 5) + (69 * 8); // Offset na fonte
        for (uint8_t i = 0; i < 5; ++i) {
            uint8_t line = font[index + i];
            for (uint8_t j = 0; j < 5; ++j) {
                if ((line >> (4 - j)) & 0x01) {
                    ssd1306_pixel(ssd, x + j, y + i, true);
                }
            }
        }
    }
}

// Desenha um caractere
void ssd1306_draw_char(ssd1306_t *ssd, char c, uint8_t x, uint8_t y, bool use_small_numbers) {
    if (use_small_numbers && c >= '0' && c <= '9') {
        ssd1306_draw_small_number(ssd, c, x, y);
        return;
    }

    uint16_t index = 0;
    bool rotate = false;

    // Mapeamento de caracteres
    if (c >= '0' && c <= '9') {
        index = (c - '0' + 1) * 8;
    } else if (c >= 'A' && c <= 'Z') {
        index = (c - 'A' + 11) * 8;
    } else if (c >= 'a' && c <= 'z') {
        index = (c - 'a' + 37) * 8;
    } else if (c == ':') {
        index = 64 * 8;
        rotate = true;
    } else if (c == '.') {
        index = 65 * 8;
        rotate = true;
    } else if (c == '>') {
        index = 66 * 8;
        rotate = true;
    } else if (c == '-') {
        index = 67 * 8;
        rotate = true;
    } else if (c == 127) {
        index = 68 * 8; // Símbolo Ohm (Ω)
    }
      else if (c == '!') {
        index = 69 * 8;  
        rotate = true;
    }

    // Renderização do caractere
    for (uint8_t i = 0; i < 8; ++i) {
        uint8_t line = font[index + i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (rotate) {
                ssd1306_pixel(ssd, x + (7 - j), y + i, (line >> j) & 0x01);
            } else {
                ssd1306_pixel(ssd, x + i, y + j, (line >> j) & 0x01);
            }
        }
    }
}

// Desenha uma string
void ssd1306_draw_string(ssd1306_t *ssd, const char *str, uint8_t x, uint8_t y, bool use_small_numbers) {
    while (*str) {
        char c = *str;
        uint8_t char_width = 8;

        if (use_small_numbers && c >= '0' && c <= '9') {
            char_width = 5;
        }

        // Quebra de linha automática
        if (x + char_width > ssd->width) {
            x = 0;
            y += 8;
            if (y + 8 > ssd->height) break;
        }

        ssd1306_draw_char(ssd, c, x, y, use_small_numbers);
        x += char_width;
        str++;
    }
}

// Desenha um retângulo (opcionalmente preenchido)
void ssd1306_rect(ssd1306_t *ssd, uint8_t top, uint8_t left, uint8_t width, uint8_t height, bool value, bool fill) {
    // Bordas
    for (uint8_t x = left; x < left + width; ++x) {
        ssd1306_pixel(ssd, x, top, value);
        ssd1306_pixel(ssd, x, top + height - 1, value);
    }
    for (uint8_t y = top; y < top + height; ++y) {
        ssd1306_pixel(ssd, left, y, value);
        ssd1306_pixel(ssd, left + width - 1, y, value);
    }
    // Preenchimento
    if (fill) {
        for (uint8_t x = left + 1; x < left + width - 1; ++x) {
            for (uint8_t y = top + 1; y < top + height - 1; ++y) {
                ssd1306_pixel(ssd, x, y, value);
            }
        }
    }
}

// Desenha uma linha (algoritmo de Bresenham)
void ssd1306_line(ssd1306_t *ssd, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool value) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (1) {
        ssd1306_pixel(ssd, x0, y0, value);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

// Linha horizontal
void ssd1306_hline(ssd1306_t *ssd, uint8_t x0, uint8_t x1, uint8_t y, bool value) {
    for (uint8_t x = x0; x <= x1; ++x) {
        ssd1306_pixel(ssd, x, y, value);
    }
}

// Linha vertical
void ssd1306_vline(ssd1306_t *ssd, uint8_t x, uint8_t y0, uint8_t y1, bool value) {
    for (uint8_t y = y0; y <= y1; ++y) {
        ssd1306_pixel(ssd, x, y, value);
    }
}
