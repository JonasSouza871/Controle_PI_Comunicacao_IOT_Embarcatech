#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

//Estrutura para configurações do display SSD1306
typedef struct {
    uint8_t width; //Largura do display em pixels
    uint8_t height; //Altura do display em pixels
    uint8_t pages; //Número de páginas (8 pixels por página)
    uint8_t address; //Endereço I2C do display
    i2c_inst_t *i2c_port; //Instância do barramento I2C
    uint16_t bufsize; //Tamanho do buffer de dados
    uint8_t *ram_buffer; //Buffer para dados do display
    uint8_t port_buffer[2]; //Buffer para comandos
} ssd1306_t;

//Funções básicas do display
void ssd1306_init(ssd1306_t *ssd, uint8_t width, uint8_t height, bool external_vcc, uint8_t address, i2c_inst_t *i2c); //Inicializa o display
void ssd1306_config(ssd1306_t *ssd); //Configura parâmetros iniciais
void ssd1306_command(ssd1306_t *ssd, uint8_t command); //Envia um comando
void ssd1306_send_data(ssd1306_t *ssd); //Envia buffer de dados
void ssd1306_pixel(ssd1306_t *ssd, uint8_t x, uint8_t y, bool value); //Desenha um pixel
void ssd1306_fill(ssd1306_t *ssd, bool value); //Preenche a tela
void ssd1306_rect(ssd1306_t *ssd, uint8_t top, uint8_t left, uint8_t width, uint8_t height, bool value, bool fill); //Desenha um retângulo
void ssd1306_line(ssd1306_t *ssd, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool value); //Desenha uma linha
void ssd1306_hline(ssd1306_t *ssd, uint8_t x0, uint8_t x1, uint8_t y, bool value); //Desenha uma linha horizontal
void ssd1306_vline(ssd1306_t *ssd, uint8_t x, uint8_t y0, uint8_t y1, bool value); //Desenha uma linha vertical

//Funções para texto
void ssd1306_draw_small_number(ssd1306_t *ssd, char c, uint8_t x, uint8_t y); //Desenha números pequenos (5x5 pixels)
void ssd1306_draw_char(ssd1306_t *ssd, char c, uint8_t x, uint8_t y, bool use_small_numbers); //Desenha um caractere
void ssd1306_draw_string(ssd1306_t *ssd, const char *str, uint8_t x, uint8_t y, bool use_small_numbers); //Desenha uma string

#endif /* SSD1306_H */