#include "matriz_led.h"

const uint32_t COR_VERDE    = GRB(0,   150, 0);  //Verde com intensidade moderada
const uint32_t COR_AMARELO  = GRB(255, 140, 0);  //Amarelo com tom quente
const uint32_t COR_VERMELHO = GRB(190,   0, 0);  //Vermelho com brilho reduzido
const uint32_t COR_OFF      = 0;                 //Desliga o LED

const uint8_t PAD_OK[5]  = {0b00001,0b00010,0b00100,0b11000,0b10000};  //Padrão "✓" para verde
const uint8_t PAD_EXC[5] = {0b00100,0b00100,0b00100,0b00000,0b00100};  //Padrão "!" para amarelo
const uint8_t PAD_X[5]   = {0b10001,0b01010,0b00100,0b01010,0b10001};  //Padrão "X" para vermelho

const uint8_t PAD_0[5] = {0b01110, 0b10001, 0b10001, 0b10001, 0b01110};
const uint8_t PAD_1[5] = {0b00100, 0b01100, 0b00100, 0b00100, 0b01110};
const uint8_t PAD_2[5] = {0b01110, 0b10001, 0b00110, 0b01000, 0b11111};
const uint8_t PAD_3[5] = {0b11111, 0b00010, 0b01110, 0b00010, 0b11111};
const uint8_t PAD_4[5] = {0b10001, 0b10001, 0b11111, 0b00001, 0b00001};
const uint8_t PAD_5[5] = {0b11111, 0b10000, 0b11110, 0b00001, 0b11110};
const uint8_t PAD_6[5] = {0b01110, 0b10000, 0b11110, 0b10001, 0b01110};
const uint8_t PAD_7[5] = {0b11111, 0b00010, 0b00100, 0b01000, 0b01000};
const uint8_t PAD_8[5] = {0b01110, 0b10001, 0b01110, 0b10001, 0b01110};
const uint8_t PAD_9[5] = {0b01110, 0b10001, 0b01111, 0b00001, 0b01110};

static inline void ws2812_put(uint32_t grb) {  //Envia dados GRB para um LED
    pio_sm_put_blocking(pio0, 0, grb << 8u);  //Desloca 8 bits para alinhar protocolo WS2812
}

void inicializar_matriz_led(void) {  //CONFIGURA PIO PARA CONTROLAR WS2812
    PIO pio = pio0;
    uint off = pio_add_program(pio, &ws2812_program);  //Carrega programa PIO
    ws2812_program_init(pio, 0, off, PINO_WS2812, 800000, RGBW_ATIVO);  //Inicia PIO a 800kHz
}

void matriz_draw_pattern(const uint8_t pad[5], uint32_t cor_on) {  //DESENHA PADRÃO NA MATRIZ
    /* placa montada “de cabeça-para-baixo” → linha 4 primeiro */
    for (int lin = 4; lin >= 0; --lin) {
        for (int col = 0; col < 5; ++col) {
            bool aceso = pad[lin] & (1 << (4 - col));  //Verifica bit do padrão
            ws2812_put(aceso ? cor_on : COR_OFF);  //Aplica cor ou desliga LED
        }
    }
    sleep_us(60);  //Latência para atualizar matriz
}

void matriz_clear(void) {  //LIMPA TODOS OS LEDs
    for (int i = 0; i < NUM_PIXELS; ++i)
        ws2812_put(COR_OFF);  //Desliga cada LED
    sleep_us(60);  //Latência para confirmar limpeza
}
