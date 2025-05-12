#include "matriz_led.h"

const CorRGB PALETA_CORES[] = {
    {"Preto", 0, 0, 0}, //cor preta não tem valor rgb
    {"Marrom",   30,   10,    10},  
    {"Vermelho", 190,   0,    0},  
    {"Laranja",  255,  65,    0},  
    {"Amarelo",  255, 140,    0},  
    {"Verde",     0,  150,    0},
    {"Azul",      0,    0,  200}, 
    {"Violeta", 130,    0,  130},  
    {"Cinza",   40,    35,  35},
    {"Branco",  255,  255,  255},  // Branco verdadeiro
    {"Prata",   192,  192,  192},
    {"Ouro",    218,  165,   32},
    {"---",      0,    0,    0}
};

const uint8_t PAD_OK[5]  = {0b00001,0b00010,0b00100,0b11000,0b10000};  //Padrão "✓" para verde
const uint8_t PAD_EXC[5] = {0b00100,0b00100,0b00100,0b00000,0b00100};  //Padrão "!" para amarelo
const uint8_t PAD_X[5]   = {0b10001,0b01010,0b00100,0b01010,0b10001};  //Padrão "X" para vermelho

// Novos padrões de números (invertidos)
const bool padrao_numeros[10][25] = {
    // 0 (invertido)
    {
        1, 1, 1, 1, 1,
        1, 0, 0, 0, 1,
        1, 0, 0, 0, 1,
        1, 0, 0, 0, 1,
        1, 1, 1, 1, 1
    },
    // 1 (invertido)
    {
        1, 1, 1, 1, 1,
        0, 0, 1, 0, 0,
        0, 0, 1, 0, 0,
        0, 1, 1, 0, 0,
        0, 0, 1, 0, 0
    },
    // 2 (invertido)
    {
        1, 1, 1, 1, 1,
        1, 0, 0, 0, 0,
        1, 1, 1, 1, 1,
        0, 0, 0, 0, 1,
        1, 1, 1, 1, 1
    },
    // 3 (invertido)
    {
        1, 1, 1, 1, 1,
        0, 0, 0, 0, 1,
        1, 1, 1, 1, 1,
        0, 0, 0, 0, 1,
        1, 1, 1, 1, 1
    },
    // 4 (invertido)
    {
        1, 0, 0, 0, 0,
        0, 0, 0, 0, 1,
        1, 1, 1, 1, 1,
        1, 0, 0, 0, 1,
        1, 0, 0, 0, 1
    },
    // 5 (invertido)
    {
        1, 1, 1, 1, 1,
        0, 0, 0, 0, 1,
        1, 1, 1, 1, 1,
        1, 0, 0, 0, 0,
        1, 1, 1, 1, 1
    },
    // 6 (invertido)
    {
        1, 1, 1, 1, 1,
        1, 0, 0, 0, 1,
        1, 1, 1, 1, 1,
        1, 0, 0, 0, 0,
        1, 1, 1, 1, 1
    },
    // 7 (invertido)
    {
        1, 0, 0, 0, 0,
        0, 0, 0, 0, 1,
        1, 1, 1, 0, 0,
        0, 0, 0, 0, 1,
        1, 1, 1, 1, 1
    },
    // 8 (invertido)
    {
        1, 1, 1, 1, 1,
        1, 0, 0, 0, 1,
        1, 1, 1, 1, 1,
        1, 0, 0, 0, 1,
        1, 1, 1, 1, 1
    },
    // 9 (invertido)
    {
        1, 1, 1, 1, 1,
        0, 0, 0, 0, 1,
        1, 1, 1, 1, 1,
        1, 0, 0, 0, 1,
        1, 1, 1, 1, 1
    }
};

static inline void ws2812_put(uint32_t grb) {  //Envia dados GRB para um LED
    pio_sm_put_blocking(pio0, 0, grb << 8u);  //Desloca 8 bits para alinhar protocolo WS2812
}

void inicializar_matriz_led(void) {  //CONFIGURA PIO PARA CONTROLAR WS2812
    PIO pio = pio0;
    uint off = pio_add_program(pio, &ws2812_program);  //Carrega programa PIO
    ws2812_program_init(pio, 0, off, PINO_WS2812, 800000, RGBW_ATIVO);  //Inicia PIO a 800kHz
}

void matriz_draw_pattern(const uint8_t pad[5], uint32_t cor_on) {  //DESENHA PADRÃO NA MATRIZ
    /* placa montada "de cabeça-para-baixo" → linha 4 primeiro */
    for (int lin = 4; lin >= 0; --lin) {
        for (int col = 0; col < 5; ++col) {
            bool aceso = pad[lin] & (1 << (4 - col));  //Verifica bit do padrão
            ws2812_put(aceso ? cor_on : COR_OFF);  //Aplica cor ou desliga LED
        }
    }
    sleep_us(60);  //Latência para atualizar matriz
}

void matriz_draw_number(uint8_t numero, uint32_t cor_on) {  //DESENHA UM NÚMERO NA MATRIZ
    if (numero > 9) return;  // Validação para garantir número entre 0-9
    
    /* O formato da matriz boolean requer uma lógica diferente para desenhar */
    for (int lin = 0; lin < 5; ++lin) {
        for (int col = 0; col < 5; ++col) {
            bool aceso = padrao_numeros[numero][lin * 5 + col];
            ws2812_put(aceso ? cor_on : COR_OFF);
        }
    }
    sleep_us(60);  //Latência para atualizar matriz
}

void matriz_clear(void) {  //LIMPA TODOS OS LEDs
    for (int i = 0; i < NUM_PIXELS; ++i)
        ws2812_put(COR_OFF);  //Desliga cada LED
    sleep_us(60);  //Latência para confirmar limpeza
}