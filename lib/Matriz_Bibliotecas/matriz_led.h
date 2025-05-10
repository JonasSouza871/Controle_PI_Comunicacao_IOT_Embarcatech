#ifndef MATRIZ_LED_H
#define MATRIZ_LED_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "generated/ws2812.pio.h"


#define PINO_WS2812   7  //Pino GPIO para comunicação com WS2812
#define NUM_LINHAS    5  //Número de linhas da matriz
#define NUM_COLUNAS   5  //Número de colunas da matriz
#define NUM_PIXELS    (NUM_LINHAS * NUM_COLUNAS)  //Total de LEDs (25)
#define RGBW_ATIVO    false  //Define protocolo RGB (não RGBW)

/* ---------- Utilidades de cor ---------- */
#define GRB(r,g,b)   ( ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (b) )  //Converte RGB para formato GRB do WS2812

/* ---------- Cores ---------- */
extern const uint32_t COR_VERDE;  //Cor para estado verde
extern const uint32_t COR_AMARELO;  //Cor para estado amarelo
extern const uint32_t COR_VERMELHO;  //Cor para estado vermelho
extern const uint32_t COR_OFF;  //Desliga LEDs

/* ---------- Padrões 5 × 5 (✓, !, X) ---------- */
extern const uint8_t PAD_OK[5];  //Padrão "✓" para verde
extern const uint8_t PAD_EXC[5];  //Padrão "!" para amarelo
extern const uint8_t PAD_X[5];  //Padrão "X" para vermelho

/* ---------- API mínima ---------- */
void inicializar_matriz_led(void);  //Inicializa PIO para WS2812
void matriz_draw_pattern(const uint8_t pad[5], uint32_t cor_on);  //Desenha padrão na matriz
void matriz_clear(void);  //Limpa todos os LEDs

#endif /* MATRIZ_LED_H */