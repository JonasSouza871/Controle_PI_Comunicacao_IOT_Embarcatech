// main.c
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lib/DS18b20/ds18b20.h"
#include "lib/Display_Bibliotecas/ssd1306.h"
#include "lib/Matriz_Bibliotecas/matriz_led.h"

//Pinos utilizados no projeto
#define PIN_DS18B20   20    //Pino do sensor de temperatura DS18B20
#define PIN_VRY       26    //Pino vertical do joystick (ADC0)
#define PIN_BTN_A     5     //Pino do botão A
#define PIN_PWM_OUT   11    //Pino PWM para controle do motor/ventoinha
#define I2C_SDA       14    //Pino SDA do I2C para o display OLED
#define I2C_SCL       15    //Pino SCL do I2C para o display OLED

//Configurações do I2C para o display OLED
#define OLED_I2C_PORT i2c1  //Instância do I2C usada
#define OLED_ADDR     0x3C  //Endereço I2C do OLED

//Valores fixos de RPM
#define RPM_MIN 300.0f      //RPM mínimo do motor
#define RPM_MAX 2000.0f     //RPM máximo do motor

//Variáveis globais
static ssd1306_t oled;              //Estrutura para o display OLED
static float     temperatura_atual; //Temperatura lida pelo sensor
static int       setpoint = 20;     //Temperatura desejada (em °C)
static volatile uint16_t duty_cycle_pwm = 0; //Duty cycle do PWM
static float     rpm_simulado = RPM_MIN; //RPM simulado para o motor

//Flags compartilhadas entre tasks
static bool selecionando = true;         //Indica modo de seleção do setpoint
static bool exibir_tela_principal = true;//Alterna entre tela principal e RPM

//===== Task_Sensor =====
//Lê a temperatura do sensor DS18B20 a cada 1 segundo
void Task_Sensor(void *pv) {
    ds18b20_init(PIN_DS18B20); //Inicializa o sensor
    while (true) {
        temperatura_atual = ds18b20_get_temperature(); //Atualiza a temperatura
        vTaskDelay(pdMS_TO_TICKS(1000)); //Espera 1 segundo
    }
}

//===== Task_Input =====
//Ajusta o setpoint com o joystick e confirma com o botão A
void Task_Input(void *pv) {
    adc_init(); //Inicializa o conversor analógico-digital
    adc_gpio_init(PIN_VRY); //Configura o pino do joystick
    adc_select_input(0); //Seleciona o canal ADC0

    gpio_init(PIN_BTN_A); //Inicializa o pino do botão
    gpio_set_dir(PIN_BTN_A, GPIO_IN); //Define como entrada
    gpio_pull_up(PIN_BTN_A); //Ativa pull-up interno

    bool ultimo_estado_btn = false; //Estado anterior do botão
    int  ultima_direcao = 0; //Última direção detectada do joystick

    while (true) {
        uint16_t valor_raw = adc_read(); //Lê o valor bruto do joystick
        bool estado_btn_atual = (gpio_get(PIN_BTN_A) == 0); //Botão pressionado?
        int direcao = (valor_raw > 3000 ? 1 : (valor_raw < 1000 ? -1 : 0)); //Detecta movimento

        if (selecionando) {
            //Ajusta o setpoint dentro dos limites (10 a 30°C)
            if (direcao == 1 && ultima_direcao == 0 && setpoint < 30) setpoint++;
            if (direcao == -1 && ultima_direcao == 0 && setpoint > 10) setpoint--;
            //Confirma o setpoint ao pressionar o botão
            if (estado_btn_atual && !ultimo_estado_btn) selecionando = false;
        } else {
            //Retorna ao modo de seleção ao pressionar o botão
            if (estado_btn_atual && !ultimo_estado_btn) selecionando = true;
        }

        ultimo_estado_btn = estado_btn_atual; //Atualiza estado do botão
        ultima_direcao = direcao; //Atualiza direção do joystick
        vTaskDelay(pdMS_TO_TICKS(100)); //Espera 100ms
    }
}

//===== Task_Control =====
//Implementa o controle PI e ajusta o PWM a cada 1 segundo
void Task_Control(void *pv) {
    const float kp = 120.0f; //Ganho proporcional
    const float ki = 120.0f / 15.0f; //Ganho integral
    const float h = 1.0f; //Período de amostragem (1s)
    static float integral = 0.0f; //Acumulador do erro integral

    //Configura o pino PWM
    gpio_set_function(PIN_PWM_OUT, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_PWM_OUT);
    uint chan = pwm_gpio_to_channel(PIN_PWM_OUT);
    pwm_set_wrap(slice, 65535); //Define resolução do PWM
    pwm_set_chan_level(slice, chan, 0); //Inicia com duty cycle 0
    pwm_set_enabled(slice, true); //Habilita o PWM

    while (true) {
        //Calcula o erro entre a temperatura atual e o setpoint
        float erro = temperatura_atual - (float)setpoint;
        float P = kp * erro; //Termo proporcional
        integral += ki * erro * h; //Termo integral
        integral = fmaxf(fminf(integral, 4096.0f), -4096.0f); //Limita o integral

        //Calcula o sinal de controle e converte para duty cycle
        float U = P + integral;
        int32_t duty = (int32_t)((U + 4096.0f) * (65535.0f / 8192.0f));
        duty = duty < 0 ? 0 : (duty > 65535 ? 65535 : duty); //Limita entre 0 e 65535

        pwm_set_chan_level(slice, chan, duty); //Aplica o duty cycle
        duty_cycle_pwm = duty; //Atualiza variável global

        //Simula o RPM com base no duty cycle
        rpm_simulado = RPM_MIN + (RPM_MAX - RPM_MIN) * (duty / 65535.0f);

        vTaskDelay(pdMS_TO_TICKS((uint32_t)(h * 1000))); //Espera 1 segundo
    }
}

//===== Task_Display =====
//Gerencia a exibição no OLED e na matriz de LEDs
void Task_Display(void *pv) {
    char buf[32]; //Buffer para strings
    uint32_t ultimo_alternar = to_ms_since_boot(get_absolute_time()); //Controla alternância de telas

    while (true) {
        uint32_t agora = to_ms_since_boot(get_absolute_time());
        //Alterna entre telas a cada 5 segundos quando não está selecionando
        if (!selecionando && agora - ultimo_alternar > 5000) {
            exibir_tela_principal = !exibir_tela_principal;
            ultimo_alternar = agora;
        }

        ssd1306_fill(&oled, false); //Limpa o display OLED

        if (selecionando) {
            //Tela de ajuste do setpoint
            ssd1306_draw_string(&oled, "Ajuste Setpoint:", 0, 0, false);
            snprintf(buf, sizeof(buf), "   %2d C", setpoint);
            ssd1306_draw_string(&oled, buf, 0, 16, false);
            ssd1306_draw_string(&oled, "[A] Confirma", 0, 32, false);
        } else if (exibir_tela_principal) {
            //Tela principal de monitoramento
            snprintf(buf, sizeof(buf), "Temp: %4.1f C", temperatura_atual);
            ssd1306_draw_string(&oled, buf, 0, 0, false);
            snprintf(buf, sizeof(buf), "Set:  %3d C", setpoint);
            ssd1306_draw_string(&oled, buf, 0, 16, false);
            snprintf(buf, sizeof(buf), "Erro: %4.1f C", setpoint - temperatura_atual);
            ssd1306_draw_string(&oled, buf, 0, 32, false);
            snprintf(buf, sizeof(buf), "PWM:  %5u", duty_cycle_pwm);
            ssd1306_draw_string(&oled, buf, 0, 48, false);

            //Exibe o erro na matriz de LEDs com a nova lógica de arredondamento
            float erro = fabsf(setpoint - temperatura_atual); //Valor absoluto do erro
            int parte_inteira = (int)floorf(erro); //Parte inteira do erro
            float parte_decimal = erro - parte_inteira; //Parte decimal do erro
            int digito = (parte_decimal >= 0.6f) ? parte_inteira + 1 : parte_inteira; //Arredonda só a partir de 0.6

            uint32_t cor;
            switch (digito) {
                case 0: cor = COR_BRANCO;  break;
                case 1: cor = COR_PRATA;   break;
                case 2: cor = COR_CINZA;   break;
                case 3: cor = COR_VIOLETA; break;
                case 4: cor = COR_AZUL;    break;
                case 5: cor = COR_MARROM;  break;
                case 6: cor = COR_VERDE;   break;
                case 7: cor = COR_OURO;    break;
                case 8: cor = COR_LARANJA; break;
                case 9: cor = COR_AMARELO; break;
                default: cor = COR_OFF;    break;
            }
            if (erro > 9.6f) {
                matriz_draw_pattern(PAD_X, COR_VERMELHO); //Erro alto: padrão X vermelho
            } else {
                matriz_draw_number(digito, cor); //Exibe o dígito do erro
            }
        } else {
            //Tela de exibição do RPM
            snprintf(buf, sizeof(buf), "RPM: %4.0f", rpm_simulado);
            ssd1306_draw_string(&oled, buf, 0, 0, false);

            //Barra de progresso do RPM
            float range = RPM_MAX - RPM_MIN;
            float pos = (rpm_simulado - RPM_MIN) / range;
            int len = (int)(pos * oled.width);
            for (int i = 0; i < len; i++) {
                ssd1306_line(&oled, i, 20, i, 25, true);
            }

            //Exibe valores mínimo e máximo de RPM
            snprintf(buf, sizeof(buf), "Min:%4.0f Max:%4.0f", RPM_MIN, RPM_MAX);
            ssd1306_draw_string(&oled, buf, 0, 32, false);

            //Indica ação baseada na temperatura
            const char *acao = (temperatura_atual > setpoint) ? "ESFRIAR!!" : "ESQUENTAR!!";
            ssd1306_draw_string(&oled, acao, 0, 50, false);
        }

        ssd1306_send_data(&oled); //Atualiza o display
        vTaskDelay(pdMS_TO_TICKS(100)); //Espera 100ms
    }
}

//===== Função Principal =====
//Inicializa o sistema e cria as tasks
int main() {
    stdio_init_all(); //Inicializa comunicação serial
    inicializar_matriz_led(); //Configura a matriz de LEDs

    //Inicializa o I2C para o OLED
    i2c_init(OLED_I2C_PORT, 400000); //Frequência de 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA); //Ativa pull-ups nos pinos I2C
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&oled, 128, 64, false, OLED_ADDR, OLED_I2C_PORT); //Configura OLED
    ssd1306_config(&oled);

    //Cria as tasks do FreeRTOS
    xTaskCreate(Task_Sensor, "Sensor", 256, NULL, 2, NULL); //Prioridade alta para sensor
    xTaskCreate(Task_Input, "Input", 512, NULL, 1, NULL);
    xTaskCreate(Task_Control, "Control", 512, NULL, 1, NULL);
    xTaskCreate(Task_Display, "Display", 512, NULL, 1, NULL);

    vTaskStartScheduler(); //Inicia o escalonador do FreeRTOS
    while (true) tight_loop_contents(); //Loop de espera (nunca alcançado)
    return 0;
}