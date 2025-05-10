#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/netif.h"
#include "hardware/adc.h"
#include "lib/Wifi/wifi_config.h"  // Inclui o arquivo de cabeçalho com as credenciais Wi-Fi

// Definição dos pinos dos LEDs
#define LED_BLUE_PIN 12
#define LED_GREEN_PIN 11
#define LED_RED_PIN 13
#define CYW43_LED_PIN CYW43_WL_GPIO_LED_PIN
#define MAX_RETRIES 5

// Protótipos de função
void blink_led(int blinks, int interval_ms);
int init_cyw43_arch();
void gpio_led_bitdog(void);
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
float temp_read(void);
void user_request(char **request);

// Função para piscar o LED para indicar status
void blink_led(int blinks, int interval_ms) {
    for (int i = 0; i < blinks; i++) {
        cyw43_arch_gpio_put(CYW43_LED_PIN, 1);
        sleep_ms(interval_ms);
        cyw43_arch_gpio_put(CYW43_LED_PIN, 0);
        sleep_ms(interval_ms);
    }
}

// Inicializa a arquitetura do cyw43
int init_cyw43_arch() {
    printf("\n\n===== DIAGNÓSTICO DE CONEXÃO WI-FI =====\n");

    // 1. Inicialização da arquitetura CYW43
    printf("Passo 1: Inicializando hardware CYW43...\n");
    int retries = MAX_RETRIES;
    while (retries--) {
        if (cyw43_arch_init() == 0) {
            printf("✅ Hardware CYW43 inicializado com sucesso!\n");
            blink_led(1, 200);  // 1 piscada rápida - hardware OK
            break;
        }
        printf("❌ Tentativa %d falhou. Tentando novamente em 1 segundo...\n", MAX_RETRIES - retries);
        sleep_ms(1000);
    }

    if (retries < 0) {
        printf("❌❌❌ ERRO CRÍTICO: Falha ao inicializar hardware CYW43\n");
        return -1;
    }

    // 2. Configuração do modo Station
    printf("\nPasso 2: Configurando modo Station...\n");
    cyw43_arch_enable_sta_mode();
    printf("✅ Modo Station ativado\n");
    blink_led(2, 200);  // 2 piscadas rápidas - modo station OK

    // 3. Verificação de credenciais
    printf("\nPasso 3: Verificando credenciais Wi-Fi...\n");
    printf("- SSID configurado: %s\n", WIFI_SSID);
    printf("- Senha configurada: %s\n", WIFI_PASSWORD);

    // 4. Conexão à rede Wi-Fi com senha
    printf("\nPasso 4: Tentando conexão Wi-Fi com senha...\n");

    bool connected = false;
    retries = MAX_RETRIES;

    while (retries--) {
        printf("- Tentativa %d de %d...", MAX_RETRIES - retries, MAX_RETRIES);
        fflush(stdout);  // Garante que a mensagem seja exibida

        int result = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000);

        if (result == 0) {
            printf(" ✅ CONECTADO!\n");
            connected = true;
            break;
        } else {
            printf(" ❌ falhou (código de erro: %d)\n", result);
            // Pisca o LED para indicar falha na tentativa
            blink_led(1, 100);

            // Vamos desativar e reativar o modo station para uma nova tentativa limpa
            if (retries > 0) {
                printf("   Resetando interface Wi-Fi antes da próxima tentativa...\n");
                cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
                sleep_ms(1000);
                cyw43_arch_enable_sta_mode();
                sleep_ms(1000);
            }
        }
    }

    if (!connected) {
        printf("\n❌❌❌ FALHA NA CONEXÃO WI-FI após %d tentativas\n", MAX_RETRIES);
        printf("Verificações para rede com senha:\n");
        printf("1. O SSID está correto?\n");
        printf("2. A senha está correta?\n");
        printf("3. Verifique se o roteador está próximo e funcionando\n");
        printf("4. Tente reiniciar o roteador\n");

        // Pisca o LED em padrão de erro
        for (int i = 0; i < 3; i++) {
            blink_led(3, 200);
            sleep_ms(1000);
        }

        return -2;
    }

    // 5. Verificação de IP
    printf("\nPasso 5: Verificando configuração de IP...\n");
    sleep_ms(1000); // Aguarda um pouco para o IP ser atribuído

    if (netif_default) {
        printf("✅ IP atribuído: %s\n", ipaddr_ntoa(&netif_default->ip_addr));

        // Verificar se recebemos um IP válido (não 0.0.0.0)
        if (strcmp(ipaddr_ntoa(&netif_default->ip_addr), "0.0.0.0") == 0) {
            printf("⚠️ IP inválido (0.0.0.0). Problema com DHCP!\n");
        }
    } else {
        printf("⚠️ Aviso: Interface de rede não disponível ou IP não atribuído\n");
    }

    // Celebração de conexão bem-sucedida
    printf("\n✅✅✅ CONEXÃO WI-FI ESTABELECIDA COM SUCESSO! ✅✅✅\n");

    // Pisca o LED para indicar sucesso na conexão
    for (int i = 0; i < 3; i++) {
        blink_led(3, 100);
        sleep_ms(300);
    }

    return 0;
}

// Inicializar os Pinos GPIO para acionamento dos LEDs da BitDogLab
void gpio_led_bitdog(void){
    // Configuração dos LEDs como saída
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    gpio_put(LED_BLUE_PIN, false);

    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, false);

    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, false);
}

// Função de callback ao aceitar conexões TCP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, tcp_server_recv);
    return ERR_OK;
}

// Tratamento do request do usuário
void user_request(char **request){
    if (strstr(*request, "GET /blue_on") != NULL) {
        gpio_put(LED_BLUE_PIN, 1);
    }
    else if (strstr(*request, "GET /blue_off") != NULL) {
        gpio_put(LED_BLUE_PIN, 0);
    }
    else if (strstr(*request, "GET /green_on") != NULL) {
        gpio_put(LED_GREEN_PIN, 1);
    }
    else if (strstr(*request, "GET /green_off") != NULL) {
        gpio_put(LED_GREEN_PIN, 0);
    }
    else if (strstr(*request, "GET /red_on") != NULL) {
        gpio_put(LED_RED_PIN, 1);
    }
    else if (strstr(*request, "GET /red_off") != NULL) {
        gpio_put(LED_RED_PIN, 0);
    }
    else if (strstr(*request, "GET /on") != NULL) {
        cyw43_arch_gpio_put(CYW43_LED_PIN, 1);
    }
    else if (strstr(*request, "GET /off") != NULL) {
        cyw43_arch_gpio_put(CYW43_LED_PIN, 0);
    }
}

// Leitura da temperatura interna
float temp_read(void){
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
    uint16_t raw_value = adc_read();
    const float conversion_factor = 3.3f / (1 << 12);
    float temperature = 27.0f - ((raw_value * conversion_factor) - 0.706f) / 0.001721f;
    return temperature;
}

// Função de callback para processar requisições HTTP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(tpcb);
        tcp_recv(tpcb, NULL);
        return ERR_OK;
    }

    // Alocação do request na memória dinámica
    char *request = (char *)malloc(p->len + 1);
    memcpy(request, p->payload, p->len);
    request[p->len] = '\0';

    printf("Request: %s\n", request);

    // Tratamento de request - Controle dos LEDs
    user_request(&request);

    // Leitura da temperatura interna
    float temperature = temp_read();

    // Cria a resposta HTML
    char html[1024];

    // Instruções html do webserver
    snprintf(html, sizeof(html),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n"
             "\r\n"
             "<!DOCTYPE html>\n"
             "<html>\n"
             "<head>\n"
             "<title> Embarcatech - LED Control </title>\n"
             "<style>\n"
             "body { background-color: #b5e5fb; font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }\n"
             "h1 { font-size: 64px; margin-bottom: 30px; }\n"
             "button { background-color: LightGray; font-size: 36px; margin: 10px; padding: 20px 40px; border-radius: 10px; }\n"
             ".temperature { font-size: 48px; margin-top: 30px; color: #333; }\n"
             "</style>\n"
             "</head>\n"
             "<body>\n"
             "<h1>Embarcatech: LED Control</h1>\n"
             "<form action=\"./blue_on\"><button>Ligar Azul</button></form>\n"
             "<form action=\"./blue_off\"><button>Desligar Azul</button></form>\n"
             "<form action=\"./green_on\"><button>Ligar Verde</button></form>\n"
             "<form action=\"./green_off\"><button>Desligar Verde</button></form>\n"
             "<form action=\"./red_on\"><button>Ligar Vermelho</button></form>\n"
             "<form action=\"./red_off\"><button>Desligar Vermelho</button></form>\n"
             "<p class=\"temperature\">Temperatura Interna: %.2f &deg;C</p>\n"
             "</body>\n"
             "</html>\n",
             temperature);

    // Escreve dados para envio (mas não os envia imediatamente).
    tcp_write(tpcb, html, strlen(html), TCP_WRITE_FLAG_COPY);

    // Envia a mensagem
    tcp_output(tpcb);

    //libera memória alocada dinamicamente
    free(request);

    //libera um buffer de pacote (pbuf) que foi alocado anteriormente
    pbuf_free(p);

    return ERR_OK;
}

int main() {
    stdio_init_all();

    // Aguarda o console serial se estabelecer
    sleep_ms(3000);

    printf("\n\n========== SISTEMA DE CONEXÃO WI-FI - PICO W ==========\n");
    printf("Versão: 1.4 - Com Webserver e Controle de LEDs\n");
    printf("======================================================\n\n");

    // Inicializa os LEDs da BitDogLab
    gpio_led_bitdog();

    // Inicializa o ADC para leitura de temperatura
    adc_init();
    adc_set_temp_sensor_enabled(true);

    int result = init_cyw43_arch();
    if (result != 0) {
        printf("Programa finalizado com código de erro: %d\n", result);

        // Pisca LED continuamente para indicar erro
        while (1) {
            blink_led(result == -1 ? 1 : 2, 200);
            sleep_ms(1000);
        }
    }

    // Configura o servidor TCP - cria novos PCBs TCP
    struct tcp_pcb *server = tcp_new();
    if (!server) {
        printf("Falha ao criar servidor TCP\n");
        return -1;
    }

    // Vincula o servidor TCP à porta 80
    if (tcp_bind(server, IP_ADDR_ANY, 80) != ERR_OK) {
        printf("Falha ao associar servidor TCP à porta 80\n");
        return -1;
    }

    // Coloca o servidor em modo de escuta
    server = tcp_listen(server);

    // Define a função de callback para aceitar conexões
    tcp_accept(server, tcp_server_accept);
    printf("\n✅ Servidor Web iniciado na porta 80\n");
    printf("Acesse o IP %s no seu navegador\n", ipaddr_ntoa(&netif_default->ip_addr));

    printf("\nConexão Wi-Fi estabelecida e servidor web ativo. Mantendo conexão...\n");

    // Variável para contar segundos conectados
    unsigned int uptime_seconds = 0;
    unsigned int reconnect_attempt = 0;

    // Loop principal - mantém a conexão e monitora status
    while (1) {
        // No modo background, você precisa chamar esta função regularmente
        cyw43_arch_poll();

        // A cada segundo, verifica estado
        sleep_ms(100);
        uptime_seconds++;

        // A cada 15 segundos, mostra status
        if (uptime_seconds % 15 == 0) {
            int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
            printf("[%u segundos] Status Wi-Fi: %s (RSSI: %d dBm)\n",
                   uptime_seconds,
                   status == CYW43_LINK_UP ? "CONECTADO" : "DESCONECTADO",
                   cyw43_wifi_get_rssi(&cyw43_state, CYW43_ITF_STA));

            // Pisca o LED para indicar que o sistema está vivo
            blink_led(1, 50);
        }

        // Verifica o status da conexão e tenta reconectar se necessário
        if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) {
            reconnect_attempt++;
            printf("⚠️ Conexão Wi-Fi perdida! Tentativa de reconexão %d...\n", reconnect_attempt);

            // Resetar interface Wi-Fi antes de tentar reconectar
            cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
            sleep_ms(1000);
            cyw43_arch_enable_sta_mode();
            sleep_ms(1000);

            if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000) == 0) {
                printf("✅ Reconectado com sucesso!\n");
                blink_led(2, 200);
                reconnect_attempt = 0;
            } else {
                printf("❌ Falha ao reconectar\n");
                blink_led(3, 100);

                // Se tiver muitas falhas consecutivas, reinicie o hardware
                if (reconnect_attempt >= 5) {
                    printf("Reiniciando hardware Wi-Fi após múltiplas falhas...\n");
                    cyw43_arch_deinit();
                    sleep_ms(1000);

                    if (init_cyw43_arch() == 0) {
                        printf("Hardware Wi-Fi reiniciado com sucesso!\n");
                        reconnect_attempt = 0;
                    } else {
                        printf("Falha ao reiniciar hardware Wi-Fi!\n");
                    }
                }
            }
        }
    }
}
