#include <stdio.h>
#include "pico/stdlib.h"
//#include "pico/cyw43_arch.h" //Comentado: não necessário para leitura do sensor
//#include "lwip/pbuf.h"       //Comentado: não necessário para leitura do sensor
//#include "lwip/tcp.h"        //Comentado: não necessário para leitura do sensor
//#include "lwip/netif.h"      //Comentado: não necessário para leitura do sensor
//#include "lib/Wifi/wifi_config.h" //Comentado: não necessário para leitura do sensor
#include "lib/DS18b20/ds18b20.h"         //Biblioteca para o sensor DS18B20

//Define o pino do sensor DS18B20
#define DS18B20_PIN 2

int main() {
    stdio_init_all();
    sleep_ms(3000); //Aguarda o console serial

    printf("\n\n========== LEITURA DE TEMPERATURA DS18B20 ==========\n");
    printf("Versão: 1.0 - Exibição no monitor serial\n");
    printf("==================================================\n\n");

    //Inicializa o sensor DS18B20
    ds18b20_init(DS18B20_PIN);

    //Loop principal: lê e exibe a temperatura
    while (1) {
        if (!ds18b20_reset()) {
            printf("❌ Sensor DS18B20 não detectado!\n");
        } else {
            float temperature = ds18b20_get_temperature();
            printf("Temperatura: %.2f °C\n", temperature);
        }
        sleep_ms(1000); //Aguarda 1 segundo antes da próxima leitura
    }
}