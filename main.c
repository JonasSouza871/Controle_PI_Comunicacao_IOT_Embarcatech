// ============================================================================
//  main.c – Controle de temperatura + web-monitor para Raspberry Pi Pico W
// ============================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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

#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"

// -----------------  CONFIG Wi-Fi  -----------------
#define WIFI_SSID     "Jonas Souza"
#define WIFI_PASSWORD "12345678"

// -----------------  PINOS  -----------------
#define PIN_DS18B20 20
#define PIN_VRY     26
#define PIN_BTN_A    5
#define PIN_RGB_R   11
#define PIN_RGB_G   12
#define PIN_RGB_B   13
#define PIN_BUZZER  10
#define I2C_SDA     14
#define I2C_SCL     15
#define OLED_I2C_PORT i2c1
#define OLED_ADDR     0x3C

#define RPM_MIN 300.0f
#define RPM_MAX 2000.0f

// -----------------  GLOBAIS  -----------------
static ssd1306_t oled;
static float  temperatura_atual;
static int    setpoint = 20;
static volatile uint16_t duty_cycle_pwm;
static float  rpm_simulado;

static bool selecionando = true;
static bool exibir_tela_principal = true;

static uint sliceR, sliceG, sliceB;
static uint chanR,  chanG,  chanB;

// -----------------  PROTÓTIPOS  -----------------
void Task_Sensor   (void*);
void Task_Input    (void*);
void Task_Control  (void*);
void Task_Buzzer   (void*);
void Task_Display  (void*);
void Task_Webserver(void*);

// TCP callbacks
static err_t tcp_server_accept(void*, struct tcp_pcb*, err_t);
static err_t tcp_server_recv  (void*, struct tcp_pcb*, struct pbuf*, err_t);
static err_t tcp_server_sent  (void*, struct tcp_pcb*, uint16_t);

// ============================================================================
//  Tasks
// ============================================================================
void Task_Sensor(void*)
{
    ds18b20_init(PIN_DS18B20);
    for(;;) {
        temperatura_atual = ds18b20_get_temperature();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Task_Input(void*)
{
    adc_init();
    adc_gpio_init(PIN_VRY);
    adc_select_input(0);
    gpio_init(PIN_BTN_A);
    gpio_set_dir(PIN_BTN_A, GPIO_IN);
    gpio_pull_up(PIN_BTN_A);

    bool ultimo_btn = false;
    int  ultima_dir = 0;

    for(;;) {
        uint16_t raw = adc_read();
        bool btn = (gpio_get(PIN_BTN_A)==0);
        int dir = (raw>3000?1:(raw<1000?-1:0));

        if(selecionando){
            if(dir==1 && ultima_dir==0 && setpoint<30) setpoint++;
            if(dir==-1&& ultima_dir==0 && setpoint>10) setpoint--;
            if(btn && !ultimo_btn) selecionando=false;
        } else if(btn && !ultimo_btn){
            selecionando=true;
        }
        ultimo_btn=btn;
        ultima_dir=dir;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Task_Control(void*)
{
    const float kp=120.0f, ki=120.0f/15.0f, h=1.0f;
    float integral=0;
    for(;;){
        float erro=temperatura_atual-setpoint;
        integral += ki*erro*h;
        integral = fmaxf(fminf(integral,4096),-4096);
        float U = kp*erro + integral;
        int32_t d=(int32_t)((U+4096)*(65535.0f/8192.0f));
        if(d<0) d=0; if(d>65535) d=65535;
        duty_cycle_pwm=d;

        pwm_set_chan_level(sliceR,chanR,d);
        pwm_set_chan_level(sliceG,chanG,d);
        pwm_set_chan_level(sliceB,chanB,d);

        rpm_simulado = RPM_MIN + (RPM_MAX-RPM_MIN)*(d/65535.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Task_Buzzer(void*)
{
    gpio_set_function(PIN_BUZZER,GPIO_FUNC_PWM);
    uint slice=pwm_gpio_to_slice_num(PIN_BUZZER);
    uint chan =pwm_gpio_to_channel(PIN_BUZZER);
    pwm_set_enabled(slice,true);

    for(;;){
        float erro=fabsf(temperatura_atual-setpoint);
        if(selecionando){
            pwm_set_enabled(slice,false);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        uint16_t on=500, off_fast=100, off_slow=600;
        if(erro>9.6f){
            pwm_set_clkdiv(slice,125.0f/1000); //1kHz
            pwm_set_wrap(slice,1000);
            pwm_set_chan_level(slice,chan,500);
            pwm_set_enabled(slice,true); vTaskDelay(pdMS_TO_TICKS(100));
            pwm_set_enabled(slice,false);vTaskDelay(pdMS_TO_TICKS(100));
        }else if(erro>=3.6f){
            pwm_set_clkdiv(slice,125.0f/500); //2kHz
            pwm_set_wrap(slice,1000);
            pwm_set_chan_level(slice,chan,500);
            pwm_set_enabled(slice,true); vTaskDelay(pdMS_TO_TICKS(200));
            pwm_set_enabled(slice,false);vTaskDelay(pdMS_TO_TICKS(600));
        }else{
            pwm_set_clkdiv(slice,125.0f/200); //5kHz
            pwm_set_wrap(slice,1000);
            pwm_set_chan_level(slice,chan,500);
            pwm_set_enabled(slice,true); vTaskDelay(pdMS_TO_TICKS(300));
            pwm_set_enabled(slice,false);vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void Task_Display(void*)
{
    char buf[32];
    uint32_t t0=to_ms_since_boot(get_absolute_time());

    for(;;){
        uint32_t now=to_ms_since_boot(get_absolute_time());
        if(!selecionando && now-t0>5000){
            exibir_tela_principal=!exibir_tela_principal;
            t0=now;
        }

        ssd1306_fill(&oled,false);
        if(selecionando){
            ssd1306_draw_string(&oled,"Ajuste Setpoint:",0,0,false);
            snprintf(buf,sizeof(buf),"   %2d C",setpoint);
            ssd1306_draw_string(&oled,buf,0,16,false);
            ssd1306_draw_string(&oled,"[A] Confirma",0,32,false);
        }else if(exibir_tela_principal){
            snprintf(buf,sizeof(buf),"Temp: %4.1f C",temperatura_atual);
            ssd1306_draw_string(&oled,buf,0,0,false);
            snprintf(buf,sizeof(buf),"Set:  %3d C",setpoint);
            ssd1306_draw_string(&oled,buf,0,16,false);
            snprintf(buf,sizeof(buf),"Erro: %4.1f C",setpoint-temperatura_atual);
            ssd1306_draw_string(&oled,buf,0,32,false);
            snprintf(buf,sizeof(buf),"PWM:  %5u",duty_cycle_pwm);
            ssd1306_draw_string(&oled,buf,0,48,false);

            float erro=fabsf(setpoint-temperatura_atual);
            int dig = (int)(erro+0.4f); if(dig>9) dig=9;
            if(erro>9.6f) matriz_draw_pattern(PAD_X,COR_VERMELHO);
            else           matriz_draw_number(dig,COR_VERDE);
        }else{
            snprintf(buf,sizeof(buf),"RPM: %4.0f",rpm_simulado);
            ssd1306_draw_string(&oled,buf,0,0,false);
            float pct=(rpm_simulado-RPM_MIN)/(RPM_MAX-RPM_MIN);
            int len=(int)(pct*oled.width);
            for(int i=0;i<len;i++) ssd1306_line(&oled,i,20,i,25,true);
            snprintf(buf,sizeof(buf),"Min:%4.0fMax:%4.0f",RPM_MIN,RPM_MAX);
            ssd1306_draw_string(&oled,buf,0,32,false);
            ssd1306_draw_string(&oled,(temperatura_atual>setpoint)?"ESFRIAR!!":"ESQUENTAR!!",0,50,false);
        }
        ssd1306_send_data(&oled);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================================
//  Webserver
// ============================================================================
static err_t tcp_server_sent(void* arg, struct tcp_pcb* tpcb, uint16_t len){
    // fecha a conexão após completar envio
    tcp_close(tpcb);
    return ERR_OK;
}

static err_t tcp_server_recv(void* arg, struct tcp_pcb* tpcb,
                             struct pbuf* p, err_t err)
{
    if(!p){
        tcp_close(tpcb);
        return ERR_OK;
    }

    // descartamos o request; resposta não depende dele
    pbuf_free(p);

    int brilho = (duty_cycle_pwm*100)/65535;

    static char body[640];
    int body_len = snprintf(body,sizeof(body),
        "<!DOCTYPE html><html><head><meta charset=\"UTF-8\">"
        "<meta http-equiv=\"refresh\" content=\"2\">"
        "<title>Monitor Pico W</title>"
        "<style>body{font-family:sans-serif;padding:16px}"
        "h1{font-size:28px}</style></head><body>"
        "<h1>Dados do Sistema (att. 2 s)</h1>"
        "<p>Temperatura atual: %.2f &#8451;</p>"
        "<p>Setpoint: %d &#8451;</p>"
        "<p>RPM simulado: %.0f</p>"
        "<p>Brilho LED RGB: %d&nbsp;%%</p>"
        "</body></html>",
        temperatura_atual,setpoint,rpm_simulado,brilho);

    char header[128];
    int header_len = snprintf(header,sizeof(header),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n", body_len);

    tcp_write(tpcb,header,header_len,TCP_WRITE_FLAG_COPY);
    tcp_write(tpcb,body,body_len,TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    // registra callback para fechar depois do envio
    tcp_sent(tpcb,tcp_server_sent);
    return ERR_OK;
}

static err_t tcp_server_accept(void* arg, struct tcp_pcb* newpcb, err_t err){
    tcp_recv(newpcb,tcp_server_recv);
    return ERR_OK;
}

void Task_Webserver(void*)
{
    if(cyw43_arch_init()){
        printf("Falha ao iniciar Wi-Fi\n");
        vTaskDelete(NULL);
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,0);
    cyw43_arch_enable_sta_mode();

    printf("Conectando a %s…\n",WIFI_SSID);
    if(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID,WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK,20000)){
        printf("Falha na conexão\n");
        vTaskDelete(NULL);
    }
    printf("Conectado!\n");
    if(netif_default)
        printf("IP: %s\n", ipaddr_ntoa(&netif_default->ip_addr));

    struct tcp_pcb* srv=tcp_new();
    if(!srv || tcp_bind(srv,IP_ADDR_ANY,80)!=ERR_OK){
        printf("Erro bind porta 80\n");
        vTaskDelete(NULL);
    }
    srv=tcp_listen(srv);
    tcp_accept(srv,tcp_server_accept);
    printf("HTTP on 80\n");

    for(;;){
        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================================
//  main
// ============================================================================
int main()
{
    stdio_init_all();
    inicializar_matriz_led();

    // OLED
    i2c_init(OLED_I2C_PORT,400000);
    gpio_set_function(I2C_SDA,GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL,GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA); gpio_pull_up(I2C_SCL);
    ssd1306_init(&oled,128,64,false,OLED_ADDR,OLED_I2C_PORT);
    ssd1306_config(&oled);

    // PWM RGB
    gpio_set_function(PIN_RGB_R,GPIO_FUNC_PWM);
    gpio_set_function(PIN_RGB_G,GPIO_FUNC_PWM);
    gpio_set_function(PIN_RGB_B,GPIO_FUNC_PWM);
    sliceR=pwm_gpio_to_slice_num(PIN_RGB_R); chanR=pwm_gpio_to_channel(PIN_RGB_R);
    sliceG=pwm_gpio_to_slice_num(PIN_RGB_G); chanG=pwm_gpio_to_channel(PIN_RGB_G);
    sliceB=pwm_gpio_to_slice_num(PIN_RGB_B); chanB=pwm_gpio_to_channel(PIN_RGB_B);
    pwm_set_wrap(sliceR,65535); pwm_set_wrap(sliceG,65535); pwm_set_wrap(sliceB,65535);
    pwm_set_enabled(sliceR,true); pwm_set_enabled(sliceG,true); pwm_set_enabled(sliceB,true);

    // Cria tasks
    xTaskCreate(Task_Sensor  ,"Sensor" ,256 ,NULL,3,NULL);
    xTaskCreate(Task_Input   ,"Input"  ,512 ,NULL,2,NULL);
    xTaskCreate(Task_Control ,"Control",512 ,NULL,2,NULL);
    xTaskCreate(Task_Display ,"Display",512 ,NULL,1,NULL);
    xTaskCreate(Task_Buzzer  ,"Buzzer" ,256 ,NULL,1,NULL);
    xTaskCreate(Task_Webserver,"WebSrv",1024,NULL,1,NULL);

    vTaskStartScheduler();
    for(;;) tight_loop_contents();
    return 0;
}
