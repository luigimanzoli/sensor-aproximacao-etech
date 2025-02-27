#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <math.h>

// Bibliotecas referentes à configuração do aproximacao
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// Arquivo .pio
#include "aproximacao.pio.h"

// Número de LEDs da matriz
#define NUM_PIXELS 25

// Pino de saída da matriz de LEDs
#define OUT_PIN 7

// Definição dos pinos ADC

const int XAXIS = 26; const int ADCC_0 = 0; // Pino do eixo X do Joystick e seu canal correspondente
const int YAXIS = 27; const int ADCC_1 = 1; // Pino do eixo Y do Joystick e seu canal correspondente

// Definição dos pinos PWM

const float PWM_DIVISER = 16.0; // Definindo o Divisor do PWM
const uint16_t PERIOD = 2048; // Definindo o WRAP ou máxima contagem do PWM
uint16_t R_LED_level, G_LED_level = 100;
uint R_LED_slice, G_LED_slice;

// Definição dos LEDs RGB
#define R_LED 13
#define G_LED 11
#define B_LED 12

// Definição dos botões
#define A_BUTTON 5
#define B_BUTTON 6
#define J_BUTTON 22 // Botão do Joystick

// Definição dos pinos do Buzzer
#define A_BUZZER 21
#define B_BUZZER 10

// Variável ligada ao debounce dos botões
static volatile uint32_t last_time = 0; 

// Variável para controlar o estado do LED
static volatile bool led_state = true; 

// Inicializa a estrutura do aproximacao
ssd1306_t ssd; 

// Inicialização dos lEDs e Botões
void init_all() {

    gpio_init(A_BUZZER);
    gpio_set_dir(A_BUZZER, GPIO_OUT);

    gpio_init(B_BUZZER);
    gpio_set_dir(B_BUZZER, GPIO_OUT);
}

void adc_setup(){

    adc_init();
    adc_gpio_init(XAXIS); // Inicialização do ADC do pino 26
    adc_gpio_init(YAXIS); // Inicialização do ADC do pino 27

}

void pwm_setup(){
    gpio_set_function(R_LED, GPIO_FUNC_PWM); 
    uint R_LED_slice = pwm_gpio_to_slice_num(R_LED);   
    pwm_set_clkdiv(R_LED_slice, PWM_DIVISER);            
    pwm_set_wrap(R_LED_slice, PERIOD);  
    pwm_set_gpio_level(R_LED, R_LED_level);            
    pwm_set_enabled(R_LED_slice, true); 
    
    gpio_set_function(G_LED, GPIO_FUNC_PWM); 
    uint G_LED_slice = pwm_gpio_to_slice_num(G_LED);   
    pwm_set_clkdiv(G_LED_slice, PWM_DIVISER);            
    pwm_set_wrap(G_LED_slice, PERIOD);
    pwm_set_gpio_level(G_LED, G_LED_level);              
    pwm_set_enabled(G_LED_slice, true);
}

void get_buzzer(bool state){
    gpio_put(A_BUZZER, state);
    gpio_put(B_BUZZER, state);
}

// Função que é chamada quando ocorre a interrupção
void gpio_irq_handler(uint gpio, uint32_t events){

    // Definição da variável do tempo atual do sistema dês do começo
    uint32_t current_time = to_us_since_boot(get_absolute_time());
        if (current_time - last_time > 200000){
            last_time = current_time;
            
            if (gpio == A_BUTTON){

                led_state = !led_state; // Altera o estado do LED
                if (led_state == false){
                    pwm_set_gpio_level(R_LED, 0); 
                    pwm_set_gpio_level(B_LED, 0);
                }

            }
            else if (gpio == J_BUTTON){

                gpio_put(G_LED, !gpio_get(G_LED)); // Alterna o estado do LED verde
                printf("Estado do LED Verde Alternado.\n");

            }
    }

}

uint32_t matrix_rgb(double b, double r, double g)
{
  unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

// Inicialização do PIO e das variáveis necessárias
PIO pio = pio0;
uint32_t led_value;
uint offset, sm;

// Configura a PIO
void pio_config(PIO pio, uint *offset, uint *sm) {
    *offset = pio_add_program(pio, &aproximacao_program);
    *sm = pio_claim_unused_sm(pio, true);
    aproximacao_program_init(pio, *sm, *offset, OUT_PIN);
}

void clear_matrix(PIO pio, uint sm){

    // Iniciando a variável que detém informação das cores de cada LED da matriz
    uint32_t led_value;

    // Condição para que os valores não ultrapassem o intervalor desejado
    for (int16_t i = 0; i < NUM_PIXELS; i++) {
        // Coloca valor 0 em todos os LEDs
        led_value = matrix_rgb(0.0, 0.0, 0.0);
        pio_sm_put_blocking(pio, sm, led_value); // Envia o valor para a matriz
    }
   
}

void alarm_animation(PIO pio, uint sm, double r, double g, double b){

    double frame[25] = {
        0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0
    };

    // Iniciando a variável que detém informação das cores de cada LED da matriz
    uint32_t led_value;

    double intensity_values[5] = {1.0, 0.7, 0.5, 0.3, 0.0};
    double inty = 0;

    for (int j = 0; j < 5; j++){
        inty = intensity_values[j];
        for (int16_t i = 0; i < NUM_PIXELS; i++) {
            led_value = matrix_rgb(inty*b*(frame[24 - i]), inty*r*(frame[24 - i]), inty*g*(frame[24 - i])); // Apenas o valor vermelho está ativo
            pio_sm_put_blocking(pio, sm, led_value); // Envia o valor para o LED
        }
        sleep_ms(100-20*j);
    }

}

void alarm(){
    get_buzzer(1);
    alarm_animation(pio, sm, 1, 0, 0);   
}

void clear_alarm(){
    get_buzzer(0);
    clear_matrix(pio, sm);
}

// Função principal
int main() {
    // Inicializa clock, stdio e configurações
    stdio_init_all();
    init_all();
    adc_setup();
    pwm_setup();

    pio_config(pio, &offset, &sm);

    printf("Sistema inicializado.\n");

    clear_matrix(pio, sm);

    // Inicialização do I2C. Usando em 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Configura o pino do GPIO para I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Configura o pino do GPIO para I2C
    gpio_pull_up(I2C_SDA); // Ativa um resistor Pull Up para linha de data
    gpio_pull_up(I2C_SCL); // Ativa um resistor Pull Up para linha de clock
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o aproximacao
    ssd1306_config(&ssd); // Configura o aproximacao
    ssd1306_send_data(&ssd); // Envia os dados para o aproximacao

    // Limpa o aproximacao. O aproximacao inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    int x_value, y_value = 0; // Inicializa as variáveis do valor dos eixos do joystick

    int dx, dy = 0; // Inicializa as variáveis de controle da posição do quadrado do aproximacao

    int TAM = 8; // Define o tamanho do quadrado do aproximacao

    while (true) {
        
        adc_select_input(ADCC_0); // Seleciona o canal de conversor 0
        sleep_us(10);
        x_value = adc_read(); // Armazena o valor na variável do eixo x

        adc_select_input(ADCC_1); // Seleciona o canal de conversor 1
        sleep_us(10);
        y_value = adc_read(); // Armazena o valor na variável do eixo y

        // Imprime os valores no terminal para melhor visualização 
        printf("x_value = %i y_value = %i \n", x_value, y_value);

        // Caso o estado do LED seja ativo, controla sua intensidade 
        
            if (x_value > 2048 && x_value < 2730){
                //pwm_set_gpio_level(B_LED, x_value-2048);
                pwm_set_gpio_level(G_LED, 4096);
                pwm_set_gpio_level(R_LED, (x_value-2048)*3);
                clear_alarm();
            }
            else if (x_value > 2730 && x_value < 3413){
                pwm_set_gpio_level(G_LED, 4096 - (x_value-2048)*3);
                pwm_set_gpio_level(R_LED, 4096);
                clear_alarm();
            }
            else if (x_value > 3413){
                pwm_set_gpio_level(G_LED, 0);
                pwm_set_gpio_level(R_LED, 4096);
                alarm();
            }
            else if (x_value < 2048 && x_value > 1420){
                //pwm_set_gpio_level(B_LED, x_value-2048);
                pwm_set_gpio_level(G_LED, 4096);
                pwm_set_gpio_level(R_LED, (2048-x_value)*3);
                clear_alarm();
            }
            else if (x_value < 1420 && x_value > 511){
                pwm_set_gpio_level(G_LED, 4096 - (2048-x_value)*3);
                pwm_set_gpio_level(R_LED, 4096);
                clear_alarm();
            }
            else if (x_value < 511){
                pwm_set_gpio_level(G_LED, 0);
                pwm_set_gpio_level(R_LED, 4096);
                alarm();
            }
        
        // Definição dos valores de posição para que não ultrapassem o valor máximo do aproximacao
        dx = 60-(x_value*60/4096)-(TAM/2);
        dy = y_value*120/4096;

        // Imprime os valores no terminal para melhor visualização 
        printf("dx = %i, dy = %i \n", dx, dy);

        // Apaga o display
        ssd1306_fill(&ssd, false);

        // Desenha os limites superiores e inferiores respeitando as condicionais
        ssd1306_hline(&ssd, 0, 128, 3, 1);
        ssd1306_hline(&ssd, 0, 128, 7, 1);
        ssd1306_hline(&ssd, 0, 128, 18, 1);

        ssd1306_hline(&ssd, 0, 128, 61, 1);
        ssd1306_hline(&ssd, 0, 128, 57, 1);
        ssd1306_hline(&ssd, 0, 128, 46, 1);

        ssd1306_rect(&ssd, 3, 0, 128, 58, 1, 0);

        // Condicional para que não coloque valores negativos na função
        if (dx < 0){
            // Desenho do objeto em movimento
            ssd1306_rect(&ssd, -dx, dy, TAM, TAM, 1, 1);
        }
        else{
            // Desenho do objeto em movimento
            ssd1306_rect(&ssd, dx, dy, TAM, TAM, 1, 1);
        }
        ssd1306_send_data(&ssd); // Manda a informação para o display

        sleep_ms(10);
    }
    return 0;

}
