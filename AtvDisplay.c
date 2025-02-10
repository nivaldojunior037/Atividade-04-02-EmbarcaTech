// Definição das biliotecas a serem utilizadas na execução
#include <stdio.h> 
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "debouncing_matrix.pio.h" 

#define I2C_PORT i2c1
#define I2C_SDA 14          //Definição da porta GPIO da comunicação I2C (DADOS)
#define I2C_SCL 15          //Definição da porta GPIO da comunicação I2C (CLOCK)
#define endereco 0x3c       //Definição do endereço do display ssd1306
#define NUM_PIXELS 25       //Definição do número de LEDs da matriz 5X5
#define LEDS_PIN 7          //Definição da porta GPIO da matriz de LEDs 5X5
#define LED_PIN_GREEN 11    //Definição da porta GPIO do led verde do conjunto RGB 
#define LED_PIN_BLUE 12     //Definição da porta GPIO do led azul do conjunto RGB
#define BUTTON_A 5          //Definição da porta GPIO do botão A
#define BUTTON_B 6          //Definição da porta GPIO do botão B
#define UART_ID uart0       //Seleciona a UART0
#define BAUD_RATE 115200    //Define a taxa de transmissão
#define UART_TX_PIN 0       //Pino GPIO usado para TX
#define UART_RX_PIN 1       //Pino GPIO usado para RX

// Definição de todos os contadores, flags, variáveis e estruturas que serão utilizadas de forma global 
static volatile bool ledb = false;
static volatile bool ledg = false;
static volatile bool botaoa_press = false;
static volatile bool botaob_press = false;
static volatile uint32_t ultimo_tempo = 0;
static volatile char ch = '\0'; 
static volatile int cont = 4;
static volatile bool leitura = false;
static volatile double r = 0.0, b = 0.0, g = 0.0;
static volatile uint32_t valor_led;
static volatile PIO pio = pio0;
static volatile uint offset;
static volatile uint sm;
static ssd1306_t ssd; 
static volatile bool cor = true;

// Vetores usados para criar os desenhos na matriz de LEDs
double numero0[NUM_PIXELS] = {0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0}; 

double numero1[NUM_PIXELS] = {0.0, 0.0, 0.5, 0.0, 0.0,
                              0.0, 0.0, 0.5, 0.0, 0.0,
                              0.0, 0.0, 0.5, 0.0, 0.0,
                              0.0, 0.0, 0.5, 0.0, 0.0,
                              0.0, 0.0, 0.5, 0.0, 0.0}; 

double numero2[NUM_PIXELS] = {0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.0, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.0, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0}; 

double numero3[NUM_PIXELS] = {0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.0, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.0, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0}; 

double numero4[NUM_PIXELS] = {0.0, 0.5, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.5, 0.0}; 

double numero5[NUM_PIXELS] = {0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.0, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.0, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0}; 

double numero6[NUM_PIXELS] = {0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.0, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0}; 

double numero7[NUM_PIXELS] = {0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.5, 0.0}; 

double numero8[NUM_PIXELS] = {0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0}; 

double numero9[NUM_PIXELS] = {0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.5, 0.0,
                              0.0, 0.5, 0.5, 0.5, 0.0,
                              0.0, 0.5, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.5, 0.0};

// Função para converter os parâmetros r, g, b em um valor de 32 bits
uint32_t matrix_rgb(double b, double r, double g)
{
    unsigned char R, G, B;
    R = r * 255;
    G = g * 255;
    B = b * 255;
    return (G << 24) | (R << 16) | (B << 8);
}

// Função para formar os desenhos na matriz de LEDs 5x5
void desenhos(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b)
{
    // O loop aciona cada LED da matriz com base em um valor de cor 
    for (int i = 0; i < NUM_PIXELS; i++){
        // Determinação da cor de cada LED
        uint32_t valor_led = matrix_rgb(desenho[24 - i]*r, desenho[24 - i]*g, desenho[24 - i]*b);
        // O valor é enviado ao LED para ser exibido
        pio_sm_put_blocking(pio, sm, valor_led);
    }
}

// Função que administrará a interrupção
static void gpio_irq_handler(uint gpio, uint32_t events);

// Função que determina qual número será desenhado na matriz de LEDs quando uma tecla numérica for digitada
static void colorirmatriz(char ch){
    switch (ch){
    case '0':
    // Os parâmetros para cada número são definidos 
    b = 1.0;
    r = 1.0;
    g = 1.0;
    // A função de desenho é chamada e os parâmetros são passados a ela
    desenhos(numero0, valor_led, pio, sm, b, r ,g);
    // O caso se encerra e o número fica exposto até uma nova interrupção ser detectada
    break;

    case '1':
    b = 0.5;
    r = 1.0;
    g = 1.0;
    desenhos(numero1, valor_led, pio, sm, b, r ,g);
    break;

    case '2':
    b = 1.0;
    r = 0.5;
    g = 1.0;
    desenhos(numero2, valor_led, pio, sm, b, r ,g);
    break;

    case '3':
    b = 1.0;
    r = 1.0;
    g = 0.5;
    desenhos(numero3, valor_led, pio, sm, b, r ,g);
    break;

    case '4':
    b = 0.5;
    r = 0.5;
    g = 1.0;
    desenhos(numero4, valor_led, pio, sm, b, r ,g);
    break;

    case '5':
    b = 0.5;
    r = 1.0;
    g = 0.5;
    desenhos(numero5, valor_led, pio, sm, b, r ,g);
    break;

    case '6':
    b = 1.0;
    r = 0.5;
    g = 0.5;
    desenhos(numero6, valor_led, pio, sm, b, r ,g);
    break;

    case '7':
    b = 0.0;
    r = 1.0;
    g = 1.0;
    desenhos(numero7, valor_led, pio, sm, b, r ,g);
    break;

    case '8':
    b = 1.0;
    r = 0.0;
    g = 1.0;
    desenhos(numero8, valor_led, pio, sm, b, r ,g);
    break;

    case '9':
    b = 1.0;
    r = 1.0;
    g = 0.0;
    desenhos(numero9, valor_led, pio, sm, b, r ,g);
    break;

    default:
    b = 0.0; 
    r = 0.0;
    g = 0.0;
    desenhos(numero0, valor_led, pio, sm, b, r ,g);

}
}

int main(){

  // Inicialização dos comandos e variáveis necessárias
  stdio_init_all();
  offset = pio_add_program(pio, &debouncing_matrix_program);
  sm = pio_claim_unused_sm(pio, true);
  debouncing_matrix_program_init(pio, sm, offset, LEDS_PIN);

  //Inicialização e configuração da UART
  uart_init(UART_ID, 115200);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART); 
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART); 
  gpio_pull_up(UART_RX_PIN);

  // Inicialização do LED e definição como saída
  gpio_init(LED_PIN_GREEN);
  gpio_set_dir(LED_PIN_GREEN, GPIO_OUT);
  gpio_init(LED_PIN_BLUE);
  gpio_set_dir(LED_PIN_BLUE, GPIO_OUT);

  // Inicialização dos botões A e B, definição como entrada e acionamento do pull-up interno
  gpio_init(BUTTON_A);
  gpio_set_dir(BUTTON_A, GPIO_IN);
  gpio_pull_up(BUTTON_A);
  gpio_init(BUTTON_B);
  gpio_set_dir(BUTTON_B, GPIO_IN);
  gpio_pull_up(BUTTON_B);

  // Interrupção com callback para cada um dos botões 
  gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

  // Inicialização e configuração da I2C
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); 
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); 
  gpio_pull_up(I2C_SDA); 
  gpio_pull_up(I2C_SCL);

  // Inicialização e configuração do display ssd1306
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); 
  ssd1306_config(&ssd); 
  ssd1306_send_data(&ssd); 
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  // A matriz de LEDs é inicialmente acionada sem nenhum desenho
  b = 0.0;
  r = 0.0;
  g = 0.0;
  desenhos(numero0, valor_led, pio, sm, b, r ,g);

  // Loop infinito
  while (true)
  {
    sleep_ms(500);
    // Condicional para a flag que permitirá a "inserção" de teclas no monitor serial
    if(cont == 0 && !leitura){
        cor = !cor;
        // O display é atualizado e é solicitado que uma tecla seja digitada
        ssd1306_fill(&ssd, !cor); 
        ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); 
        ssd1306_draw_string(&ssd, "Digite uma", 24, 20); 
        ssd1306_draw_string(&ssd, "tecla", 44, 36);      
        ssd1306_send_data(&ssd); 
        sleep_ms(50);
        // A flag e o contador são atualizados
        leitura = true;
        cont = 4;
    }
    // Se o botão A é pressionado, o estado do LED é alternado e o contador sofre decremento
    if(botaoa_press){
        gpio_put(LED_PIN_GREEN, ledg);
        botaoa_press = false;
        cont--;
        sleep_ms(1000);
    }
    // Se o botão B é pressionado, o estado do LED é alternado e o contador sofre decremento
    if(botaob_press){
        gpio_put(LED_PIN_BLUE, ledb);
        botaob_press = false;
        cont--; 
        sleep_ms(1000);
    }
    // Se a flag permitir, ocorre a leitura de um caractere no terminal serial
    if(leitura){
        char verif;
        // Se o caractere prévio inserido for válido, ele é passado para a variável principal
        if(scanf(" %c", &verif)==1)
        ch = verif;
        // O caractere inserido é impresso no terminal serial
        printf("Nova entrada: %c\n", ch);  
        cor = !cor;
        // O caractere principal é colocado dentro de uma string para facilirar sua impressão
        char entrada[2] = {ch, '\0'};
        // O display é atualizado e o caractere inserido é impresso no display
        ssd1306_fill(&ssd, !cor); 
        ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); 
        ssd1306_draw_string(&ssd, "Entrada atual", 12, 20); 
        ssd1306_draw_string(&ssd, entrada, 60, 36);      
        ssd1306_send_data(&ssd); 
        // A função para colorir a matriz é chamada
        colorirmatriz(ch);
        sleep_ms(1000);
        // A variável e a flag são atualizadas
        ch = '\0';
        leitura = false;
    } 
    // Quando falta mais de um aperto, são impressas mensagens no monitor serial e no display avisando essa quantidade
    if (cont>=1){
    cor = !cor;
    // A quantidade que falta é inserida em uma string para poder ser impressa
    char aviso[2];
    sprintf(aviso, "%d", cont);
    // O display é atualizado e a mensagem é impressa
    ssd1306_fill(&ssd, !cor); 
    ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); 
    ssd1306_draw_string(&ssd, "Tecle apos", 24, 20); 
    ssd1306_draw_string(&ssd, aviso, 28, 36);
    ssd1306_draw_string(&ssd, "apertos", 44, 36);   
    ssd1306_send_data(&ssd); 
    printf("Digite uma tecla após %d apertos.\n", cont);
    sleep_ms(50);
    }
  }
}

// Função de interrupção com debouncing
static void gpio_irq_handler(uint gpio, uint32_t events){
    // Criação de booleanos para obter o estado de cada botão durante a interrupção
    bool trocaverde = gpio_get(BUTTON_A);
    bool trocaazul = gpio_get(BUTTON_B);

    // Obtenção do tempo em que ocorre a interrupção desde a inicialização
    uint32_t tempo_atual = to_us_since_boot(get_absolute_time());

    // Verificação de alteração em um intervalo maior que 300ms (debouncing)
    if(tempo_atual - ultimo_tempo > 300000){
        // Determinação de qual botão foi ativado 
        if(trocaverde == false && trocaazul == true){
            //Se o botão A foi pressionado, o tempo é atualizado 
            ultimo_tempo = tempo_atual;
            ledg = !ledg;
            printf("Botão A pressionado!\n");
            // A flag é atualizada 
            botaoa_press = true;
            cor = !cor;
            // O display é atualizado e indica que o botão B foi pressionado
            ssd1306_fill(&ssd, !cor); 
            ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); 
            ssd1306_draw_string(&ssd, "Botao A", 36, 20); 
            ssd1306_draw_string(&ssd, "pressionado", 20, 36);      
            ssd1306_send_data(&ssd); 

    } else if (trocaverde == true && trocaazul == false){
        // Se o botão B foi pressionado, o tempo é atualizado 
        ultimo_tempo = tempo_atual;
        ledb = !ledb;
        printf("Botão B pressionado!\n");
        // A flag é atualizada
        botaob_press = true;
        cor = !cor;
        // O display é atualizado e indica que o botão B foi pressionado
        ssd1306_fill(&ssd, !cor); 
        ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); 
        ssd1306_draw_string(&ssd, "Botao B", 36, 20); 
        ssd1306_draw_string(&ssd, "pressionado", 20, 36);      
        ssd1306_send_data(&ssd); 

    }
  }
}
