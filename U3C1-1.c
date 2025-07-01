#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h" // Utilizada para se comunicar com o sensor BH1750

// Configurações de GPIOs e endereços
#define PINO_SERVO 16     // Pino do Servo 

#define LED_R_PIN 13  // LED vermelho
#define LED_G_PIN 11  // LED verde
#define LED_B_PIN 12  // LED azul

#define I2C_PORT i2c0
#define SDA_PIN 0      // GPIO4 para SDA
#define SCL_PIN 1      // GPIO5 para SCL

#define BH1750_ADDR 0x23 // Endereço do BH1750
#define BH1750_CMD_START 0x10  // Medição contínua, alta resolução

// Duty Cycle em microssegundos
#define DUTY_MIN 1000      // Pulso de 1 ms (0°)
#define DUTY_MID 1500      // Pulso de 1.5 ms (90°)
#define DUTY_MAX 2000      // Pulso de 2 ms (180°)
#define PERIODO_SERVO 20   // Período do PWM do servo em ms

// Inicializa os LEDs
void init_leds() {
    gpio_init(LED_R_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_put(LED_R_PIN, 0);

    gpio_init(LED_G_PIN);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_put(LED_G_PIN, 0);

    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);
    gpio_put(LED_B_PIN, 0);
}

// Envia comando ao BH1750
void bh1750_start_measurement() {
    uint8_t cmd = BH1750_CMD_START;
    i2c_write_blocking(I2C_PORT, BH1750_ADDR, &cmd, 1, false);
}

// Lê o valor da luminosidade do BH1750
uint16_t bh1750_read_lux() {
    uint8_t data[2];
    i2c_read_blocking(I2C_PORT, BH1750_ADDR, data, 2, false);
    return (data[0] << 8) | data[1];
}

// Controla os LEDs
void set_leds(bool red, bool green, bool blue) {
    gpio_put(LED_R_PIN, red);
    gpio_put(LED_G_PIN, green);
    gpio_put(LED_B_PIN, blue);
}


// Função para enviar pulsos PWM manualmente ao servo motor
void enviar_pulso_servo(uint duty_us) {
    gpio_put(PINO_SERVO, 1);                  // Sinal alto
    sleep_us(duty_us);                        // Mantém o sinal alto por 'duty_us'
    gpio_put(PINO_SERVO, 0);                  // Sinal baixo
    sleep_ms(PERIODO_SERVO - (duty_us / 1000));  // Aguarda até completar o período (20 ms)
}

// Ajusta o servo motor com base na luminosidade (lux)
void ajustar_servo(uint16_t lux) {
    uint duty_us;

    if (lux < 100.0) {
        duty_us = DUTY_MAX;  // Janela totalmente aberta
        printf("Servo: Janela totalmente aberta\n");
    } else if (lux < 500.0) {
        duty_us = DUTY_MID;  // Janela semiaberta
        printf("Servo: Janela semiaberta\n");
    } else {
        duty_us = DUTY_MIN;  // Janela fechada
        printf("Servo: Janela fechada\n");
    }

    // Envia pulsos para estabilizar o servo
    for (int i = 0; i < 50; i++) {
        enviar_pulso_servo(duty_us);
    }
}

int main() {
    // Inicializa UART para debug (opcional)
    stdio_init_all();

    // Inicializa LEDs
    init_leds();

    // Inicializa I2C para o BH1750
    i2c_init(I2C_PORT, 100 * 1000);  // Frequência de 100 kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Inicializa o servo
    gpio_init(PINO_SERVO);
    gpio_set_dir(PINO_SERVO, GPIO_OUT);

    // Configura o BH1750 para começar a medir
    bh1750_start_measurement();

    while (true) {
        uint16_t lux = bh1750_read_lux();
        printf("Luminosidade: %u lux\n", lux);

        // Atualiza os LEDs com base na luminosidade
        if (lux < 100) {
            set_leds(1, 0, 0);  // LED vermelho aceso
        } else if (lux < 500) {
            set_leds(0, 1, 0);  // LED verde aceso
        } else {
            set_leds(0, 0, 1);  // LED azul aceso
        }

        ajustar_servo(lux);

        sleep_ms(1000);  // Aguarda 1 segundo para a próxima interação
    }

    return 0;
}
