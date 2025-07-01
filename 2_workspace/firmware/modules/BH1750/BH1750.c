#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"


#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17
#define BH1750_ADDR 0x23

void bh1750_init() {
    // Prendo el bh1750 y le fijo la resolucion
    uint8_t config[] = {0x10};  // 1 lux de resolucion, con 120ms de muestreo
    i2c_write_blocking(I2C_PORT, BH1750_ADDR, config, 1, false);
}

uint16_t bh1750_read_lux() {
    uint8_t buf[2];
    i2c_read_blocking(I2C_PORT, BH1750_ADDR, buf, 2, false);
    uint16_t lux = (buf[0] << 8) | buf[1];
    // Divido por 1.2 para convertir en Lux
    return lux;
}

int main()
{
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);


    // Prendo el led por que me gusta prender leds
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    bh1750_init();
    sleep_ms(200); // tiempo de inicio

    while (1) {
        uint16_t lux = bh1750_read_lux();
        printf("Luz: %u lux\n", lux);
        sleep_ms(1000);
    }
}
