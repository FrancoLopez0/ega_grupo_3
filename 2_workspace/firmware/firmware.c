/**
 * @file firmware.c
 * @author Franco Lopez & Matias Pietras (francoalelopez@gmail.com mati_pietras@yahoo.com.ar)
 * @brief 
 * @version 0.1
 * @date 2025-06-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#include "modules/temt6000/temt6000.h"
#include "modules/bh1750/bh1750.h"
#include "modules/ui/ui.h"

#define I2C_FREQ        400*1000
#define PIN_TEMT6000    28
#define CHANN_TEMT6000  2

user_t user = {
    .sp = MAX_SP/2,
    .mode = true,
    .lux = MAX_LUX/2,
    .select = set_sp
};

void i2c_config(void){
    i2c_init(I2C_PORT, I2C_FREQ);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void adc_config(void){
    adc_init();
    adc_gpio_init(PIN_TEMT6000);
    adc_select_input(CHANN_TEMT6000);
}

int main() {
    stdio_init_all();

    adc_config();
    i2c_config();
    bh1750_init();

    ssd1306_t oled;
    oled.external_vcc = false;
    ui_init(&oled, I2C_PORT, &user);

    uint8_t c = 0;
    
    while (1) {
        printf("Lux TEMT6000:%f\n", temt6000_get_lux());
        uint16_t lux = bh1750_read_lux();
        printf("Luz BH1750: %u lux\n", lux);
        user.lux=lux;
        // user.sp = (user.sp==0) ? MAX_SP:(user.sp - 1);

        ui_update(&oled, &user);
        
        // user.select = (c==20) ? (user.select + 1)%(not_show) : user.select;
        // c = (c+1)%21;

        sleep_ms(150);
    }
}

