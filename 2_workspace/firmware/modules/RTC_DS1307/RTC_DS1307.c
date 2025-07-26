#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17

#define DS1307_ADDRESS 0x68

// Convierte BCD a binario
static uint8_t bcd_to_bin(uint8_t val) {
    return (val & 0x0F) + ((val >> 4) * 10);
}

// Estructura para almacenar la hora
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint16_t year;  // El DS1307 no guarda el año exacto
} rtc_time_t;

bool get_time_from_rtc(rtc_time_t *time) {
    uint8_t buffer[7];
    uint8_t start_reg = 0x00;

    // Escribe la dirección de registro de inicio
    if (i2c_write_blocking(i2c0, DS1307_ADDRESS, &start_reg, 1, true) != 1) {
        return false;
    }

    // Leer 7 bytes de tiempo
    if (i2c_read_blocking(i2c0, DS1307_ADDRESS, buffer, 7, false) != 7) {
        return false;
    }

    time->seconds = bcd_to_bin(buffer[0] & 0x7F);
    time->minutes = bcd_to_bin(buffer[1]);
    time->hours   = bcd_to_bin(buffer[2] & 0x3F);  // Asume formato 24h
    time->day     = bcd_to_bin(buffer[3]);
    time->date    = bcd_to_bin(buffer[4]);
    time->month   = bcd_to_bin(buffer[5]);
    time->year    = 2000 + bcd_to_bin(buffer[6]);  // Año estimado (no exacto)

    return true;
}




int main() {
    stdio_init_all();

    // Inicializa I2C0 en GPIO 16 (SDA), GPIO 17 (SCL)
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    rtc_time_t now;

    while (true) {
        if (get_time_from_rtc(&now)) {
            printf("Hora: %02d:%02d:%02d - %02d/%02d/%04d\n",
                   now.hours, now.minutes, now.seconds,
                   now.date, now.month, now.year);
        } else {
            printf("Error al leer desde el RTC\n");
        }
        sleep_ms(1000);
    }
}