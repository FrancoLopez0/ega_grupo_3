#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17

#define DS1307_ADDRESS 0x68

QueueHandle_t q_to_storage;
#define BUFF_STORAGE 100


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


#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>

#define FLASH_TARGET_OFFSET (2 * 1024 * 1024 - 4096)  // Último sector de 4 KB de la flash



#define MAX_LOG_ENTRIES 128
#define ENTRY_SIZE 64  // Tamaño fijo por entrada para facilidad

static char log_buffer[MAX_LOG_ENTRIES * ENTRY_SIZE] __attribute__((aligned(4)));

void save_log(const char *entry) {
    uint32_t ints = save_and_disable_interrupts();

    // Leer la flash existente
    const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(log_buffer, flash_target_contents, sizeof(log_buffer));

    // Buscar próxima entrada vacía
    int i;
    for (i = 0; i < MAX_LOG_ENTRIES; ++i) {
        if (log_buffer[i * ENTRY_SIZE] == 0xFF) break;  // 0xFF indica espacio vacío
    }

    // Si está llena, borrar y empezar desde cero
    if (i == MAX_LOG_ENTRIES) {
        memset(log_buffer, 0xFF, sizeof(log_buffer));  // Limpia
        i = 0;
    }

    // Escribe la nueva entrada en la posición encontrada
    strncpy(&log_buffer[i * ENTRY_SIZE], entry, ENTRY_SIZE - 1);

    // Borra la flash
    flash_range_erase(FLASH_TARGET_OFFSET, 4096);

    // Escribe el nuevo buffer completo
    flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t *)log_buffer, sizeof(log_buffer));

    restore_interrupts(ints);
}

void read_all_logs() {
    const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    for (int i = 0; i < MAX_LOG_ENTRIES; ++i) {
        const char *entry = (const char *)&flash_target_contents[i * ENTRY_SIZE];
        if (entry[0] == 0xFF) break;  // Fin de datos
        printf("LOG[%d]: %s\n", i, entry);
    }
}

void erase_all_logs(void) {

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

}

void over_write(const uint8_t *empty_data, size_t page_size) {
    
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, empty_data, page_size); // Por si me pinta rellenarlo con ceros o "espacios vacios"
    restore_interrupts(ints);
}

void storage_task(void *params) {

    //erase_all_logs();

    rtc_time_t now;
    int data_value = 12345;  // Dato que voy a guardar
    
    char log_entry[ENTRY_SIZE];

    while (true) {
        
        xQueueReceive(q_to_storage, &data_value, portMAX_DELAY);

        if (get_time_from_rtc(&now)) {
            // printf("Hora: %02d:%02d:%02d - %02d/%02d/%04d\n",
            //        now.hours, now.minutes, now.seconds,
            //        now.date, now.month, now.year);
            snprintf(log_entry, sizeof(log_entry), "%02d:%02d-%02d/%02d: %d",now.hours, now.minutes, now.date, now.month, data_value);
            //snprintf(log_entry, sizeof(log_entry), "12:00-25/12: %d",data_value);  //Harcode
           
            save_log(log_entry);
        } else {
            printf("Error al leer desde el RTC\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
        read_all_logs();
    }
}

void generador(void *params) {

int dato = 10527;
while (true) {
    xQueueSend(q_to_storage, &dato, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

static uint8_t bin_to_bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

bool set_time_to_rtc(const rtc_time_t *time) {
    uint8_t buffer[8];

    buffer[0] = 0x00; // Dirección inicial de escritura en el RTC
    buffer[1] = bin_to_bcd(time->seconds);
    buffer[2] = bin_to_bcd(time->minutes);
    buffer[3] = bin_to_bcd(time->hours);        // Modo 24h
    buffer[4] = bin_to_bcd(time->day);          // Día de la semana (1=Dom, 7=Sáb)
    buffer[5] = bin_to_bcd(time->date);         // Día del mes
    buffer[6] = bin_to_bcd(time->month);
    buffer[7] = bin_to_bcd(time->year % 100);   // Solo los últimos 2 dígitos del año

    int written = i2c_write_blocking(I2C_PORT, DS1307_ADDRESS, buffer, 8, false);
    return written == 8;
}

int main() {
    stdio_init_all();

    // Inicializa I2C0 en GPIO 16 (SDA), GPIO 17 (SCL)
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

   
    q_to_storage = xQueueCreate(BUFF_STORAGE, sizeof(int));

    xTaskCreate(storage_task, "storage_task", configMINIMAL_STACK_SIZE*2, NULL, 2, NULL);
    xTaskCreate(generador, "generador", configMINIMAL_STACK_SIZE*2, NULL, 2, NULL);

    rtc_time_t t = {
    .seconds = 30,
    .minutes = 45,
    .hours = 15,
    .day = 1,
    .date = 25,
    .month = 7,
    .year = 2025
    };

    set_time_to_rtc(&t);
    

   vTaskStartScheduler();
   while(1);
}



// int main() {
//     stdio_init_all();

//     // Inicializa I2C0 en GPIO 16 (SDA), GPIO 17 (SCL)
//     i2c_init(I2C_PORT, 400 * 1000);
//     gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
//     gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
//     gpio_pull_up(I2C_SDA);
//     gpio_pull_up(I2C_SCL);

//     rtc_time_t now;

//     int data_value = 12345;  // Ejemplo: lo que quieras registrar
//     get_time_from_rtc(&now);
//     char log_entry[ENTRY_SIZE];
//     snprintf(log_entry, sizeof(log_entry), "%02d:%02d-%02d/%02d: %d",
//           now.hours, now.minutes, now.date, now.month, data_value);

//     //snprintf(log_entry, sizeof(log_entry), "12:00-25/12: %d",data_value);
//     save_log(log_entry);
//     save_log(log_entry);

//     read_all_logs();

//     while (true) {
//         if (get_time_from_rtc(&now)) {
//             printf("Hora: %02d:%02d:%02d - %02d/%02d/%04d\n",
//                    now.hours, now.minutes, now.seconds,
//                    now.date, now.month, now.year);
//         } else {
//             printf("Error al leer desde el RTC\n");
//         }
//         sleep_ms(1000);
//         read_all_logs();
//     }
// }