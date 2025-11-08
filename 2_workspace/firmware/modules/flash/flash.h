#include "hardware/flash.h"
#include "hardware/sync.h"
#include "stdio.h"
#include <string.h>

#define DATA_BLOCK_SIZE 64*1024
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - DATA_BLOCK_SIZE)  // Último sector de 4 KB de la flash
#define MAX_LOG_ENTRIES 128
#define ENTRY_SIZE 64  // Tamaño fijo por entrada para facilidad
#define MAX_PAGES (DATA_BLOCK_SIZE / FLASH_PAGE_SIZE)
#define DATA_SEP 0xDEAD
#define STATE_BLOCK_SIZE 12 // Tamaño del bloque de datos sin contar el separador

typedef enum{
    FLASH_OK,
    FLASH_WRITE_ERROR,
    FLASH_READ_ERROR,
    FLASH_ERASE_ERROR,
    FLASH_PAGE_FULL
}flash_err_t;

extern char __flash_binary_end;

static char log_buffer[MAX_LOG_ENTRIES * ENTRY_SIZE] __attribute__((aligned(4)));

flash_err_t read_page(uint16_t page_num, uint8_t *buff);

flash_err_t erase_sector(uint16_t sector_num);

flash_err_t write_page(uint16_t page_num, uint8_t *buff);

flash_err_t append_data_to_page(uint16_t page_num, size_t end,uint8_t *data, size_t data_size);

// void read_all_logs();

// void read_log(uint32_t num_log, char *log);

// void save_log(const char *entry);

// void erase_all_logs(void);

// void over_write(const uint8_t *empty_data, size_t page_size);

// void save_log_on(const char *entry, int i);

// uint16_t read_log_u16(int i);

// void save_u16_as_bytes(uint16_t value, int i);