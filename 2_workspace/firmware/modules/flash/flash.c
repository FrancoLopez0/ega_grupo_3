#include "flash.h"

static uint8_t aux_buff[FLASH_PAGE_SIZE];

flash_err_t read_page(uint16_t page_num, uint8_t *buff){
    const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(buff, flash_target_contents + page_num * FLASH_PAGE_SIZE, FLASH_PAGE_SIZE);
    return FLASH_OK;
}

flash_err_t erase_sector(uint16_t sector_num){
    uint32_t ints = save_and_disable_interrupts();
    uint16_t start_page = sector_num * 16;
    flash_range_erase(FLASH_TARGET_OFFSET + (sector_num * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

    for(int j = 0; j < 16; j++){
        read_page(start_page + j, aux_buff);
    
        for(int i=0; i<FLASH_PAGE_SIZE; i++){
            if(aux_buff[i] != 0xFF){
                return FLASH_ERASE_ERROR;
            }
        }
    }

    return FLASH_OK;
}

flash_err_t append_data_to_page(uint16_t page_num, size_t end,uint8_t *data, size_t data_size){
    uint32_t ints = save_and_disable_interrupts();
    uint32_t target_offset = FLASH_TARGET_OFFSET + (page_num * FLASH_PAGE_SIZE) + end;
    flash_range_program(target_offset, data, data_size);
    restore_interrupts(ints);
    return FLASH_OK;
}

flash_err_t write_page(uint16_t page_num, uint8_t *buff){
    uint32_t ints = save_and_disable_interrupts();
    uint32_t target_offset = FLASH_TARGET_OFFSET + (page_num * FLASH_PAGE_SIZE);
    flash_range_program(target_offset, buff, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    
    read_page(page_num, aux_buff);
    
    if(memcmp(buff, aux_buff, FLASH_PAGE_SIZE) != 0){
        return FLASH_WRITE_ERROR;
    }
    return FLASH_OK;
}


// void save_log(const char *entry) {
//     uint32_t ints = save_and_disable_interrupts();

//     // Leer la flash existente
//     const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
//     memcpy(log_buffer, flash_target_contents, sizeof(log_buffer));

//     // Buscar próxima entrada vacía
//     int i;
//     for (i = 0; i < MAX_LOG_ENTRIES; ++i) {
//         if (log_buffer[i * ENTRY_SIZE] == 0xFF) break;  // 0xFF indica espacio vacío
//     }

//     // Si está llena, borrar y empezar desde cero
//     if (i == MAX_LOG_ENTRIES) {
//         memset(log_buffer, 0xFF, sizeof(log_buffer));  // Limpia
//         i = 0;
//     }

//     // Escribe la nueva entrada en la posición encontrada
//     strncpy(&log_buffer[i * ENTRY_SIZE], entry, ENTRY_SIZE - 1);

//     // Borra la flash
//     flash_range_erase(FLASH_TARGET_OFFSET, 4096);

//     // Escribe el nuevo buffer completo
//     flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t *)log_buffer, sizeof(log_buffer));

//     restore_interrupts(ints);
// }

// uint16_t log_buffer_16_t[4];

// void save_u16_as_bytes(uint16_t value, int index) {
//     uint32_t ints = save_and_disable_interrupts();

//     // Leer toda la página de flash
//     const uint8_t *flash_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
//     memcpy(log_buffer, flash_contents, sizeof(log_buffer));

//     // Convertir uint16_t en dos bytes
//     log_buffer[index * 2]     = value & 0xFF;        // byte bajo
//     log_buffer[index * 2 + 1] = (value >> 8) & 0xFF; // byte alto

//     // Borrar y reprogramar
//     flash_range_erase(FLASH_TARGET_OFFSET, FLASH_PAGE_SIZE);
//     flash_range_program(FLASH_TARGET_OFFSET, log_buffer, sizeof(log_buffer));

//     restore_interrupts(ints);
// }

// void read_all_logs() {
//     const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
//     for (int i = 0; i < MAX_LOG_ENTRIES; ++i) {
//         const char *entry = (const char *)&flash_target_contents[i * ENTRY_SIZE];
//         if (entry[0] == 0xFF) break;  // Fin de datos
//         printf("LOG[%d]: %s\n", i, entry);
//     }
// }

// uint16_t read_log_u16(int i) {
//     const uint16_t *flash_data = (const uint16_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

//     if (i < 0 || i >= MAX_LOG_ENTRIES) {
//         return 0xFFFF; // índice inválido
//     }

//     return flash_data[i];
// }

// void read_log(uint32_t num_log, char *log){
//     const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
//     const char *entry = (const char *)&flash_target_contents[num_log * ENTRY_SIZE];
//     strcpy(log, entry);
// }

// void erase_all_logs(void) {

//     uint32_t ints = save_and_disable_interrupts();
//     flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
//     restore_interrupts(ints);

// }

// void over_write(const uint8_t *empty_data, size_t page_size) {
    
//     uint32_t ints = save_and_disable_interrupts();
//     flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
//     flash_range_program(FLASH_TARGET_OFFSET, empty_data, page_size); // Por si me pinta rellenarlo con ceros o "espacios vacios"
//     restore_interrupts(ints);
// }
