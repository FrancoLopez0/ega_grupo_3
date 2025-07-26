
''
void erase_all_logs(void) {
    // const uint8_t empty_data[FLASH_PAGE_SIZE] = {0};  // Un bloque de ceros

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);

    // flash_range_program(FLASH_TARGET_OFFSET, empty_data, FLASH_PAGE_SIZE); // Por si me pinta rellenarlo con ceros o "espacios vacios"
    restore_interrupts(ints);
}
''

''
void over_write(void) {
    const uint8_t empty_data[FLASH_PAGE_SIZE] = {0};  // Un bloque de ceros
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, empty_data, FLASH_PAGE_SIZE); // Por si me pinta rellenarlo con ceros o "espacios vacios"
    restore_interrupts(ints);
}
''
