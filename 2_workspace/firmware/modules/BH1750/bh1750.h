#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17
#define BH1750_ADDR 0x23

/**
 * @brief Inicializa el bh1750
 * 
 */
void bh1750_init(void);

/**
 * @brief Obtiene el valor en lux
 * 
 * @return uint16_t lux
 */
uint16_t bh1750_read_lux(void);