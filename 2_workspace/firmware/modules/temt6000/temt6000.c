#include "temt6000.h"

static float g_conversion_factor = 3.3f / (1 << 12);

static float calib = 3300.0f;

float temt6000_set_calib(float set_calib){
    calib = set_calib;
    return calib;
}

/**
 * @brief Inicializa el adc en el pin del temt6000
 * 
 * @param pin 
 * @param chann 
 */
void temt6000_init(uint8_t pin, uint8_t chann){
    adc_init(); 
    adc_gpio_init(pin);
    adc_select_input(pin);
}

/**
 * @brief Calcula el voltaje a partir de la medicion del adc
 * 
 * @param adc 
 * @return float voltaje
 */
float adc_get_voltage(uint16_t adc_raw){
    return (float)adc_raw * (3.3f / (1 << 12));
}

/**
 * @brief Obtiene la corriente a partir del voltage
 * 
 * @param voltage 
 * @return float corriente sobre la resistencia de sensado
 */
float temt6000_get_current(float voltage){
    return (voltage/RESISTOR)*1000;
}

float temt6000_get_raw_lux(uint16_t adc_raw){
    return ((float)adc_raw / 4095.0);
}

/**
 * @brief Obtiene los lux
 * @param voltage
 * @return float lux
 */
float temt6000_get_lux(uint16_t adc_raw){
    // float voltage = adc_get_voltage(adc_raw);
    // float current = temt6000_get_current(voltage);
    // return current * DEFAULT_CTE_LUX;
    return calib * ((float)adc_raw / 4095.0);
}