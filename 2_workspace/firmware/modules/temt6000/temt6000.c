#include "temt6000.h"

/**
 * @brief Calcula el voltaje a partir de la medicion del adc
 * 
 * @param adc 
 * @return float voltaje
 */
float adc_get_voltage(uint16_t adc_raw){
    return ((float)adc_raw / ADC_RESOLUTION)*V_REF;
}

/**
 * @brief Obtiene la corriente a partir del voltage
 * 
 * @param voltage 
 * @return float corriente sobre la resistencia de sensado
 */
float temt6000_get_current(float voltage){
    return voltage/RESISTOR;
}

/**
 * @brief Obtiene los lux
 * @param voltage
 * @return float lux
 */
float temt6000_get_lux(void){
    uint16_t adc_raw = adc_read();
    float voltage = adc_get_voltage(adc_raw);
    float current = temt6000_get_current(voltage);
    return current * DEFAULT_CTE_LUX;
}