#include "tmt6000.h"

/// @brief Calcula el voltaje a partir de la medicion del adc
/// @param adc 
/// @return voltaje
float adc_get_voltage(uint16_t adc){
    return ((float)adc / ADC_RESOLUTION)*V_REF;
}

/// @brief Obtiene la corriente a partir del voltage
/// @param voltage 
/// @return corriente sobre la resistencia de sensado
float tmt600_get_current(float voltage){
    return voltage/RESISTOR;
}

/// @brief Obtiene los lux
/// @param voltage 
/// @return lux
float tmt6000_get_lux(void){
    uint16_t adc_raw = adc_read();
    float voltage = adc_get_voltage(adc_raw);
    float current = tmt6000_get_current(voltage);
    return current * DEFAULT_CTE_LUX;
}