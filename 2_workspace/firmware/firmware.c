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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define I2C_FREQ        400*1000
#define PIN_TEMT6000    28
#define CHANN_TEMT6000  2

#define BUFF_SIZE 100
#define SAMPLE_RATE 10
#define PERIOD_RATE 1000/SAMPLE_RATE
#define ADC_CLK_BASE 48000000.f
#define CONVERSION_FACTOR 3.3f / (1 << 12)

QueueHandle_t q_lux_values, q_raw_adc_values;
SemaphoreHandle_t semphr_i2c;
ssd1306_t oled;

user_t user = {
    .sp = MAX_SP/2,
    .mode = true,
    .lux = MAX_LUX/2,
    .select = set_sp
};

void moving_average_filter(float* buffer, int buff_size, float* sum, int* index, float new_value, float* avg) {
    // Obtener el índice del valor más antiguo antes de actualizar el buffer
    int old_index = *index;

    // Restar el valor antiguo del buffer de la suma acumulada
    *sum -= buffer[old_index];

    // Agregar el nuevo valor al buffer y actualizar la suma acumulada
    buffer[old_index] = new_value;
    *sum += new_value;

    // Avanzar el índice circularmente
    *index = (old_index + 1) % buff_size;

    // Calcular la nueva media
    *avg = *sum / buff_size;
}

void get_filtered(float new_sample, float *value_filtered,float *samples, int size_samples, float *accumulator,int *buff_index,bool *is_not_full)
{
	if(*is_not_full)
	{
		samples[*buff_index] = new_sample;
		*accumulator+=samples[*buff_index];

		*value_filtered = new_sample;

		(*buff_index)++;

		if(*buff_index == size_samples)
		{
			*buff_index = 0;

			*is_not_full = false;
		}
	}
	else{
		moving_average_filter(samples, size_samples, accumulator, buff_index, new_sample, value_filtered);
	}
}

/**
 * @brief Tarea que se encarga de controlar el led
 * 
 * @param params 
 */
void control_task(void *params){
    float lux_raw_adc = 0, lux = 0, lux_raw_adc_acumul = 0;
    int index_raw_adc_buff=0;
    bool is_not_full = 0;

    float lux_raw_adc_buff[10];

    for(;;){    
        if(xQueueReceive(q_lux_values, &lux_raw_adc, portMAX_DELAY)){

            get_filtered(lux_raw_adc, &lux, lux_raw_adc_buff, 10, &lux_raw_adc_acumul, &index_raw_adc_buff, &is_not_full);

            user.lux = lux_raw_adc;
            ui_update(&oled, &user);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Tarea que se encarga de procesar los datos
 * 
 */
void get_lux_task(void *params){
    adc_run(true);
    uint16_t raw_adc_values;
    float lux = 0;
    for(;;){
        if(xQueueReceive(q_raw_adc_values, &raw_adc_values, portMAX_DELAY)){
            lux = 3300.0 * ((float)raw_adc_values / 4095.0);
            xQueueSend(q_lux_values, &lux, portMAX_DELAY);

            // xSemaphoreTake(semphr_i2c, portMAX_DELAY);

            // xSemaphoreGive(semphr_i2c, portMAX_DELAY);

            vTaskDelay(10);
        }
    }
}

/**
 * @brief Tarea que se encarga de leer el valor del ADC
 * 
 * @param params 
 */
void enable_adc_fifo_task(void *params){
    assert(PERIOD_RATE>=1);
    for(;;){
        adc_irq_set_enabled(true);
        adc_run(true);
        vTaskDelay(pdMS_TO_TICKS(PERIOD_RATE));
    }
}

/**
 * @brief Interrupcion que inicia la conversion del adc
 * 
 */
void IRQ_ReadAdcFifo(){
    adc_irq_set_enabled(false);
    adc_run(false);
    uint16_t adc_raw_values = adc_fifo_get();
    adc_fifo_drain();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(q_raw_adc_values, &adc_raw_values, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                                          // Cambio el contexto del Scheduler para que ejecute la tarea de procesamiento
}

/**
 * @brief Configura el I2C
 * 
 */
void i2c_config(void){
    i2c_init(I2C_PORT, I2C_FREQ);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

/**
 * @brief Configuro el ADC
 * 
 */
void adc_config(){
    adc_init();                                     // Inicio el periferico
    adc_gpio_init(PIN_TEMT6000);
    adc_select_input(CHANN_TEMT6000);

    adc_fifo_setup(
        true,      // Habilito el fifo
        true,      // Cada muestra pushea al FIFO
        1,        // Genera solicitud DMA o IRQ al tener al menos 1 muestra
        false,     // Desactivo el bit de error
        false      // El registro va a contener un dato de mas de un byte, sera de 16bit aunque el adc es de 12bit
    );    

    adc_set_clkdiv(ADC_CLK_BASE/(float)SAMPLE_RATE);        // Seteo el sample rate del adc
    
    irq_set_exclusive_handler(ADC_IRQ_FIFO, IRQ_ReadAdcFifo);
    
    adc_irq_set_enabled(true);
    irq_set_enabled(ADC_IRQ_FIFO, true);
}

int main() {
    stdio_init_all();

    q_raw_adc_values = xQueueCreate(BUFF_SIZE, sizeof(uint16_t));
    q_lux_values = xQueueCreate(BUFF_SIZE, sizeof(float));

    semphr_i2c = xSemaphoreCreateMutex();

    adc_config();
    i2c_config();
    bh1750_init();

    oled.external_vcc = false;
    ui_init(&oled, I2C_PORT, &user);

    uint8_t c = 0;

    xTaskCreate(
        control_task,
        "control_task",
        configMINIMAL_STACK_SIZE*2,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL
    );

    xTaskCreate(
        get_lux_task,
        "get_lux_task",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    xTaskCreate(
        enable_adc_fifo_task,
        "enable_adc_fifo_task",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL
    );

    vTaskStartScheduler();
    
    while (1) {
        // printf("Lux TEMT6000:%f\n", temt6000_get_lux());
        // uint16_t lux = bh1750_read_lux();
        // printf("Luz BH1750: %u lux\n", lux);
        // user.lux=lux;
        // user.sp = (user.sp==0) ? MAX_SP:(user.sp - 1);

        // ui_update(&oled, &user);
        
        // user.select = (c==20) ? (user.select + 1)%(not_show) : user.select;
        // c = (c+1)%21;

        // sleep_ms(150);
    }
}

