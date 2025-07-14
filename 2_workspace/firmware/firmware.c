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

#define MAX_SET_POINT 1000
#define PWM_WRAP      4096

#define SELECT_BTN 15

#define BUFF_SIZE 100
#define SAMPLE_RATE 800
#define PERIOD_RATE 1000/SAMPLE_RATE
#define ADC_CLK_BASE 48000000.f
#define CONVERSION_FACTOR 3.3f / (1 << 12)

#define BH1750_SAMPLE_TIME_MS 120

#define MAX_COUNT 100

QueueHandle_t q_raw_adc_values, q_values_to_show;
QueueHandle_t q_send_uart;
SemaphoreHandle_t semphr_i2c;
ssd1306_t oled;

user_t user = {
    .sp = MAX_SP/2,
    .mode = true,
    .lux = MAX_LUX/2,
    .select = set_sp
};

/**
 * @brief Tarea que controla la luminosidad del Led y envia los datos a graficar
 * 
 * @param params 
 */
void central_task(void *params){
    // Control
    int pwm;
    float lux = 0.0f;
    float lux_temt6000 = 0.0f;
    float lux_bh1750 = 0.0f;
    float prev_lux = 0.0f;
    float error;
    float kp = 0.1;
    float h = MAX_SET_POINT / PWM_WRAP;

    // Filtrado
    const float alpha = 0.414;
    float set_point;
    uint16_t raw_adc_values;
    uint8_t c = 0;
    uint8_t samples = 0;

    float coef_fusion = 0.3f;

    adc_run(true);

    for(;;){
        if(xQueueReceive(q_raw_adc_values, &raw_adc_values, portMAX_DELAY)){

            if(samples == 120){
                xSemaphoreTake(semphr_i2c, portMAX_DELAY);
                    lux_bh1750 = bh1750_read_lux();
                xSemaphoreGive(semphr_i2c);
                samples = 0;
            }

            lux_temt6000 = temt6000_get_lux(raw_adc_values);

            lux = lux_temt6000 * coef_fusion + lux_bh1750 * (1.0f - coef_fusion);

            // lux = alpha * lux + (1.0f - alpha) * prev_lux; // Mejorar usando filtro fir con CMSIS DSP
            
            prev_lux = lux;

            error = set_point - lux;
    
            pwm += kp * error * h;

            c++;
            samples++;

            xQueueSend(q_send_uart, &lux, portMAX_DELAY);

            if(c==MAX_COUNT){ // Cada una cierta cantidad de muestras enviare la muestra a la tarea que se encarga de la ui
                c = 0;
                xQueueSend(q_values_to_show, &lux, portMAX_DELAY);
            }

            adc_irq_set_enabled(true);
            adc_run(true);
            vTaskDelay(pdMS_TO_TICKS(1)); // Se espera una distancia entre muestras de 1.3ms
        }
    }
}

void send_uart_task(void *params){
    float serial_value;
    for(;;){
        if(xQueueReceive(q_send_uart,&serial_value, portMAX_DELAY)){
            printf("%.2f\n", serial_value);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Muestra y maneja la interfaz de usuario
 * 
 * @param params 
 */
void ui_task(void *params){
    float lux = 0;
    for(;;){
        if(xQueueReceive(q_values_to_show, &lux, portMAX_DELAY)){
            user.lux = lux;
            
            xSemaphoreTake(semphr_i2c, portMAX_DELAY);
                ui_update(&oled, &user);
            xSemaphoreGive(semphr_i2c);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Interrupcion que maneja el boton de seleccion
 * 
 */
void IRQ_BTN(void){

    if(gpio_get_irq_event_mask(SELECT_BTN) & GPIO_IRQ_EDGE_FALL){
        gpio_acknowledge_irq(SELECT_BTN, GPIO_IRQ_EDGE_FALL);
        user.select = (user.select + 1) % not_show; // Cambio el modo de seleccion
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
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                                         
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
    q_values_to_show = xQueueCreate(BUFF_SIZE, sizeof(float));
    q_send_uart = xQueueCreate(BUFF_SIZE, sizeof(float));

    semphr_i2c = xSemaphoreCreateMutex();

    adc_config();
    i2c_config();
    bh1750_init();

    oled.external_vcc = false;
    ui_init(&oled, I2C_PORT, &user);

    uint8_t c = 0;

    xTaskCreate(
        central_task,
        "central_task",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL
    );

    xTaskCreate(
        send_uart_task,
        "send_uart_task",
        configMINIMAL_STACK_SIZE*2,
        NULL,
        tskIDLE_PRIORITY * 1,
        NULL
    );

    xTaskCreate(
        ui_task,
        "ui_task",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,
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

