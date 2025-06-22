#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "hardware/pwm.h"
#include "bmp280.h"
#include "lcd.h"
#include "semphr.h"
#include "queue.h"

#define SDA_PIN         4U
#define SCL_PIN         5U
#define LCD_DIR         0x27 
#define I2C_FREQ        100000

#define BUFF_SIZE_LCD   10
#define BUFF_SIZE_I2C   10
#define BUFF_SIZE_BMP   10
#define SAMPLE_TIME_MS  500

#define BTN_PIN         14
#define LED_PIN         13

#define PWM_WRAP        4096

#define SET_POINT       25
#define KP              PWM_WRAP/SET_POINT

typedef enum
{
    pres_temp_menu,
    error_menu,
}menu_t;

typedef struct 
{
    char str[16];
    uint8_t line;
    uint8_t pos;
}lcd_w_t;

typedef struct 
{
    float temp;
    float pres;
}bmp_t;

menu_t g_menu = pres_temp_menu;

SemaphoreHandle_t g_sI2c, g_qBmp_values;

QueueHandle_t g_qLcd;

void IRQ_btn(uint gpio, uint32_t events){

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    gpio_set_irq_enabled(BTN_PIN, GPIO_IRQ_EDGE_FALL,false);

    uint32_t last_time = get_absolute_time();
    uint32_t now_time = last_time;
    while(now_time - last_time < (100e3)){
        now_time = get_absolute_time();
    }

    g_menu = (g_menu + 1)%2;    

    lcd_w_t lcd_write={
        .line = 0,
        .pos = 0,
        .str = '\r'
    };

    xQueueSendFromISR(g_qLcd, &lcd_write, &xHigherPriorityTaskWoken);

    switch (g_menu)
    {
    case pres_temp_menu:
        lcd_write.line = 0;
        sprintf(lcd_write.str, "Temp:");
        xQueueSendFromISR(g_qLcd, &lcd_write, &xHigherPriorityTaskWoken);

        lcd_write.line = 1;
        sprintf(lcd_write.str, "Pres:");
        xQueueSendFromISR(g_qLcd, &lcd_write, &xHigherPriorityTaskWoken);
        break;
    case error_menu:
        lcd_write.line = 0;
        sprintf(lcd_write.str, "Error:");
        xQueueSendFromISR(g_qLcd, &lcd_write, &xHigherPriorityTaskWoken);

        lcd_write.line = 1;
        sprintf(lcd_write.str, "SP   :");
        xQueueSendFromISR(g_qLcd, &lcd_write, &xHigherPriorityTaskWoken);
        break;
    default:
        break;
    }

    gpio_set_irq_enabled(BTN_PIN, GPIO_IRQ_EDGE_FALL,true);
}

void gpio_config(void){
    gpio_init(BTN_PIN);
    gpio_set_input_enabled(BTN_PIN, true);
    gpio_set_irq_enabled_with_callback(BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &IRQ_btn);

    gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
    
    uint slice_num = pwm_gpio_to_slice_num(LED_PIN);

    pwm_set_wrap(slice_num, PWM_WRAP);
    pwm_set_chan_level(slice_num, 1, PWM_WRAP);
    pwm_set_enabled(slice_num, true);
}

void i2c_config(void){
    i2c_init(i2c0, I2C_FREQ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

void lcd_guardian_task(void *args){
    
    lcd_w_t w;
    
    while(1){
        if(xQueueReceive(g_qLcd, &w, portMAX_DELAY)){
            xSemaphoreTake(g_sI2c, portMAX_DELAY);
                lcd_set_cursor(w.line, w.pos);
                if(w.str[0] == '\r'){
                    lcd_clear();
                }
                lcd_string(w.str);
            xSemaphoreGive(g_sI2c);
        }
    }
}

void bmp_sense_task(void *args){
    lcd_w_t lcd_write = {
        .str="--",
        .line = 0,
        .pos = 6,
    };

    bmp_t measures = {
        .pres = 0.0f,
        .temp = 0.0f
    };

    int32_t raw_temp, raw_preasure;

    struct bmp280_calib_param calib_param;

    bmp280_get_calib_params(&calib_param);

    while (1)
    {
        xSemaphoreTake(g_sI2c, portMAX_DELAY);
            bmp280_read_raw(&raw_temp,&raw_preasure);
        xSemaphoreGive(g_sI2c);

        measures.pres = bmp280_convert_pressure(raw_preasure, raw_temp, &calib_param) / 1000;
        measures.temp = bmp280_convert_temp(raw_temp, &calib_param);

        if(g_menu == pres_temp_menu){
            lcd_write.line = 0;
            sprintf(lcd_write.str, "%.2f %cC", measures.temp, (char)223);
            xQueueSend(g_qLcd, &lcd_write, portMAX_DELAY);
    
            lcd_write.line = 1;
            sprintf(lcd_write.str, "%.2f Kpa", measures.pres);
            xQueueSend(g_qLcd, &lcd_write, portMAX_DELAY);
        }

        xQueueSend(g_qBmp_values, &measures, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_TIME_MS));
    }    
}

void temp_control_task(void* args){
    lcd_w_t lcd_write = {
        .str="--",
        .line = 0,
        .pos = 6,
    };
    bmp_t measure;
    float error;

    uint slice_num = pwm_gpio_to_slice_num(LED_PIN);

    while(1){
        if(xQueueReceive(g_qBmp_values, &measure, portMAX_DELAY)){

            error = SET_POINT - measure.temp;
            
            if(error>0){
                pwm_set_chan_level(slice_num, 1, (uint16_t)error * KP);
            }
            else{
                pwm_set_chan_level(slice_num, 1, 0);
            }

            if(g_menu == error_menu){
                lcd_write.line = 0;
                sprintf(lcd_write.str, "%.2f", error);
                xQueueSend(g_qLcd, &lcd_write, portMAX_DELAY);

                lcd_write.line = 1;
                sprintf(lcd_write.str, "%.2f", SET_POINT);
                xQueueSend(g_qLcd, &lcd_write, portMAX_DELAY);
            }
        }
    }
}

int main()
{
    stdio_init_all();

    g_qLcd = xQueueCreate(BUFF_SIZE_LCD, sizeof(lcd_w_t));
    g_qBmp_values = xQueueCreate(BUFF_SIZE_BMP, sizeof(bmp_t));

    g_sI2c = xSemaphoreCreateMutex();
    
    gpio_config();
    
    i2c_config();

    bmp280_init(i2c0);
    lcd_init(i2c0, LCD_DIR);

    xTaskCreate(
        lcd_guardian_task,
        "LCD Task",
        configMINIMAL_STACK_SIZE*2,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        bmp_sense_task,
        "BMP Task",
        configMINIMAL_STACK_SIZE*2,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        temp_control_task,
        "Control Task",
        configMINIMAL_STACK_SIZE*2,
        NULL,
        1,
        NULL
    );

    vTaskStartScheduler();

    while (true) {

    }
}
