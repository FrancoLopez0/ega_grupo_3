#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "bmp280.h"
#include "lcd.h"
#include "semphr.h"
#include "queue.h"

#define SDA_PIN          4U
#define SCL_PIN          5U
#define LCD_DIR          0x27 
#define I2C_FREQ         100000

#define BUFF_SIZE_LCD    10
#define BUFF_SIZE_I2C    10
#define SAMPLE_TIME_MS   1000

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

SemaphoreHandle_t g_sI2c;

QueueHandle_t g_qLcd;

void i2c_config(void){
    i2c_init(i2c0, I2C_FREQ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

void lcd_guardian_task(void *args){
    
    lcd_w_t w;

    lcd_set_cursor(0,0);
    lcd_string("Temp:");

    lcd_set_cursor(1,0);
    lcd_string("Pres:");
    
    while(1){
        if(xQueueReceive(g_qLcd, &w, portMAX_DELAY)){
            xSemaphoreTake(g_sI2c, portMAX_DELAY);
                lcd_set_cursor(w.line, w.pos);
                lcd_string(w.str);
            xSemaphoreGive(g_sI2c);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
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

    uint32_t c=0;

    while (1)
    {
        xSemaphoreTake(g_sI2c, portMAX_DELAY);
            bmp280_read_raw(&raw_temp,&raw_preasure);
        xSemaphoreGive(g_sI2c);

        measures.pres = bmp280_convert_pressure(raw_preasure, raw_temp, &calib_param) / 1000;
        measures.temp = bmp280_convert_temp(raw_temp, &calib_param);

        lcd_write.line = 0;
        sprintf(lcd_write.str, "%.2f %cC", measures.temp, (char)223);
        xQueueSend(g_qLcd, &lcd_write, portMAX_DELAY);

        lcd_write.line = 1;
        sprintf(lcd_write.str, "%.2f Kpa", measures.pres);
        xQueueSend(g_qLcd, &lcd_write, portMAX_DELAY);
        printf(lcd_write.str);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_TIME_MS));
    }    
}

int main()
{
    stdio_init_all();

    g_qLcd = xQueueCreate(BUFF_SIZE_LCD, sizeof(lcd_w_t));

    g_sI2c = xSemaphoreCreateMutex();

    i2c_config();
    lcd_init(i2c0, LCD_DIR);
    bmp280_init(i2c0);

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

    vTaskStartScheduler();

    while (true) {

    }
}
