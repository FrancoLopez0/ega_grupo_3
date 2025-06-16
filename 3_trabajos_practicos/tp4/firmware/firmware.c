#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "bmp280.h"
#include "lcd.h"
#include "semphr.h"
#include "queue.h"

#define SDA_PIN 4U
#define SCL_PIN 5U
#define LCD_DIR 0x27 
#define I2C_FREQ 100000

#define BUFF_SIZE_LCD 10
#define BUFF_SIZE_I2C 10
#define SAMPLE_TIME   1000

typedef struct 
{
    char str[16];
    uint8_t line;
    uint8_t pos;
}lcd_w_t;

SemaphoreHandle_t g_sI2c;

QueueHandle_t g_qLcd, g_qI2C;

void i2c_config(void){
    i2c_init(i2c0, I2C_FREQ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    bi_decl(bi_2pins_with_func(SDA_PIN,SCL_PIN, GPIO_FUNC_I2C));
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
                printf("%s, (%d,%d)\n", w.str, w.line, w.pos);
                lcd_set_cursor(w.line, w.pos);
                lcd_string(w.str);
            xSemaphoreGive(g_sI2c);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void bmp_sense_task(void *args){
    lcd_w_t bmp_temp = {
        .str="--",
        .line = 0,
        .pos = 6
    };
    lcd_w_t bmp_pres = {
        .str="--",
        .line = 1,
        .pos = 6
    };
    
    uint32_t c = 0;

    while (1)
    {
        xSemaphoreTake(g_sI2c, portMAX_DELAY);
            printf("Sensar BMP\n");
            sprintf(bmp_temp.str, "%d %cC", c, (char)223);
            sprintf(bmp_pres.str, "%d Kpa", c);
        xSemaphoreGive(g_sI2c);

        xQueueSend(g_qLcd, &bmp_temp, portMAX_DELAY);
        xQueueSend(g_qLcd, &bmp_pres, portMAX_DELAY);
        c++;
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_TIME));
    }    
}

int main()
{
    stdio_init_all();

    g_qLcd = xQueueCreate(BUFF_SIZE_LCD, sizeof(lcd_w_t));
    // xQueueCreate(BUFF_SIZE_I2C, sizeof(void*));

    g_sI2c = xSemaphoreCreateMutex();

    i2c_config();
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

    vTaskStartScheduler();

    while (true) {

    }
}
