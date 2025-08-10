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
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"

#include "modules/temt6000/temt6000.h"
#include "modules/bh1750/bh1750.h"
#include "modules/ui/ui.h"
#include "modules/ds1307/ds1307.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "config.h"

#define I2C_FREQ        400*1000
#define CHANN_TEMT6000  2

#define SELECT_BTN 20

#define BUFF_SIZE   100
#define SAMPLE_RATE 800
#define PERIOD_RATE 1000/SAMPLE_RATE
#define ADC_CLK_BASE 48000000.f
#define CONVERSION_FACTOR 3.3f / (1 << 12)

#define PWM_CLK 100000

#define BH1750_SAMPLE_TIME_MS 120

#define MAX_COUNT 100

typedef struct{
    bool clk; // Estado del clk del encoder
    bool dt; // Estado del dt del encoder
    bool sw; // Estado del boton del encoder
    bool prev_clk; // Estado anterior del clk del encoder
    int user_increment; // Incremento del usuario
    int user_select; // Seleccion del usuario
    TickType_t last_valid_edge; // Ultimo flanco valido
    TickType_t debounce_time; // Tiempo de debounce
    TickType_t last_edge_time; // Ultimo tiempo de flanco
    TickType_t current_edge_time; // Tiempo actual de flanco
    TickType_t rotation_period; // Periodo de rotacion
}encoder_t;

typedef struct{
    uint16_t lux; // Valor en lux
}bh1750_t;

typedef struct{
    ssd1306_t *p_oled; // Puntero a la pantalla OLED
    user_t *p_user; // Puntero a la estructura de usuario
}ui_t;

enum i2c_devices_t {
    bh1750_device,
    ssd1306_device,
    rtc_device
};

typedef struct{
    enum i2c_devices_t device;
    QueueHandle_t return_queue;
    void (*callback)(void *params);
    void * context;
}i2c_guardian_t;

QueueHandle_t q_raw_adc_values, q_values_to_show, q_control, q_lux, q_rtc, q_rtc_config;

ds1307_t g_rtc;

#ifdef PRINT_VALUES_MODE
QueueHandle_t q_send_uart;
#endif

QueueHandle_t q_i2c_guardian;
QueueHandle_t q_i2c_bh1750;
SemaphoreHandle_t semphr_user, set_user_event, encoder_event, change_event;
ssd1306_t oled;

user_t user = {
    .sp = 600,
    .mode = true,
    .lux = MAX_LUX/2,
    .select = set_sp,
    .change_value_mode = false,
    .rise_time_ms = 1000,
    .sp_f = 2000,
    .menu = config_menu,
    .min = 100,
    .max = 2500,
    .day = 5,
    .month = 8,
    .year = 25,
    .hour = 2,
    .minute = 55,
    .sencond = 10
};

void i2c_guard_bh1750(bh1750_t *bh1750){
    bh1750->lux = bh1750_read_lux();
}

void i2c_guard_ssd1306(ui_t *ui){
    ui_update(&oled, ui->p_user);
}

void i2c_guardian_task(void *params){
    i2c_guardian_t guardian;
    for(;;){
        if(xQueueReceive(q_i2c_guardian, &guardian, portMAX_DELAY)){
                if(guardian.callback != NULL){
                    if(guardian.context!=NULL){
                        guardian.callback(guardian.context);
                    }
                    else{
                        guardian.callback(NULL);
                    }
                }
                if(guardian.return_queue != NULL){
                    xQueueSend(guardian.return_queue, guardian.context, 1);
                }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Evita que la tarea consuma todo el tiempo de CPU
    }
}

void bh1750_task(void *params){
    bh1750_t bh1750_context = {
        .lux = 0
    };
    i2c_guardian_t bh1750_guardian = {
        .device = bh1750_device,
        .return_queue = q_i2c_bh1750,
        .callback = (void *)i2c_guard_bh1750,
        .context = &bh1750_context
    };

    for(;;){
        xQueueSend(q_i2c_guardian, &bh1750_guardian, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(BH1750_SAMPLE_TIME_MS));
    }
}

float kalman_update(float z, float R, float Q, float *x_est, float *P) {
    // z: nueva medición
    // R: varianza del ruido de medición
    // Q: varianza del ruido de proceso
    // *x_est: puntero al valor estimado actual
    // *P: puntero a la covarianza del error

    // Predicción
    float x_pred = *x_est;
    float P_pred = *P + Q;

    // Ganancia de Kalman
    float K = P_pred / (P_pred + R);

    // Corrección
    *x_est = x_pred + K * (z - x_pred);
    *P = (1.0f - K) * P_pred;

    return *x_est;  // Devuelve el nuevo valor estimado
}

/**
 * @brief Tarea que se encarga de enviar las solicitudes a la tarea guardiana I2C
 * 
 * @param pvParameters 
 */
void rtc_task(void *pvParameters){
    i2c_guardian_t guardian;

    guardian = (i2c_guardian_t){
        .device = rtc_device,
        .return_queue = q_rtc,
        .callback = (void *)ds1307_get_time,
        .context = &g_rtc
    };

    while(1){

        if(xQueueReceive(q_rtc_config, &g_rtc, 0)){
            guardian.callback = (void *) ds1307_set_time;
            guardian.return_queue = NULL;
            xQueueSend(q_i2c_guardian, &guardian, portMAX_DELAY);
            guardian.callback = (void *) ds1307_get_time;
            guardian.return_queue = q_rtc;
        }

        xQueueSend(q_i2c_guardian, &guardian, portMAX_DELAY);
        vTaskDelay(1000);
    }
}

/**
 * @brief Tarea encargada de realizar el control de luminosidad
 * 
 * @param params 
 */
void control_task(void *params){
    
    float value_to_control = 0;
    float kd = 0.0f;
    float kp = 0.15;
    float ki = 0.5;
    float error, prev_error, diferential_error, integral_error;
    float prev_time;
    float dt;
    uint16_t pwm;
    float h = PWM_WRAP / MAX_SET_POINT;
    float pid;
    float set_point = 0;
    
    for(;;){

        xQueueReceive(q_control, &set_point, 0);
        xQueueReceive(q_lux, &value_to_control, portMAX_DELAY);

        error = set_point - value_to_control;

        dt = (float)pdTICKS_TO_MS(xTaskGetTickCount())/1000.0 - prev_time;

        diferential_error = (error - prev_error)/dt;

        integral_error += error * dt;

        pid = error * kp + diferential_error * kd + integral_error * ki;

        pwm = (uint16_t)(pid * h);

        // Saturación
        if(pwm>=PWM_WRAP){
            pwm = PWM_WRAP;
        }
        if(pwm<0){
            pwm = 0;
        }

        // printf("PID:%.2f PWM:%d error: %.2f lux:%.2f\n", pid, pwm, error, value_to_control);

        pwm_set_gpio_level(PIN_PWM,PWM_WRAP - pwm);

        prev_time = (float)pdTICKS_TO_MS(xTaskGetTickCount())/1000.0;

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

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
    float delta_error;
    float kp = 0.25 ;
    float kd = 0.0f;
    float h =  PWM_WRAP / MAX_SET_POINT;

    bh1750_t bh1750;

    // Filtrado
    const float alpha = 0.414;
    float set_point = user.sp;
    float coef_fusion = 1;
    uint16_t raw_adc_values;
    uint8_t c = 0;
    uint8_t samples = 0;

    float x_est = 0.0f;  // Estimación inicial (lux)
    float P = 1.0f;      // Incertidumbre inicial
    float Q = 0.01f;     // Ruido de proceso (ajustable)
    float R = 0.5f;      // Ruido de medición (depende del TMT6000 y el ADC)

    adc_run(true);

    for(;;){
        if(xQueueReceive(q_raw_adc_values, &raw_adc_values, portMAX_DELAY)){

            
            lux_temt6000 = temt6000_get_lux(raw_adc_values);
            
            if(xQueueReceive(q_i2c_bh1750, &bh1750, 0) == pdPASS)
            {
                // printf("bh1750: %d tmt6000: %.2f\n", bh1750.lux, lux_temt6000);
                // lux = kalman_update(bh1750.lux, 0.1, Q, &x_est, &P);
            }

            lux = kalman_update(lux_temt6000, R, Q, &x_est, &P); // Filtro de Kalman

            prev_lux = lux;

            xQueueSend(q_lux, &lux, portMAX_DELAY);

            // error = set_point - lux;

            // control.input = lux;
            // control.set_point = 3000.0; //MIN 80 MAX 3000

            // control.set_point += 1.0;
            // if(control.set_point > 3000) control.set_point = 80.0;

            // xSemaphoreTake(semphr_user, portMAX_DELAY);
            //     user.sp = control.set_point;
            //     // control.set_point = user.sp;
            // xSemaphoreGive(semphr_user);

            // printf("LUX DESDE CENTRAL: %.2f \n", control.input);

            // xQueueSend(q_control, &control, portMAX_DELAY);

            // pwm += (kp * error + kd * delta_error / 1000) * h; // Calculo el pwm a aplicar

            // pwm_set_gpio_level(PIN_PWM,PWM_WRAP - (uint16_t)pwm);

            // printf("pwm: %d  error: %f \n",pwm, error);

            c++;
            // xQueueSend(q_send_uart, &lux, portMAX_DELAY); // Envio el valor de lux a la tarea que envia por UART

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

#ifdef PRINT_VALUES_MODE
/**
 * @brief Envia los datos por UART
 * 
 * @param params 
 */
void send_uart_task(void *params){
    float serial_value;
    for(;;){
        if(xQueueReceive(q_send_uart,&serial_value, portMAX_DELAY)){
            printf("%.2f\n", serial_value);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif

int encoder_increment(encoder_t *encoder) {
    if(!encoder->clk && encoder->prev_clk) {
        TickType_t now = xTaskGetTickCount();
        
        // Filtro de debounce: solo aceptamos el flanco si ha pasado el tiempo mínimo
        if((now - encoder->last_valid_edge) > encoder->debounce_time) {
            encoder->last_valid_edge = now;
            return 1; // Retorna 1 si se detecta un evento de rotación
        }
    }
    return 0; // Retorna 0 si no hay evento
}

int encoder_count(encoder_t *encoder) {
    int increment = 0; // Valor de incremento basado en la velocidad de rotación
    if(!encoder->clk && encoder->prev_clk) {
        TickType_t now = xTaskGetTickCount();
        
        // Filtro de debounce: solo aceptamos el flanco si ha pasado el tiempo mínimo
        if((now - encoder->last_valid_edge) > encoder->debounce_time) {
            encoder->last_valid_edge = now;
            
            // Cálculo de velocidad (solo si es un flanco válido)
            encoder->current_edge_time = now;
            TickType_t rotation_period = encoder->current_edge_time - encoder->last_edge_time;
            encoder->last_edge_time = encoder->current_edge_time;
            
            // Cálculo del incremento basado en velocidad
            if(rotation_period < pdMS_TO_TICKS(200)) increment = 50;
            else if(rotation_period < pdMS_TO_TICKS(300)) increment = 20;
            else if(rotation_period < pdMS_TO_TICKS(400)) increment = 10;
            else increment = 1;
            
            return increment * (encoder->dt ? 1 : -1); // Retorna el incremento o decremento según el estado del dt
        }
    }
    return 0;
}

/**
 * @brief Muestra y maneja la interfaz de usuario
 * 
 * @param params 
 */
void user_task(void *params) {

    //  Vista de usario
    user_t user_view = user;
    user.sp = 0;
    int set_point = 0;
    int delta_time = 0, last_time = 0;
    
    //  Encoder
    int encoder_increment = 0;
    uint8_t set_user_event_count;

    // Interfaz OLED
    ui_t ui = {
        .p_oled = &oled,
        .p_user = &user
    };

    i2c_guardian_t ui_guardian = {
        .device = ssd1306_device,
        .return_queue = NULL,
        .callback = (void *)i2c_guard_ssd1306,
        .context = &ui
    };

    float lux = 0;

    last_time = pdTICKS_TO_MS(xTaskGetTickCount());

    int set_point_0 = user.sp, set_point_final = user.sp;
    bool motion = true;

    user_view.menu = config_menu;

    ds1307_t rtc;

    for(;;){
        set_user_event_count = uxSemaphoreGetCount(set_user_event); // Tomo la cantidad de veces quese presiono el boton del encoder

        if(xQueueReceive(encoder_event, &encoder_increment, 0)); // En caso de rotar el encoder recibo el valor en ticks de la rotacion

        if(xQueueReceive(q_rtc, &rtc, 0) && motion){
            user_view.hour = rtc.time.hours;
            user_view.minute = rtc.time.minutes;
            user_view.sencond = rtc.time.seconds;

            user_view.year = rtc.time.year;
            user_view.month = rtc.time.month;
            user_view.day = rtc.time.day;

            user = user_view;
            // printf("%02d:%02d:%02d\n", user_view.hour, user_view.minute, user_view.sencond);
        }

        user_view.menu = config_menu;//uxSemaphoreGetCount(change_event) % 3; // Tomo la cantidad de veces que se presiono el boton de cambio de menu
        user.menu = user_view.menu;

        if(set_user_event_count % 2 == 1){
            motion = false;
            ui.p_user = &user_view;
            user_view.change_value_mode = true; // Indico que se esta cambiando el valor del usuario
            
            if(user_view.menu == params_menu){
                switch (user_view.select)
                {
                case set_sp:
                    user_view.sp += encoder_increment; // Actualizo el set point del usuario
                    if(user_view.sp < 0){
                        user_view.sp = 0; // Evito que el set point sea negativo
                    }
                    if(user_view.sp > MAX_SET_POINT){
                        user_view.sp = MAX_SET_POINT; // Evito que el set point sea mayor al maximo
                    }
                    break;
                case set_sp_f:
                    user_view.sp_f += encoder_increment; // Actualizo el set point del usuario
                    if(user_view.sp_f < 0){
                        user_view.sp_f = 0; // Evito que el set point sea negativo
                    }
                    if(user_view.sp_f > MAX_SET_POINT){
                        user_view.sp_f = MAX_SET_POINT; // Evito que el set point sea mayor al maximo
                    }
                    break;
                case set_time:
                    user_view.rise_time_ms += encoder_increment; // Actualizo el set point del usuario
                    if(user_view.rise_time_ms < 0){
                        user_view.rise_time_ms = 0; // Evito que el set point sea negativo
                    }
                    if(user_view.rise_time_ms > MAX_RISE_TIME){
                        user_view.rise_time_ms = MAX_RISE_TIME; // Evito que el set point sea mayor al maximo
                    }
                default:
                    break;
                }
            }

            if(user_view.menu == config_menu){
                ui.p_user = &user_view;

                switch (user_view.select)
                {
                    case hour_config:
                        if(user_view.hour + encoder_increment >0)
                            user_view.hour += encoder_increment;
                        user_view.hour %= 60;
                        rtc.time.hours = user_view.hour;
                    break;
                    case second_config:
                        if(user_view.sencond + encoder_increment >0)
                            user_view.sencond += encoder_increment;
                        user_view.sencond %= 60;
                        rtc.time.seconds = user_view.sencond;
                    break;
                    case minute_config:
                        if(user_view.minute + encoder_increment >0)
                            user_view.minute += encoder_increment;
                        user_view.minute %= 60;
                        rtc.time.minutes = user_view.minute;
                    break;
                    case day_config:
                        if(user_view.day + encoder_increment > 0)
                            user_view.day += encoder_increment;
                        user_view.day %= 32;
                        rtc.time.day = user_view.day;
                    break;
                    case month_config:
                        if(user_view.month + encoder_increment > 0)
                            user_view.month += encoder_increment;
                        user_view.month %= 13;
                        rtc.time.month = user_view.month;
                    break;
                    case year_config:
                        if(user_view.year + encoder_increment > 0)
                            user_view.year += encoder_increment;
                        rtc.time.year = user_view.year;
                    break;
                    case min_config:
                        if(user_view.min + encoder_increment > 0)
                            user_view.min += encoder_increment;
                    break;
                    case max_config:
                        if(user_view.max + encoder_increment > 0)
                            user_view.max += encoder_increment;
                    break;
                default:
                    break;
                }
            }
            encoder_increment = 0;
        }else{
            user_view.change_value_mode = false;

            if(user_view.menu == params_menu){
                user_view.select = (user_view.select + encoder_increment) % not_show; // Actualizo la seleccion del usuario
            }
            if(user_view.menu == config_menu){
                user_view.select = (user_view.select + encoder_increment) % 9;
            }
            user.select = user_view.select;
            encoder_increment = 0;
        }
        
        if(uxSemaphoreGetCount(set_user_event)==100){
            xQueueReset(set_user_event);
        }

        delta_time = pdTICKS_TO_MS(xTaskGetTickCount()) - last_time;
        
        if(delta_time >= user_view.rise_time_ms){
            last_time = pdTICKS_TO_MS(xTaskGetTickCount());
        }
        
        if(user.rise_time_ms >0){
            set_point = (int)(((float)(set_point_final - set_point_0) / (float)user.rise_time_ms) * (float)delta_time) + set_point_0;
            user.sp = set_point;
        }
        else{
            set_point = user.sp;
        }
        
        // printf("set_point: %d rise_time: %d set_point_final: %d\n", set_point, user.rise_time_ms, user.sp_f);

        if(set_user_event_count % 2 == 0 && set_user_event_count != 0){ // Si es multiplo de 2
            // user.sp = user_view.sp;
            // user.rise_time_ms = user_view.rise_time_ms;
            // user.sp_f = user_view.sp_f;
            user = user_view;
            xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);


            set_point_0 = user.sp;
            set_point_final = user.sp_f;

            ui.p_user = &user;

            // printf("Seteado en: \n sp: %d\n sp_f: %d\n rise_time_ms: %d\n", user.sp, user.sp_f, user.rise_time_ms);

            motion = true;
            xQueueReset(set_user_event);
        }
       
        xQueueSend(q_control, &set_point, portMAX_DELAY);

        if(xQueueReceive(q_values_to_show, &lux, 0) == pdPASS){
                
                ui.p_user->lux = lux;

                if(lux < user.min){
                    // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
                    gpio_put(PIN_LED_GREEN, 0);
                    gpio_put(PIN_LED_RED, 1);
                }
                else if(lux > user.max){
                    gpio_put(PIN_LED_GREEN, 1);
                    gpio_put(PIN_LED_RED, 0);
                    // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
                }else{
                    gpio_put(PIN_LED_GREEN, 1);
                    gpio_put(PIN_LED_RED, 1);
                }
            xQueueSend(q_i2c_guardian, &ui_guardian, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Tarea encargada de la revision del estado de los botones
 * 
 * @param params 
 */
void btns_task(void *params) {
    // Encoder
    enum select_enum option_menu = set_sp; // Cantidad de modos que no se muestran en la pantalla
    bool prev_clk = false; // Variable para detectar el cambio de estado del encoder
    bool clk = false; // Variable para almacenar el estado actual del clk del encoder

    // Variables para debounce
    TickType_t last_valid_edge = 0;
    const TickType_t debounce_time = pdMS_TO_TICKS(0); // Tiempo de debounce (10ms)

    // Variables para calcular velocidad de rotación
    TickType_t last_edge_time = xTaskGetTickCount();
    TickType_t current_edge_time;
    TickType_t rotation_period;

    int32_t increment = 10; // Valor base de incremento
    int user_increment = 0; // Variable para almacenar el set point del usuario
    int user_select = set_sp;

    encoder_t encoder = {
        .clk = false,
        .dt = false,
        .sw = false,
        .prev_clk = false,
        .user_increment = user_increment,
        .user_select = user_select,
        .last_valid_edge = last_valid_edge,
        .debounce_time = debounce_time,
        .last_edge_time = last_edge_time,
        .current_edge_time = 0,
        .rotation_period = 0
    };
    int mode = 0; // Variable para almacenar el modo actual
    for(;;) {
        encoder.clk = gpio_get(CLK); // Leo el estado del clk del encoder
        encoder.dt = gpio_get(DT); // Leo el estado del dt del encoder
        encoder.sw = gpio_get(SW); // Leo el estado del boton del encoder

        user_increment = encoder_count(&encoder); // Llamo a la funcion que cuenta el encoder
        if(user_increment!=0) xQueueSend(encoder_event, &user_increment, portMAX_DELAY);
        encoder.prev_clk = encoder.clk; // Guardo el estado anterior del clk

        if(!gpio_get(PIN_BTN)){
            xSemaphoreGive(change_event);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if(!encoder.sw) { // Si el boton esta presionado
            xSemaphoreGive(set_user_event);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
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
 * @brief Configura el boton de seleccion
 * 
 */
void gpio_config(void){\
    
    // gpio_init(SELECT_BTN);
    // gpio_set_dir(SELECT_BTN, GPIO_IN);
    // gpio_pull_up(SELECT_BTN);

    gpio_init(CLK);
    gpio_set_dir(CLK, GPIO_IN);

    gpio_init(DT);
    gpio_set_dir(DT, GPIO_IN);

    gpio_init(SW);
    gpio_set_dir(SW, GPIO_IN);
    gpio_pull_up(SW);

    gpio_init(PIN_BTN);
    gpio_set_dir(PIN_BTN, GPIO_IN);
    gpio_pull_up(PIN_BTN);

    gpio_init(PIN_LED_GREEN);
    gpio_set_dir(PIN_LED_GREEN, GPIO_OUT);
    gpio_pull_up(PIN_LED_GREEN);

    gpio_init(PIN_LED_RED);
    gpio_set_dir(PIN_LED_RED, GPIO_OUT);
    gpio_pull_up(PIN_LED_RED);
    
    // gpio_set_irq_enabled_with_callback(
    //     SW,
    //     GPIO_IRQ_EDGE_FALL,
    //     true,
    //     &IRQ_BTN
    // );
}

/**
 * @brief Configura el controlador del pwm
 * 
 * @param pin 
 * @param clk 
 * @return uint 
 */
uint config_pwm(uint16_t pin, float clk){

    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();

    // pwm_config_set_wrap(&config, 4096U);
    
    pwm_config_set_clkdiv(&config, 1.25f);
    
    pwm_init(slice_num, &config, true);
    
    pwm_set_wrap(slice_num, 4096U);
    
    return slice_num;
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

    q_lux = xQueueCreate(BUFF_SIZE, sizeof(float));
    q_control = xQueueCreate(BUFF_SIZE, sizeof(float));

    q_rtc = xQueueCreate(5, sizeof(ds1307_t));
    q_rtc_config = xQueueCreate(5, sizeof(ds1307_t));

    #ifdef PRINT_VALUES_MODE
    q_send_uart = xQueueCreate(BUFF_SIZE, sizeof(float));
    #endif
    
    q_i2c_guardian = xQueueCreate(BUFF_SIZE, sizeof(i2c_guardian_t));
    q_i2c_bh1750 = xQueueCreate(BUFF_SIZE, sizeof(bh1750_t));

    semphr_user = xSemaphoreCreateMutex();
    set_user_event = xSemaphoreCreateCounting(2,0);
    change_event = xSemaphoreCreateCounting(100,0);
    encoder_event = xQueueCreate(2, sizeof(int));

    adc_config();
    i2c_config();
    bh1750_init();
    gpio_config();
    config_pwm(PIN_PWM, PWM_CLK);

    g_rtc.i2c = i2c0;
    g_rtc.addr = DS1307_ADDRESS;
    ds1307_init(&g_rtc);

    g_rtc.time.hours = 5;
    g_rtc.time.day = 2;
    g_rtc.time.month = 5;
    g_rtc.time.year = 2025;
    ds1307_set_time(&g_rtc);

    pwm_set_gpio_level(PIN_PWM, 2000);

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
    }    

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    oled.external_vcc = false;
    ui_init(&oled, I2C_PORT, &user);

    uint8_t c = 0;

    xTaskCreate(
        central_task,
        "central_task",
        configMINIMAL_STACK_SIZE*3,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL
    );

    xTaskCreate(
        bh1750_task,
        "bh1750_task",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    // xTaskCreate(
    //     bh1750_reader_task,
    //     "bh1750_reader_task",
    //     configMINIMAL_STACK_SIZE,
    //     NULL,
    //     tskIDLE_PRIORITY + 1,
    //     NULL
    // );

    #ifdef PRINT_VALUES_MODE
    xTaskCreate(
        send_uart_task,
        "send_uart_task",
        configMINIMAL_STACK_SIZE*2,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
    #endif

    xTaskCreate(
        i2c_guardian_task,
        "i2c_guardian_task",
        configMINIMAL_STACK_SIZE*4,
        NULL,
        tskIDLE_PRIORITY + 3,
        NULL
    );

    xTaskCreate(rtc_task, "RTC Task", configMINIMAL_STACK_SIZE*4, NULL, tskIDLE_PRIORITY+2, NULL);

    // xTaskCreate(
    //     ui_task,
    //     "ui_task",
    //     configMINIMAL_STACK_SIZE*2,
    //     NULL,
    //     tskIDLE_PRIORITY + 1,
    //     NULL
    // );

    xTaskCreate(
        btns_task,
        "btns_task",
        configMINIMAL_STACK_SIZE*2,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    xTaskCreate(
        user_task,
        "user_task",
        configMINIMAL_STACK_SIZE*4,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    xTaskCreate(
        control_task,
        "control_task",
        configMINIMAL_STACK_SIZE*3,
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

