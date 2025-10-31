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
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"

#include "modules/temt6000/temt6000.h"
#include "modules/bh1750/bh1750.h"
#include "modules/ui/ui.h"
#include "modules/ds1307/ds1307.h"
#include "modules/flash/flash.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "config.h"

#include "stdlib.h"

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

#define MAX_COUNT 10

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
    rtc_device,
};

typedef struct{
    enum i2c_devices_t device;
    QueueHandle_t return_queue;
    void (*callback)(void *params);
    void * context;
    bool overwrite;
}i2c_guardian_t;

QueueHandle_t q_raw_adc_values, q_values_to_show, q_control, q_lux, q_rtc, q_rtc_config, q_to_storage, q_user_config, q_pwm, q_kalman, q_alpha, q_control_params, q_calib_temt, q_kalman_get;

TaskHandle_t user_task_handler, get_lux_task_handler, control_task_handler;

ds1307_t g_rtc;

#ifdef PRINT_VALUES_MODE
QueueHandle_t q_send_uart;
#endif
QueueHandle_t q_i2c_guardian;
QueueHandle_t q_bh1750;
SemaphoreHandle_t set_user_event, encoder_event, change_event, read_logs_event, erase_logs_event;
ssd1306_t oled;

user_t user = {
    .sp = 600,
    .mode = true,
    .lux = MAX_LUX/2,
    .select = set_sp,
    .change_value_mode = false,
    .rise_time_ms = 0,
    .sp_f = 2000,
    .menu = params_menu,
    .min = MIN_SET_POINT,
    .max = MAX_SET_POINT,
    .day = 5,
    .month = 8,
    .year = 25,
    .hour = 2,
    .minute = 55,
    .sencond = 10
};

int set_point_0, set_point_final;

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
                    if(guardian.overwrite){
                        xQueueOverwrite(guardian.return_queue, guardian.context);
                    }
                    xQueueSend(guardian.return_queue, guardian.context, 1);
                }
        }
        #ifdef DEBUG_I2C
            printf("Keep alive: %d \n", guardian.device);
        #endif
        // vTaskDelay(pdMS_TO_TICKS(10)); // Evita que la tarea consuma todo el tiempo de CPU
    }
}

typedef struct{
    float x_est;
    float P;
    float Q;
    float R;
}kalman_t;

typedef struct{
    float kp;
    float ki;
    float kd;
}control_params_t;

/**
 * @brief Funcion encargada de filtrar los datos de iluminacion
 * 
 * @param z 
 * @param R 
 * @param Q 
 * @param x_est 
 * @param P 
 * @return float 
 */
float kalman_update(float z, kalman_t *kalman) {
    // z: nueva medición
    // R: varianza del ruido de medición
    // Q: varianza del ruido de proceso
    // *x_est: puntero al valor estimado actual
    // *P: puntero a la covarianza del error

    // Predicción
    float x_pred = kalman->x_est;
    float P_pred = kalman->P + kalman->Q;

    // Ganancia de Kalman
    float K = P_pred / (P_pred + kalman->R);
    // float K = P_pred / (P_pred + R);

    // Corrección
    kalman->x_est = x_pred + K * (z - x_pred);
    kalman->P = (1.0f - K) * P_pred;
    // *x_est = x_pred + K * (z - x_pred);
    // *P = (1.0f - K) * P_pred;

    return kalman->x_est;  // Devuelve el nuevo valor estimado
}

typedef void (*command_fn_t)(void *);

typedef struct {
    const char *name;
    command_fn_t fn;
} command_t;

typedef enum {
    PARAM_INT,
    PARAM_FLOAT,
    PARAM_STRING,
    PARAM_RTC
} param_type_t;

void set_point_get(void *param){
    vTaskSuspend(user_task_handler);
    printf("Set point: %d\n", user.sp);
    vTaskResume(user_task_handler);
}

void set_point_set(void *param){
    vTaskSuspend(user_task_handler);
    char *arg = (char *)param;
    int value = atoi(arg);
    user.sp_0 = value;
    set_point_get(NULL);
    // xQueueSend(q_control, &user.sp, portMAX_DELAY);
    vTaskResume(user_task_handler);
}

void set_point_f_get(void *param){
    vTaskSuspend(user_task_handler);
    printf("Final set point: %d\n", user.sp_f);
    vTaskResume(user_task_handler);
}

void set_point_f_set(void *param){
    vTaskSuspend(user_task_handler);
    char *arg = (char *)param;
    int value = atoi(arg);
    user.sp_f = value;
    vTaskResume(user_task_handler);
    set_point_f_get(NULL);
}

void rise_time_get(void *param){
    vTaskSuspend(user_task_handler);
    printf("Rise time: %d ms\n", user.rise_time_ms);
    vTaskResume(user_task_handler);
}

void rise_time_set(void *param){
    vTaskSuspend(user_task_handler);
    char *arg = (char *)param;
    int value = atoi(arg);
    user.rise_time_ms = value;
    vTaskResume(user_task_handler);
    rise_time_get(NULL);
}

void pwm_get(void *params){
    uint16_t pwm;
    xQueuePeek(q_pwm, &pwm, portMAX_DELAY);
    printf("PWM: %d\n", pwm);
}

void min_get(void *param){
    vTaskSuspend(user_task_handler);
    printf("Min: %d\n", user.min);
    vTaskResume(user_task_handler);
}

void min_set(void *param){
    vTaskSuspend(user_task_handler);
    char *arg = (char *)param;
    int value = atoi(arg);
    user.min = value;
    vTaskResume(user_task_handler);
    min_get(NULL);
}

void max_get(void *param){
    vTaskSuspend(user_task_handler);
    printf("Max: %d\n", user.max);
    vTaskResume(user_task_handler);
}

void max_set(void *param){
    vTaskSuspend(user_task_handler);
    char *arg = (char *)param;
    int value = atoi(arg);
    user.max = value;
    vTaskResume(user_task_handler);
    max_get(NULL);
}

void bh1750_get(void *param){
    bh1750_t bh1750;
    xQueuePeek(q_bh1750, &bh1750, portMAX_DELAY);
    printf("BH1750: %d\n", bh1750.lux);
}

void temt6000_get(void *param){
    uint16_t adc_raw;
    float calib;
    xQueuePeek(q_calib_temt, &calib, portMAX_DELAY);
    xQueuePeek(q_raw_adc_values, &adc_raw, portMAX_DELAY);
    printf("TEMT6000: %.2f\n", temt6000_get_lux(adc_raw, calib));
}

void alpha_get(void *param){
    float alpha;
    xQueuePeek(q_alpha, &alpha, portMAX_DELAY);
    printf("Alpha: %.2f\n", alpha);
}

void alpha_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    xQueueOverwrite(q_alpha, &value);
    alpha_get(NULL);
}

void pid_kp_get(void *param){
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
    printf("KP: %.2f\n", pid_params.kp);
}

void pid_kp_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
    pid_params.kp = value;
    xQueueOverwrite(q_control_params, &pid_params);
    pid_kp_get(NULL);
}

void pid_ki_get(void *param){
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
    printf("Ki: %.2f\n", pid_params.ki);
}

void pid_ki_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
    pid_params.ki = value;
    xQueueOverwrite(q_control_params, &pid_params);
    pid_ki_get(NULL);
}

void pid_kd_get(void *param){
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
    printf("Kd: %.2f\n", pid_params.kd);
}

void pid_kd_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);// Tomo todos los valores para no modificar los demas
    pid_params.kd = value;
    xQueueOverwrite(q_control_params, &pid_params);
    pid_kd_get(NULL);
}

void calib_get(void *param){
    float calib;
    xQueuePeek(q_calib_temt, &calib, portMAX_DELAY);
    printf("Calib: %.2f\n", calib);
}

void calib_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    float calib;
    calib = value;
    xQueueOverwrite(q_control_params, &calib);
    calib_get(NULL);
}

void kalman_Q_get(void *param){
    kalman_t kalman;
    xQueuePeek(q_kalman_get, &kalman, portMAX_DELAY);
    printf("Q: %.2f\n", kalman.Q);
}


void kalman_Q_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    kalman_t kalman;
    xQueuePeek(q_kalman_get, &kalman, portMAX_DELAY);
    kalman.Q = value;
    xQueueSend(q_kalman, &kalman, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
    kalman_Q_get(NULL);
}

void kalman_R_get(void *param){
    kalman_t kalman;
    xQueuePeek(q_kalman_get, &kalman, portMAX_DELAY);
    printf("R: %.2f\n", kalman.R);
}

void kalman_R_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    kalman_t kalman;
    xQueuePeek(q_kalman_get, &kalman, portMAX_DELAY);
    kalman.R = value;
    xQueueSend(q_kalman, &kalman, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
    kalman_R_get(NULL);
}

void rtc_get_time(void *param){
    ds1307_t rtc;
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    printf("%02d:%02d:%02d %02d/%02d/%02d\n", rtc.time.hours, rtc.time.minutes, rtc.time.seconds, rtc.time.date, rtc.time.month, rtc.time.year);
}

void rtc_set_hour(void *param){
    ds1307_t rtc;
    char *arg = (char *)param;
    int value = atoi(arg);
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.hours = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void rtc_set_minute(void *param){
    ds1307_t rtc;
    char *arg = (char *)param;
    int value = atoi(arg);
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.minutes = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void rtc_set_second(void *param){
    ds1307_t rtc;
    char *arg = (char *)param;
    int value = atoi(arg);
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.seconds = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void rtc_set_day(void *param){
    ds1307_t rtc;
    char *arg = (char *)param;
    int value = atoi(arg);
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.date = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void rtc_set_month(void *param){
    ds1307_t rtc;
    char *arg = (char *)param;
    int value = atoi(arg);
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.month = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void rtc_set_year(void *param){
    ds1307_t rtc;
    char *arg = (char *)param;
    int value = atoi(arg);
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.year = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

typedef struct {
    char *name;
    command_fn_t set;
    command_fn_t get;
}cmd_t;

cmd_t commands[]={
    {"set_point", set_point_set, set_point_get},
    {"set_point_f", set_point_f_set, set_point_f_get},
    {"rise_time", rise_time_set, rise_time_get},
    {"pwm", NULL, pwm_get},
    {"min", min_set, min_get},
    {"max", max_set, max_get},
    {"bh1750", NULL, bh1750_get},
    {"temt6000", NULL, temt6000_get},
    {"alpha", alpha_set, alpha_get},
    {"pid_kp", pid_kp_set, pid_kp_get},
    {"pid_ki", pid_ki_set, pid_ki_get},
    {"pid_kd", pid_kd_set, pid_kd_get},
    {"calib", calib_set, calib_get},
    {"kalman_R", kalman_R_set, kalman_R_get},
    {"kalman_Q", kalman_Q_set, kalman_Q_get},
    {"rtc", NULL, rtc_get_time},
    {"hour", rtc_set_hour, NULL},
    {"minute", rtc_set_minute, NULL},
    {"second", rtc_set_second, NULL},
    {"day", rtc_set_day, NULL},
    {"month", rtc_set_month, NULL},
    {"year", rtc_set_year, NULL}
};

#define NUM_COMMANDS sizeof(commands) / sizeof(cmd_t)

char *cmd;
char *arg1;
char *arg2;

/**
 * @brief Esta tarea se encarga de manejar la linea de comandos
 * @todo Añadir comandos
 * 
 * @param params 
 */
void cli_task(void *params){

    #define RESPONSE_MAX_LEN 64
    #define UART_ID uart0

    char cli_response[RESPONSE_MAX_LEN];
    char user_input[RESPONSE_MAX_LEN];
    char to_print[RESPONSE_MAX_LEN];
    char c;
    int index=0;
    bool found = false;

    while(1){
        if (uart_is_readable(UART_ID)) {
            c = uart_getc(UART_ID);
            
            if(c == '\n'){
                found = false;
                user_input[index] = '\0';
                index = 0;

                // Tokenizar el input
                cmd = strtok(user_input, " ");
                arg1 = strtok(NULL, " ");
                arg2 = strtok(NULL, " ");

                if (!cmd || !arg1) {
                    printf("Uso: get <param> | set <param> <valor>\n");
                    continue;
                }

                for(int i=0; i<NUM_COMMANDS; i++){
                    if(strcmp(arg1, commands[i].name) == 0){
                        if(strcmp(cmd, "set") == 0 && arg2){
                            if(commands[i].set){
                                commands[i].set(arg2);
                            }else{
                                printf("Comando no permitido para ese parámetro\n");
                            }
                        }else{
                            if(commands[i].get){
                                commands[i].get(NULL);
                            }
                        }
                        found = true;
                        break;
                    }
                }

                // // Buscar el parámetro
                // command_param_t *found = NULL;
                // for (int i = 0; i < NUM_PARAMS; i++) {
                //     if (strcmp(arg1, cli_params[i].name) == 0) {
                //         found = &cli_params[i];
                //         break;
                //     }
                // }

                // // Ejecutar comando
                // if (strcmp(cmd, "get") == 0 && found->read) {
                //     cli_get(found);
                // } else if (strcmp(cmd, "set") == 0 && found->write) {
                //     if(arg2){
                //         cli_set(found);
                //     }
                // } else {
                //     printf("Comando no permitido para ese parámetro\n");
                // }

                if (!found) {
                    printf("Parámetro desconocido: %s\n", arg1);
                    continue;
                }


                index = 0;

            }else{
                if (index < RESPONSE_MAX_LEN - 1) {
                    user_input[index++] = c;
                } else {
                    user_input[index] = '\0';
                    index = 0;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Tarea encargada de enviar solicitudes para leer el bh1750 a la tarea guardiana I2C
 * 
 * @param params 
 */
void bh1750_task(void *params){
    bh1750_t bh1750_context = {
        .lux = 0
    };
    i2c_guardian_t bh1750_guardian = {
        .device = bh1750_device,
        .return_queue = q_bh1750,
        .callback = (void *)i2c_guard_bh1750,
        .context = &bh1750_context,
    };

    for(;;){
        xQueueSend(q_i2c_guardian, &bh1750_guardian, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(BH1750_SAMPLE_TIME_MS));
    }
}


/**
 * @brief Tarea encargada de enviar solicitudes para leer el RTC a la tarea guardiana I2C
 *
 * @param pvParameters
 */
void rtc_task(void *pvParameters){
    i2c_guardian_t guardian;

    guardian = (i2c_guardian_t){
        .device = rtc_device,
        .return_queue = q_rtc,
        .callback = (void *)ds1307_get_time,
        .context = &g_rtc,
        .overwrite = true
    };

    char log[64];

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

void calibrate_task(void *params){

    float lux;

    vTaskSuspend(control_task_handler);

    while(1){

        xQueuePeek(q_lux, &lux, portMAX_DELAY);

    }
}

/**
 * @brief Tarea encargada de realizar el control de luminosidad
 *
 * @param params
 */
void control_task(void *params){

    float value_to_control = 0;
    // float kd = KD;
    // float kp = KP;
    // float ki = KI;
    control_params_t pid_params = {
        .kp = KP,
        .ki = KI,
        .kd = KD
    };

    xQueueSend(q_control_params, &pid_params, portMAX_DELAY); // Cargo los parametros en la cola

    float error, prev_error, diferential_error, integral_error;
    float dt;
    uint16_t pwm = 0;
    float h = PWM_WRAP / MAX_SET_POINT;
    float pid;
    int set_point = 0;
    
    set_point = SET_POINT;
    #if PRINT_CONTROL
    pwm_set_gpio_level(PIN_PWM, 0);
    #endif
    absolute_time_t start_time = get_absolute_time();
    absolute_time_t prev_time = start_time;

    xQueueOverwrite(q_pwm, &pwm);

    for(;;){
        start_time = get_absolute_time();
        xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
        xQueuePeek(q_control, &set_point, portMAX_DELAY);
        xQueueReceive(q_lux, &value_to_control, portMAX_DELAY);

        #if LUX_CALIBRATION
            if(pwm>=4096) pwm=0;
            pwm_set_gpio_level(PIN_PWM,pwm);
            pwm+=4;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        #endif

        error = (float)set_point - value_to_control;

        dt = (float)absolute_time_diff_us(prev_time, start_time)/1e3;

        diferential_error = (error - prev_error)/dt;

        integral_error += error * dt;

        if(integral_error>=4096.0){
            integral_error=4096.0;
        }

        pid = error * pid_params.kp + integral_error * pid_params.ki;//+ diferential_error * kd + integral_error * ki;

        if(pid<0){
            pid = 0;
        }

        pwm = (uint16_t)(pid * h);

        // Saturación
        if(pwm>=PWM_WRAP){
            pwm = PWM_WRAP;
        }
        if(pwm<0){
            pwm = 0;
        }

        xQueueOverwrite(q_pwm, &pwm);

        #ifdef DEBUG_CONTROL
            printf("PID:%.2f, PWM:%d, error: %.2f, lux:%.2f, set_point:%d\n", pid, pwm, error, value_to_control, set_point);
            // printf("lux:%.2f, set_point:%d, time:%.2f\n", value_to_control, set_point, (float) absolute_time_diff_us(start_time, get_absolute_time())/1000.0);
        #endif

        // if(error*error/(set_point*set_point) <= 0.01){
        //     continue;
        // }
        
        pwm_set_gpio_level(PIN_PWM,PWM_WRAP - pwm);
        // pwm_set_gpio_level(PIN_PWM, 4095);

        prev_time = get_absolute_time();    
        prev_error = error;
    }
}

#if PRINT_CONTROL
float lux_raw_bh[PRINT_SIZE];
float lux_raw_temt[PRINT_SIZE];
float lux_[PRINT_SIZE];
int   print_index=0;
#endif

/**
 * @brief Tarea que lee los sensores y enviar los datos a graficar y controlar
 *
 * @param params
 */
void get_lux_task(void *params){
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
    float set_point = user.sp;
    float coef_fusion = 1;
    uint16_t raw_adc_values;
    uint8_t c = 0;
    uint8_t samples = 0;

    float x_est = 0.0f;  // Estimación inicial (lux)
    float P = 1.0f;      // Incertidumbre inicial
    float Q = 0.01f;     // Ruido de proceso (ajustable)
    float R = 0.5f;      // Ruido de medición (depende del TMT6000 y el ADC)

    kalman_t kalman = {
        .x_est = 0.0f,
        .P = 1.0f,
        .Q = 0.01f,
        .R = 0.5f
    };

    float prom = 0.0;
    float calib = 2200.0;

    uint32_t event;

    absolute_time_t start_time = get_absolute_time();

    float alpha = 0.8;

    xQueueSend(q_alpha, &alpha, portMAX_DELAY);// Cargo el alpha inicial 
    xQueueSend(q_calib_temt, &calib, portMAX_DELAY);
    xQueueSend(q_kalman_get, &kalman, portMAX_DELAY);

    adc_run(true);

    for(;;){
        if(xQueueReceive(q_raw_adc_values, &raw_adc_values, portMAX_DELAY)){

            // printf("Tiempo de get_lux: %lld us\n", absolute_time_diff_us(start_time, get_absolute_time()));
            if(xQueueReceive(q_kalman, &kalman, 0) == pdTRUE); // Si recibo un valor de kalman actualizo

            xQueuePeek(q_alpha, &alpha, portMAX_DELAY);
            xQueuePeek(q_calib_temt, &calib, portMAX_DELAY);

            lux_temt6000 = temt6000_get_lux(raw_adc_values, calib);

            if(bh1750.lux >=100 && bh1750.lux <= 1000){ // El tramo lineal del temt6000 es desde 100-1000 lux
                lux = kalman_update(lux_temt6000, &kalman); // Filtro de Kalman
            }else{
                lux = bh1750.lux;
            }

            xQueueOverwrite(q_kalman_get, &kalman);
            // printf("Kalman actual Q: %.2f, R: %.2f\n", kalman.Q, kalman.R);


            // lux = lux_temt6000;
            
            if(xQueueReceive(q_bh1750, &bh1750, 0) == pdPASS)
            {
                lux = ((float)bh1750.lux) * (alpha) + lux * (1-alpha);
            }

            if(xTaskNotifyWait(0, 0, &event, 0) == pdPASS){
                lux = bh1750.lux;
            }
            
            xQueueOverwrite(q_lux, &lux);
            // xQueueSend(q_lux, &lux, portMAX_DELAY); // Uso Send en lugar de peek ya que la velocidad de muestreo es menor a la de control

            #if PRINT_CONTROL || LUX_CALIBRATION
                printf("lux: %f , temt:%f , bh:%d \n", lux, lux_temt6000, bh1750.lux);
                // xQueueSend(q_control, &set_point, portMAX_DELAY);
                // lux_[print_index] = lux;
                // lux_raw_bh[print_index] = bh1750.lux;
                // lux_raw_temt[print_index] = lux_temt6000;
                // print_index++;
                // if(print_index >= PRINT_SIZE){
                //     print_index = 0;
                //     xTaskNotify(user_task_handler, 0, eNoAction);
                // }
            #endif

            #if PRINT_RAW_DATA
                printf("%.2f , %d \n", lux_temt6000, bh1750.lux);
            #endif

            #ifdef DEBUG_GET_LUX
                // xQueueSend(q_send_uart, &lux, portMAX_DELAY); // Envio el valor de lux a la tarea que envia por UART
                printf("TEMT6000: %f, BH1750: %d, cte: %f\n", lux_temt6000, bh1750.lux, lux_temt6000/(float)bh1750.lux);
            #endif

            #if DISPLAY_OLED
            c++;

            if(c==MAX_COUNT){ // Cada una cierta cantidad de muestras enviare la muestra a la tarea que se encarga de la ui
                c = 0;
                xQueueSend(q_values_to_show, &lux, portMAX_DELAY);
            }
            #endif
            // start_time = get_absolute_time();
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Se espera una distancia entre muestras de 1.3ms
        adc_run(true);
        adc_irq_set_enabled(true);
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

#if DISPLAY_OLED
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
    uint32_t delta_time = 0, last_time = 0;

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

    last_time = get_absolute_time();//pdTICKS_TO_MS(xTaskGetTickCount());

    set_point_0 = user.sp;
    set_point_final = user.sp;
    bool motion = true;

    user_view.menu = user.menu;
    char log[64];

    ds1307_t rtc;
    user_view.change_value_mode = true;

    uint8_t prev_set = 0;

    for(;;){
        set_user_event_count = uxSemaphoreGetCount(set_user_event); // Tomo la cantidad de veces quese presiono el boton del encoder

        if(xQueueReceive(encoder_event, &encoder_increment, 0)); // En caso de rotar el encoder recibo el valor en ticks de la rotacion

        // if(xQueueReceive(q_rtc, &rtc, 0) && motion){
        //     user_view.hour = rtc.time.hours;
        //     user_view.minute = rtc.time.minutes;
        //     user_view.sencond = rtc.time.seconds;

        //     user_view.year = rtc.time.year;
        //     user_view.month = rtc.time.month;
        //     user_view.day = rtc.time.date;

        //     user = user_view;
        // }

        // user_view.menu = uxSemaphoreGetCount(change_event) % 3; // Tomo la cantidad de veces que se presiono el boton de cambio de menu
        // user.menu = user_view.menu;

        if(set_user_event_count%2 == 1){ // Entra en modo cambio de parametros
            if(prev_set != set_user_event_count){
                user_view = user;
            }
            motion = true;
            user_view.change_value_mode = true;

            if(user_view.menu == params_menu){
                switch (user_view.select)
                {
                case set_sp:
                    user_view.sp_0 += encoder_increment; // Actualizo el set point del usuario
                    if(user_view.sp_0 < 0){
                        user_view.sp_0 = 0; // Evito que el set point sea negativo
                    }
                    if(user_view.sp_0 > MAX_SET_POINT){
                        user_view.sp_0 = MAX_SET_POINT; // Evito que el set point sea mayor al maximo
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
            encoder_increment = 0;

            prev_set = set_user_event_count;
        }else{
            motion = false;
            user_view.change_value_mode = false;
            user.sp = user_view.sp_0;

            if(user_view.menu == params_menu){
                user_view.select = (user_view.select + encoder_increment) % not_show; // Actualizo la seleccion del usuario
            }
            if(user_view.menu == config_menu){
                user_view.select = (user_view.select + encoder_increment) % 9;
            }
            if(user_view.menu == log_menu){
                user_view.select = (user_view.select + encoder_increment) % 2;
            }
            user.select = user_view.select;
            encoder_increment = 0;
        }

        if((set_user_event_count+1) %3 == 0 && set_user_event_count != 0){ // La tercera vez que toco el boton setea los valores del usuario
            user = user_view;
            prev_set = 0;
            last_time = get_absolute_time(); // Reinicia el tiempo para la rampa
            xQueueReset(set_user_event);
        }

        set_point = user.sp;
        set_point_0 = user.sp_0;
        set_point_final = user.sp_f;
        // if(set_user_event_count % 2 == 1){
        //     motion = false;
        //     ui.p_user = &user_view;
        //     user_view.change_value_mode = true; // Indico que se esta cambiando el valor del usuario

        //     if(user_view.menu == params_menu){
        //         switch (user_view.select)
        //         {
        //         case set_sp:
        //             user_view.sp += encoder_increment; // Actualizo el set point del usuario
        //             if(user_view.sp < 0){
        //                 user_view.sp = 0; // Evito que el set point sea negativo
        //             }
        //             if(user_view.sp > MAX_SET_POINT){
        //                 user_view.sp = MAX_SET_POINT; // Evito que el set point sea mayor al maximo
        //             }
        //             break;
        //         case set_sp_f:
        //             user_view.sp_f += encoder_increment; // Actualizo el set point del usuario
        //             if(user_view.sp_f < 0){
        //                 user_view.sp_f = 0; // Evito que el set point sea negativo
        //             }
        //             if(user_view.sp_f > MAX_SET_POINT){
        //                 user_view.sp_f = MAX_SET_POINT; // Evito que el set point sea mayor al maximo
        //             }
        //             break;
        //         case set_time:
        //             user_view.rise_time_ms += encoder_increment; // Actualizo el set point del usuario
        //             if(user_view.rise_time_ms < 0){
        //                 user_view.rise_time_ms = 0; // Evito que el set point sea negativo
        //             }
        //             if(user_view.rise_time_ms > MAX_RISE_TIME){
        //                 user_view.rise_time_ms = MAX_RISE_TIME; // Evito que el set point sea mayor al maximo
        //             }
        //         default:
        //             break;
        //         }
        //     }

        //     if(user_view.menu == config_menu){
        //         ui.p_user = &user_view;

        //         switch (user_view.select)
        //         {
        //             case hour_config:
        //                 if(user_view.hour + encoder_increment >0)
        //                     user_view.hour += encoder_increment;
        //                 user_view.hour %= 24;
        //                 rtc.time.hours = user_view.hour;
        //             break;
        //             case second_config:
        //                 if(user_view.sencond + encoder_increment >0)
        //                     user_view.sencond += encoder_increment;
        //                 user_view.sencond %= 60;
        //                 rtc.time.seconds = user_view.sencond;
        //             break;
        //             case minute_config:
        //                 if(user_view.minute + encoder_increment >0)
        //                     user_view.minute += encoder_increment;
        //                 user_view.minute %= 60;
        //                 rtc.time.minutes = user_view.minute;
        //             break;
        //             case day_config:
        //                 if(user_view.day + encoder_increment > 0)
        //                     user_view.day += encoder_increment;
        //                 user_view.day %= 32;
        //                 rtc.time.date = user_view.day;
        //             break;
        //             case month_config:
        //                 if(user_view.month + encoder_increment > 0)
        //                     user_view.month += encoder_increment;
        //                 user_view.month %= 13;
        //                 rtc.time.month = user_view.month;
        //             break;
        //             case year_config:
        //                 if(user_view.year + encoder_increment > 0)
        //                     user_view.year += encoder_increment;
        //                 rtc.time.year = user_view.year;
        //             break;
        //             case min_config:
        //                 if(user_view.min + encoder_increment > 0)
        //                     user_view.min += encoder_increment;
        //             break;
        //             case max_config:
        //                 if(user_view.max + encoder_increment > 0)
        //                     user_view.max += encoder_increment;
        //             break;
        //         default:
        //             break;
        //         }
        //     }
            
        //     encoder_increment = 0;
        // }else{
        //     user_view.change_value_mode = false;

        //     if(user_view.menu == params_menu){
        //         user_view.select = (user_view.select + encoder_increment) % not_show; // Actualizo la seleccion del usuario
        //     }
        //     if(user_view.menu == config_menu){
        //         user_view.select = (user_view.select + encoder_increment) % 9;
        //     }
        //     if(user_view.menu == log_menu){
        //         user_view.select = (user_view.select + encoder_increment) % 2;
        //     }
        //     user.select = user_view.select;
        //     encoder_increment = 0;
        // }

        // if(uxSemaphoreGetCount(set_user_event)==100){
        //     xQueueReset(set_user_event);
        // }

        // delta_time = pdTICKS_TO_MS(xTaskGetTickCount()) - last_time;

        // if(set_point <= user.sp_f){
            // }else{
                //     printf("Se excedio el limite\n");
                // }
                
                // if(set_point >= user.sp_0){
                    //     last_time = get_absolute_time();//pdTICKS_TO_MS(xTaskGetTickCount());
                    // }
                    
                    
        if(user.rise_time_ms >0){
            delta_time = absolute_time_diff_us(last_time, get_absolute_time()) / 1000;
            if(delta_time>user.rise_time_ms){
                last_time = get_absolute_time();
                // printf("excedido!!! last_time: %d\n", last_time);
            }
            set_point = (int)(((float)(set_point_final - set_point_0) / (float)user.rise_time_ms) * (float)delta_time) + set_point_0;
            user.sp = set_point; // Muevo el set point de la pantalla
            // printf("sp: %d, sp_final: %d, sp_0: %d, rise_time: %d, dt: %d\n", set_point, set_point_final, set_point_0, user.rise_time_ms, delta_time);
        }else{
            user.sp = user.sp_0;
            set_point = user.sp;
        }
        // else{
        //     set_point = user.sp;
        // }

        // // printf("set_point: %d rise_time: %d set_point_final: %d\n", set_point, user.rise_time_ms, user.sp_f);

        // if(set_user_event_count % 2 == 0 && set_user_event_count != 0){ // Si es multiplo de 2
        //     // user.sp = user_view.sp;
        //     // user.rise_time_ms = user_view.rise_time_ms;
        //     // user.sp_f = user_view.sp_f;
        //     user = user_view;
        //     xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);

        //     set_point_0 = user.sp;
        //     set_point_final = user.sp_f;

        //     ui.p_user = &user;
            
        //     if(user.menu == log_menu){
        //         switch (user.select)
        //         {
        //         case 1:
        //             xSemaphoreGive(read_logs_event);
        //             break;
        //         default:
        //             xSemaphoreGive(erase_logs_event);
        //             break;
        //         }
        //     }else{
        //         sprintf(log, "%02d:%02d:%02d-%02d/%02d/%02d-lux:%d-sp:%d", user.hour, user.minute, user.sencond, user.day, user.month, user.year, user_view.lux, user.sp);
        //         xQueueSend(q_to_storage, log, portMAX_DELAY);
        //     }
            
        //     xQueueSend(q_user_config, &user, portMAX_DELAY);
            
        //     // printf("Seteado en: \n sp: %d\n sp_f: %d\n rise_time_ms: %d\n", user.sp, user.sp_f, user.rise_time_ms);

        //     motion = true;
        //     xQueueReset(set_user_event);
        // }

        // // xQueueSend(q_control, &set_point, portMAX_DELAY);

        xQueueOverwrite(q_control, &set_point);

        if(xQueueReceive(q_values_to_show, &lux, 0) == pdPASS){ // No puedo usar un Peek ya que la tarea controladora lo consume mas rapido de lo que puedo leer

            user.lux = (uint32_t)lux;
            user_view.lux = user.lux;

            if(motion){
                ui.p_user = &user_view;
                // printf("imprimiendo user_view %d\n", u);
            }else{
                ui.p_user = &user;
                // printf("imprimiendo user\n");
            }

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
#else

void user_task(void *params){
    uint32_t event;
    while(1){
        xTaskNotifyWait(0, 0, &event, portMAX_DELAY);
        // for(int i=0; i<PRINT_SIZE; i++){
        //     printf("%.2f, %.2f, %.2f \n", lux_[i], lux_raw_bh[i], lux_raw_temt);
        // }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif

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
 * @brief Tarea encargada de manejar la memoria flash
 * 
 * @param params 
 */
void storage_task(void *params) {
    char log[64];
    user_t user_config;

    while(true){
        if(xQueueReceive(q_to_storage, log, 0)){
            save_log(log);
            #ifdef DEBUG_LOGS
                printf("LOG GUARDADO! %s\n", log);
            #endif
        }
        if(xSemaphoreTake(read_logs_event, 0)){
            read_all_logs();
        }
        if(xQueueReceive(q_user_config, &user_config, 0)){
            save_u16_as_bytes((uint16_t)user_config.sp, 0);
            save_u16_as_bytes((uint16_t)user_config.rise_time_ms, 1);
            save_u16_as_bytes((uint16_t)user_config.sp_f, 2);
        }
        if(xSemaphoreTake(erase_logs_event, 0)){
            erase_all_logs();
            save_u16_as_bytes((uint16_t)500, 0);
            save_u16_as_bytes((uint16_t)0, 1);
            save_u16_as_bytes((uint16_t)1000, 2);
        }
        vTaskDelay(50);
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

    gpio_set_function(PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RX, GPIO_FUNC_UART);

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

    pwm_config_set_clkdiv(&config, 1.0f);

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

    q_raw_adc_values = xQueueCreate(10, sizeof(uint16_t));
    q_values_to_show = xQueueCreate(BUFF_SIZE*5, sizeof(float));

    q_lux = xQueueCreate(10, sizeof(float));
    q_control = xQueueCreate(10, sizeof(float));

    q_rtc = xQueueCreate(5, sizeof(ds1307_t));
    q_rtc_config = xQueueCreate(5, sizeof(ds1307_t));

    q_to_storage = xQueueCreate(10, 64*sizeof(char));
    q_user_config = xQueueCreate(10, sizeof(user_t));

    q_pwm = xQueueCreate(2, sizeof(uint16_t));

    q_kalman = xQueueCreate(1, sizeof(kalman_t));
    q_kalman_get = xQueueCreate(1, sizeof(kalman_t));
    q_alpha = xQueueCreate(1, sizeof(float));

    #ifdef PRINT_VALUES_MODE
    q_send_uart = xQueueCreate(BUFF_SIZE, sizeof(float));
    #endif

    q_i2c_guardian = xQueueCreate(BUFF_SIZE, sizeof(i2c_guardian_t));
    q_bh1750 = xQueueCreate(BUFF_SIZE, sizeof(bh1750_t));

    q_control_params = xQueueCreate(1, sizeof(control_params_t));
    q_calib_temt = xQueueCreate(1, sizeof(float));

    set_user_event = xSemaphoreCreateCounting(2,0);
    change_event = xSemaphoreCreateCounting(100,0);
    encoder_event = xQueueCreate(2, sizeof(int));
    read_logs_event = xSemaphoreCreateBinary();
    erase_logs_event = xSemaphoreCreateBinary();

    adc_config();
    i2c_config();
    bh1750_init();
    gpio_config();
    uart_init(uart0, 9600);
    config_pwm(PIN_PWM, PWM_CLK);

    g_rtc.i2c = i2c0;
    g_rtc.addr = DS1307_ADDRESS;
    ds1307_init(&g_rtc);
    ds1307_get_time(&g_rtc);
    
    pwm_set_gpio_level(PIN_PWM, 2000);

     if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    int pwm = 0;

    uint16_t adc;

    #ifndef PRINT_VALUES_MODE
        oled.external_vcc = false;
        ui_init(&oled, I2C_PORT, &user);
    #endif

    #if PRINT_CONTROL
    int led_state = 0;
    int c;
   
    
    printf("Esperando para inciar\n");
    
    // c = getchar();

    // for(int i=0; i<10; i++){
    //     cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
    //     led_state = !led_state;
    //     sleep_ms(1000);
    // }
    #endif

    uint16_t values[3];

    for(int i = 0; i < 3; i++) {
        values[i] = read_log_u16(i);
    }

    user.sp = values[0];
    user.rise_time_ms = values[1];
    user.sp_f = values[2];

    xTaskCreate(
        get_lux_task,
        "get_lux_task",
        configMINIMAL_STACK_SIZE*6,
        NULL,
        tskIDLE_PRIORITY + 2,
        &get_lux_task_handler
    );

    xTaskCreate(
        bh1750_task,
        "bh1750_task",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL
    );

    #if DISPLAY_OLED
    // xTaskCreate(
    //     storage_task,
    //     "storage_task",
    //     configMINIMAL_STACK_SIZE*2,
    //     NULL,
    //     tskIDLE_PRIORITY+2,
    //     NULL
    // );
    #endif

    // #ifdef PRINT_VALUES_MODE
    // xTaskCreate(
    //     send_uart_task,
    //     "send_uart_task",
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
        rtc_task,
        "RTC Task", 
        configMINIMAL_STACK_SIZE*4,
        NULL, 
        tskIDLE_PRIORITY+1,
        NULL
    );

    // #endif

    xTaskCreate(
        i2c_guardian_task,
        "i2c_guardian_task",
        configMINIMAL_STACK_SIZE*4,
        NULL,
        tskIDLE_PRIORITY + 3,
        NULL
    );
    
    xTaskCreate(
        user_task,
        "user_task",
        configMINIMAL_STACK_SIZE*4,
        NULL,
        tskIDLE_PRIORITY + 1,
        &user_task_handler
    );

    xTaskCreate(
        control_task,
        "control_task",
        configMINIMAL_STACK_SIZE*3,
        NULL,
        tskIDLE_PRIORITY + 3,
        &control_task_handler
    );

    xTaskCreate(
        cli_task,
        "cli_task",
        configMINIMAL_STACK_SIZE*4,
        NULL,
        tskIDLE_PRIORITY + 4,
        NULL
    );

    vTaskStartScheduler();

    while (1) {
    }
}
