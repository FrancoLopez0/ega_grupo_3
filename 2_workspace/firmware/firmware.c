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
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h"
#include "math.h"

#include "modules/temt6000/temt6000.h"
#include "modules/bh1750/bh1750.h"
#include "modules/ui/ui.h"
#include "modules/ds1307/ds1307.h"
#include "modules/flash/flash.h"
// #include "modules/littlefs-lib/pico_hal.h"
#include "pico_lfs.h"

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
    bool prev_dt; // Estado anterior del dt del encoder
    int user_increment; // Incremento del usuario
    int user_select; // Seleccion del usuario
    TickType_t last_valid_edge; // Ultimo flanco valido
    TickType_t debounce_time; // Tiempo de debounce
    TickType_t last_edge_time; // Ultimo tiempo de flanco
    TickType_t current_edge_time; // Tiempo actual de flanco
    TickType_t rotation_period; // Periodo de rotacion
    TickType_t last_valid_step_time;
}encoder_t;

typedef struct __attribute__((packed)) {
    uint16_t lux;
    rtc_time_t time;
}log_t;

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

QueueHandle_t q_raw_adc_values, q_values_to_show, q_control, q_lux, q_rtc, q_rtc_config, q_to_storage, q_user_config, q_pwm, q_kalman, q_alpha, q_control_params, q_calib_temt, q_kalman_get, q_buff_to_print;

TaskHandle_t user_task_handler, get_lux_task_handler, control_task_handler, storage_task_handler;

ds1307_t g_rtc;

#ifdef PRINT_VALUES_MODE
QueueHandle_t q_send_uart;
#endif
QueueHandle_t q_i2c_guardian, q_to_print;
QueueHandle_t q_bh1750;
SemaphoreHandle_t set_user_event, encoder_event, change_event, read_logs_event, erase_logs_event, toggle_control_event;
ssd1306_t oled;

user_t user = {
    .sp = 600,
    .mode = true,
    .lux = MAX_LUX/2,
    .select = set_sp,
    .change_value_mode = false,
    .rise_time_ms = 0,
    .sp_f = 700,
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
 * @brief Lee y añade el valor luego del ultimo dato (separado por 0xDEAD)
 * 
 * @param user 
 * @return flash_err_t 
 */
flash_err_t save_state(user_t *user) {
    
    uint8_t page_buff[FLASH_PAGE_SIZE]; // Búfer para leer la página (solo para escaneo)
    uint8_t data_block[STATE_BLOCK_SIZE]; // Búfer PEQUEÑO para el nuevo dato (Fuente de escritura)

    read_page(0, page_buff); 

    int i = 0; // Índice (offset) de la página
    
    // 1. ESCANEO (Buscar el fin de la cadena de datos)
    for(i=0; i < FLASH_PAGE_SIZE - STATE_BLOCK_SIZE; i += STATE_BLOCK_SIZE){
        // Si encontramos el separador 0xDEAD, el nuevo dato debe ir después.
        if(page_buff[i + STATE_BLOCK_SIZE - 2] == 0xDE && page_buff[i + STATE_BLOCK_SIZE - 1] == 0xAD){
            // El nuevo offset 'i' será el inicio del próximo bloque (i + 12)
            // No hacemos 'i += 2', sino que el bucle lo manejará en la siguiente iteración.
            continue; 
        } else if (page_buff[i] == 0xFF && page_buff[i + 1] == 0xFF) {
            // Si encontramos 0xFF, asumimos que hemos llegado al área virgen.
            break;
        } else {
            // Si encontramos datos incompletos o corruptos (no 0xDEAD y no 0xFF), 
            // no podemos asumir dónde escribir. Aquí podrías borrar/reiniciar.
            // Por simplicidad, asumimos que el bucle encuentra el 0xFF o 0xDEAD.
        }
    }
    
    // Al salir del bucle, 'i' es el OFFSET (0, 12, 24, etc.) donde se debe escribir.

    // 2. GESTIÓN DE PÁGINA LLENA
    if(i > FLASH_PAGE_SIZE - STATE_BLOCK_SIZE){
        // La página 0 está llena y no queda espacio para el nuevo bloque de 12 bytes.
        // Aquí deberías avanzar a la página 1, y BORRAR el sector correspondiente.
        printf("Página 0 llena. Necesita avanzar a la siguiente página y borrar.\n");
        return FLASH_PAGE_FULL; // O manejar la lógica de la siguiente página.
    }
    
    // 3. PREPARAR EL BÚFER DE ESCRITURA (Usando el búfer pequeño)
    // Se usa un búfer pequeño (data_block) para evitar pasar la página completa.
    data_block[0] = user->sp & 0xFF;
    data_block[1] = (user->sp >> 8) & 0xFF;
    data_block[2] = user->sp_f & 0xFF;
    data_block[3] = (user->sp_f >> 8) & 0xFF;
    data_block[4] = user->rise_time_ms & 0xFF;
    data_block[5] = (user->rise_time_ms >> 8) & 0xFF;
    data_block[6] = user->min & 0xFF;
    data_block[7] = (user->min >> 8) & 0xFF;
    data_block[8] = user->max & 0xFF;
    data_block[9] = (user->max >> 8) & 0xFF;
    data_block[10] = 0xDE;
    data_block[11] = 0xAD;

    // 5. LLAMADA DE ESCRITURA CORRECTA
    // Escribir SOLO el bloque de 12 bytes en el offset 'i', usando el búfer PEQUEÑO.
    if(append_data_to_page(0, i, data_block, STATE_BLOCK_SIZE) != FLASH_OK){
        return FLASH_WRITE_ERROR;
    }

    return FLASH_OK;
}

flash_err_t read_state(user_t *user) {
    
    uint8_t aux_buff[FLASH_PAGE_SIZE];
    read_page(0, aux_buff);

    int latest_i = -1; // Almacena el offset del BLOQUE del último dato válido
    
    // 1. ESCANEO LINEAL PARA ENCONTRAR EL ÚLTIMO BLOQUE VÁLIDO
    // Iteramos en pasos de 12 bytes
    for(int i = 0; i < FLASH_PAGE_SIZE; i += STATE_BLOCK_SIZE){
        
        // Comprobación de fin de datos: ¿Hemos llegado al área virgen (0xFF)?
        if(aux_buff[i] == 0xFF && aux_buff[i+1] == 0xFF){
            break; // Salimos, 'latest_i' ya tiene la posición del último dato
        }
        
        // Comprobación de validez: ¿Tiene el marcador 0xDEAD?
        // El 0xDEAD está en aux_buff[i+10] y aux_buff[i+11]
        if(aux_buff[i + 10] == 0xDE && aux_buff[i + 11] == 0xAD){
            latest_i = i; // Encontramos un bloque válido, actualizamos el puntero
        } else {
            // Esto indica datos corruptos. Es mejor parar aquí.
            printf("Error: Datos corruptos encontrados en offset %d.\n", i);
            break; 
        }
    }
    
    // 2. GESTIÓN DE ERRORES: ¿Se encontró algún dato?
    if(latest_i == -1){
        printf("Error: Página vacía o sin datos válidos.\n");
        return FLASH_READ_ERROR;
    }
    
    // 3. EXTRACCIÓN DEL ÚLTIMO DATO
    // El último dato válido comienza en latest_i.
    
    user->sp             = (aux_buff[latest_i+0] << 0) | (aux_buff[latest_i+1] << 8);
    user->sp_f           = (aux_buff[latest_i+2] << 0) | (aux_buff[latest_i+3] << 8);
    user->rise_time_ms   = (aux_buff[latest_i+4] << 0) | (aux_buff[latest_i+5] << 8);
    user->min            = (aux_buff[latest_i+6] << 0) | (aux_buff[latest_i+7] << 8);
    user->max            = (aux_buff[latest_i+8] << 0) | (aux_buff[latest_i+9] << 8);

    printf("USER READ: %d %d %d %d %d (Desde offset %d)\n", user->sp, user->sp_f, user->rise_time_ms, user->min, user->max, latest_i);

    return FLASH_OK;
}
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

void set_point_get(void *param){
    vTaskSuspend(user_task_handler);
    printf("Set point: %d\n", user.sp);
    vTaskResume(user_task_handler);
}

void set_point_set(void *param){
    vTaskSuspend(user_task_handler);
    char *arg = (char *)param;
    int value = atoi(arg);

    if(value>=MAX_SET_POINT || value<=MIN_SET_POINT){ 
        printf("Set point out of range\n");
        vTaskResume(user_task_handler);
        return;
    }

    user.sp_0 = value;
    user.sp = value;
    // xQueueSend(q_control, &user.sp, portMAX_DELAY);
    vTaskResume(user_task_handler);
    xQueueSend(q_user_config, &user, portMAX_DELAY);
    xTaskNotify(user_task_handler, 0, eSetValueWithOverwrite);
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
    if(value>=MAX_SET_POINT || value<=MIN_SET_POINT){ 
        printf("Set point out of range\n");
        vTaskResume(user_task_handler);
        return;
    }
    user.sp_f = value;
    xQueueSend(q_user_config, &user, portMAX_DELAY);
    vTaskResume(user_task_handler);
    // set_point_f_get(NULL);
}

void user_params_get(void *param){
    vTaskSuspend(user_task_handler);
    printf("setPoint: %d, setPointFinal: %d, riseTime: %d, min: %d, max: %d\n", user.sp_0, user.sp_f, user.rise_time_ms, user.min, user.max);
    vTaskResume(user_task_handler);
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
    if((value>=MAX_RISE_TIME || value<=MIN_RISE_TIME) && value!=0){ 
        printf("Value out of range\n");
        vTaskResume(user_task_handler);
        return;
    }
    user.rise_time_ms = value;
    vTaskResume(user_task_handler);
    // rise_time_get(NULL);
    xQueueSend(q_user_config, &user, portMAX_DELAY);
}

void pwm_get(void *params){
    uint16_t pwm;
    xQueuePeek(q_pwm, &pwm, portMAX_DELAY);
    printf("PWM: %d\n", pwm);
}

void min_get(void *param){
    vTaskSuspend(user_task_handler);
    printf("min: %d\n", user.min);
    vTaskResume(user_task_handler);
}

void min_set(void *param){
    vTaskSuspend(user_task_handler);
    char *arg = (char *)param;
    int value = atoi(arg);
    if(value<0 || value>MAX_SET_POINT){
        printf("Value out of range\n");
        vTaskResume(user_task_handler);
        return;
    }
    user.min = value;
    vTaskResume(user_task_handler);
    // min_get(NULL);
    xQueueSend(q_user_config, &user, portMAX_DELAY);
}

void max_get(void *param){
    vTaskSuspend(user_task_handler);
    printf("max: %d\n", user.max);
    vTaskResume(user_task_handler);
}

void max_set(void *param){
    vTaskSuspend(user_task_handler);
    char *arg = (char *)param;
    int value = atoi(arg);
    if(value<0 || value>MAX_SET_POINT){
        printf("Value out of range\n");
        vTaskResume(user_task_handler);
        return;
    }
    user.max = value;
    xQueueSend(q_user_config, &user, portMAX_DELAY);
    vTaskResume(user_task_handler);
    // max_get(NULL);
}

void bh1750_get(void *param){
    bh1750_t bh1750;
    xQueuePeek(q_bh1750, &bh1750, portMAX_DELAY);
    printf("bh1750: %d\n", bh1750.lux);
}

void temt6000_get(void *param){
    uint16_t adc_raw;
    float calib;
    xQueuePeek(q_calib_temt, &calib, portMAX_DELAY);
    xQueuePeek(q_raw_adc_values, &adc_raw, portMAX_DELAY);
    printf("Temt6000: %.2f\n", temt6000_get_lux(adc_raw, calib));
}

void alpha_get(void *param){
    float alpha;
    xQueuePeek(q_alpha, &alpha, portMAX_DELAY);
    printf("alpha: %.2f\n", alpha);
}

void alpha_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    if(value > 1 || value < 0){
        printf("Value out of range\n");
        return;
    }
    xQueueOverwrite(q_alpha, &value);
    // alpha_get(NULL);
}

void pid_kp_get(void *param){
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
    printf("kp: %.2f\n", pid_params.kp);
}

void pid_kp_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
    pid_params.kp = value;
    xQueueOverwrite(q_control_params, &pid_params);
    vTaskSuspend(user_task_handler);
    xQueueSend(q_user_config, &user, portMAX_DELAY);
    vTaskResume(user_task_handler);
}

void pid_ki_get(void *param){
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
    printf("ki: %.2f\n", pid_params.ki);
}

void pid_ki_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
    pid_params.ki = value;
    xQueueOverwrite(q_control_params, &pid_params);
    vTaskSuspend(user_task_handler);
    xQueueSend(q_user_config, &user, portMAX_DELAY);
    vTaskResume(user_task_handler);
    // pid_ki_get(NULL);
}

void pid_kd_get(void *param){
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
    printf("kd: %.2f\n", pid_params.kd);
}

void pid_kd_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);// Tomo todos los valores para no modificar los demas
    pid_params.kd = value;
    xQueueOverwrite(q_control_params, &pid_params);
    vTaskSuspend(user_task_handler);
    xQueueSend(q_user_config, &user, portMAX_DELAY);
    vTaskResume(user_task_handler);
    // pid_kd_get(NULL);
}

void pid_params_get(void *param){
    control_params_t pid_params;
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);// Tomo todos los valores para no modificar los demas
    printf("kp: %.2f, ki: %.2f, kd: %.2f\n", pid_params.kp, pid_params.ki, pid_params.kd);
}

void calib_get(void *param){
    float calib;
    xQueuePeek(q_calib_temt, &calib, portMAX_DELAY);
    printf("calib: %.2f\n", calib);
}

void calib_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    float calib;
    if(calib<0){
        printf("Value out of range\n");
        return;
    }
    calib = value;
    xQueueOverwrite(q_calib_temt, &calib);
    calib_get(NULL);
}

void kalman_Q_get(void *param){
    kalman_t kalman;
    xQueuePeek(q_kalman_get, &kalman, portMAX_DELAY);
    printf("q: %.2f\n", kalman.Q);
}

void kalman_Q_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    kalman_t kalman;
    xQueuePeek(q_kalman_get, &kalman, portMAX_DELAY);
    kalman.Q = value;
    xQueueSend(q_kalman, &kalman, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
    // kalman_Q_get(NULL);
}

void kalman_R_get(void *param){
    kalman_t kalman;
    xQueuePeek(q_kalman_get, &kalman, portMAX_DELAY);
    // printf("R: %.2f\n", kalman.R);
}

void kalman_R_set(void *param){
    char *arg = (char *)param;
    float value = atof(arg);
    kalman_t kalman;
    xQueuePeek(q_kalman_get, &kalman, portMAX_DELAY);
    kalman.R = value;
    xQueueSend(q_kalman, &kalman, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
    // kalman_R_get(NULL);
}

void filter_params_get(void *param){
    kalman_t kalman;
    float alpha;
    xQueuePeek(q_alpha, &alpha, portMAX_DELAY);
    xQueuePeek(q_kalman_get, &kalman, portMAX_DELAY);

    printf("alpha: %.2f, q: %.2f, r: %.2f\n", alpha, kalman.Q, kalman.R);
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
    if(value>=24 || value<0){
        printf("Value out of range\n");
        return;
    }
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.hours = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void rtc_set_minute(void *param){
    ds1307_t rtc;
    char *arg = (char *)param;
    int value = atoi(arg);
    if(value>=60 || value<0){
        printf("Value out of range\n");
        return;
    }
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.minutes = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void rtc_set_second(void *param){
    ds1307_t rtc;
    char *arg = (char *)param;
    int value = atoi(arg);
    if(value>=60 || value<0){
        printf("Value out of range\n");
        return;
    }
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.seconds = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void rtc_set_day(void *param){
    ds1307_t rtc;
    char *arg = (char *)param;
    int value = atoi(arg);
    if(value>=31 || value<0){
        printf("Value out of range\n");
        return;
    }
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.date = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void rtc_set_month(void *param){
    ds1307_t rtc;
    char *arg = (char *)param;
    int value = atoi(arg);
    if(value>=60 || value<0){
        printf("Value out of range\n");
        return;
    }
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.month = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void rtc_set_year(void *param){
    ds1307_t rtc;
    char *arg = (char *)param;
    int value = atoi(arg);
    if(value<0){
        printf("Value out of range\n");
        return;
    }
    xQueuePeek(q_rtc, &rtc, portMAX_DELAY);
    rtc.time.year = value;
    xQueueSend(q_rtc_config, &rtc, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void logs_get(void *param){
    if(param == NULL){
        xTaskNotify(storage_task_handler, 0, eSetValueWithOverwrite);
        return;
    }
    int cant = atoi(param);

    if(param<0){
        printf("Value out of range\n");
        return;
    }
    xTaskNotify(storage_task_handler, (uint32_t)cant, eSetValueWithOverwrite);
}

void toogle_control(void *param){
    // xTaskNotify(control_task_handler, E_TOGGLE_CONTROL, eSetValueWithOverwrite);
    xSemaphoreGive(toggle_control_event);
}

typedef struct {
    char *name;
    command_fn_t set;
    command_fn_t get;
}cmd_t;

cmd_t commands[]={
    {"logs", NULL, logs_get},
    {"user_params", NULL, user_params_get},
    {"pid_params", NULL, pid_params_get},
    {"filter_params", NULL, filter_params_get},
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

void sys_print(const char* msg){
    xQueueSend(q_to_print, msg, portMAX_DELAY);
}

// ===== HEADERS ====

const char *lux_arr_header = "lux_array";

enum pointer_type{
    BUFF_FLOAT,
    BUFF_INT
};

typedef struct{
    void *p_buff;
    int size;
    enum pointer_type type;
    char *header;
}buff_to_print_t;

/**
 * @brief Esta tarea se encarga de manejar la linea de comandos
 * @todo Añadir comandos
 * 
 * @param params 
 */
void cli_task(void *params){

    #define RESPONSE_MAX_LEN 128
    #define UART_ID uart0

    char cli_response[RESPONSE_MAX_LEN];
    char user_input[RESPONSE_MAX_LEN];
    char to_print[RESPONSE_MAX_LEN];
    char c;
    int index=0;
    bool found = false;
    int count = 0;
    float lux;
    bh1750_t bh1750;
    float temt6000;
    uint16_t adc;
    bool print_lux = PRINT_LUX;
    buff_to_print_t buff_to_print;

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

                if(!cmd) continue;

                if(strcmp(cmd, "stop") == 0){
                    print_lux = false;
                    continue;
                }

                if(strcmp(cmd, "continue") == 0){
                    print_lux = true;
                    continue;
                }

                if(strcmp(cmd, "toggle") == 0){
                    // xTaskNotify(control_task_handler, E_TOGGLE_CONTROL, eSetValueWithOverwrite);
                    xSemaphoreGive(toggle_control_event);
                    continue;
                }

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
                                commands[i].get(arg2);
                            }
                        }
                        found = true;
                        break;
                    }
                }

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

        if(xQueueReceive(q_to_print, &to_print, 0)){
            printf(to_print);
        }
        
        count++;
        if(count>=10){ // Cada 10 * 10ms = 100ms envio el dato de los lux actuales
            count = 0;
            if(print_lux){
                xQueuePeek(q_values_to_show, &lux, portMAX_DELAY);
                printf("lux: %d, time: 1\n", (uint16_t)lux);
            }
        }

        if(xQueueReceive(q_buff_to_print, &buff_to_print, 0)){
            printf(buff_to_print.header);
            printf(":");
            switch (buff_to_print.type)
            {
            case BUFF_FLOAT:
                for(int i=0; i<buff_to_print.size; i++){
                    printf("%.2f, ", ((float *)buff_to_print.p_buff)[i]);
                }
                break;
            case BUFF_INT:
                for(int i=0; i<buff_to_print.size; i++){
                    printf("%d, ", ((int *)buff_to_print.p_buff)[i]);
                }
                break;
            default:
                break;
            }
            printf("\n");
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

    // xQueueSend(q_control_params, &pid_params, portMAX_DELAY); // Cargo los parametros en la cola
    xQueuePeek(q_control_params, &pid_params, portMAX_DELAY);
    char msg[64];

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

    uint32_t event = 4;

    // static float lux_buffer[LUX_BUFFER_SIZE]={0.0};

    // buff_to_print_t buff_to_print = {
    //     .p_buff = lux_buffer,
    //     .size = LUX_BUFFER_SIZE,
    //     .type = BUFF_FLOAT,
    //     .header = "lux_buffer"
    // };

    bool pwm_state = true;
    int toggle_count = 0;

    for(;;){
        start_time = get_absolute_time();

        if(xSemaphoreTake(toggle_control_event,0)){
            pwm_state = !pwm_state;
            sprintf(msg,"control: %d \n", pwm_state);
            sys_print(msg);
        }

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

        // if(fabs(error) < 10){
        //     prev_error = error;
        //     sprintf(msg, "Dentro de la banda de error!\n");
        //     sys_print(msg);
        //     pwm_state ? pwm_set_gpio_level(PIN_PWM,pwm) : pwm_set_gpio_level(PIN_PWM,0);
        //     prev_time = get_absolute_time();   
        //     continue;
        // }

        dt = (float)absolute_time_diff_us(prev_time, start_time);

        diferential_error = (error - prev_error)/dt;

        integral_error += error * dt;

        if(fabs(integral_error)>=4096.0/h ){
            integral_error=4096.0/h;
        }

        if(fabs(diferential_error)>=10/h){
            diferential_error=10/h;
        }


        if(pid_params.kd == 0){
            pid = error * pid_params.kp + integral_error * pid_params.ki;
        }else{
            pid = error * pid_params.kp + integral_error * pid_params.ki + diferential_error * pid_params.kd;
        }

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
        
        // pwm_set_gpio_level(PIN_PWM,pwm);
        if(pwm_state) {
            pwm_set_gpio_level(PIN_PWM,pwm);
        }else{
            pwm_set_gpio_level(PIN_PWM,0);
            integral_error = 0;
            diferential_error = 0;
        }
        // pwm_set_gpio_level(PIN_PWM, 4095);
        // pwm_set_gpio_level(PIN_PWM,pwm);

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
    
    int increment = 0;
    int change = 0; // Almacena el resultado de la FSM: -1 (CCW), 0 (sin cambio/rebote), +1 (CW)

    int current_clk = encoder->clk;
    int current_dt = encoder->dt;

    // --- 1. Máquina de Estados Finitos (FSM) de Cuadratura ---
    // Esta lógica detecta CADA transición (4 por "clic")
    // y es inherentemente resistente al rebote (un rebote es +1 y luego -1, sumando 0).
    
    // Un cambio en CLK
    if (current_clk != encoder->prev_clk) {
        if (current_clk == 1) { // Flanco ascendente de CLK
            change = (current_dt == 0) ? 1 : -1; // Dirección 1 (CW)
        } else { // Flanco descendente de CLK
            change = (current_dt == 1) ? 1 : -1; // Dirección 1 (CW)
        }
    } 
    // Un cambio en DT (solo si CLK no cambió, para evitar doble conteo)
    else if (current_dt != encoder->prev_dt) { 
        if (current_dt == 1) { // Flanco ascendente de DT
            change = (current_clk == 1) ? 1 : -1; // Dirección 1 (CW)
        } else { // Flanco descendente de DT
            change = (current_clk == 0) ? 1 : -1; // Dirección 1 (CW)
        }
    }

    // (Nota: Si la dirección está invertida, simplemente invierte los 1 y -1 en la lógica de 'change')

    // --- 2. Actualización de Estados Anteriores ---
    // Actualizamos los estados para la próxima vez que se llame a la función.
    // Esto se hace siempre, incluso si fue un rebote.
    encoder->prev_clk = current_clk;
    encoder->prev_dt = current_dt;

    // --- 3. Cálculo de Velocidad (Solo si hubo un paso válido) ---
    
    // Si 'change' es 0, significa que no hubo cambio de estado o
    // los pines están en un estado "inválido" (ambos cambiaron a la vez).
    if (change == 0) {
        return 0;
    }

    // Si llegamos aquí, 'change' es +1 o -1 (un paso válido detectado)
    TickType_t now = xTaskGetTickCount();
    TickType_t rotation_period = now - encoder->last_valid_step_time;
    encoder->last_valid_step_time = now; // Actualizamos el tiempo del último paso VÁLIDO

    // --- 4. Asignación de Incremento (Basado en tu lógica original) ---
    // Ajusta estos valores según la sensibilidad que desees
    
    if (rotation_period < pdMS_TO_TICKS(50)) { // Muy rápido
        increment = 50;
    } else if (rotation_period < pdMS_TO_TICKS(100)) { // Rápido
        increment = 10;
    } else { // Lento (o el primer movimiento)
        increment = 1;
    }

    // Retornamos el incremento multiplicado por la dirección
    return increment * change;
}

static uint32_t rtc_get_asbolute_seconds(ds1307_t *rtc){
    return rtc->time.seconds + rtc->time.minutes * 60 + rtc->time.hours * 3600;
}

static uint32_t rtc_get_absolute_diff_seconds(uint32_t from, uint32_t to){
    return to - from;
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
    user.sp_0 = user.sp;
    int set_point = user.sp;
    
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

    ds1307_t rtc, now_rtc;
    log_t usr_log;
    user_view.change_value_mode = true;

    uint8_t prev_set = 0;

    uint32_t rtc_last_time = rtc_get_asbolute_seconds(&rtc);
    uint32_t e=0;

    bool config_has_changed = false, params_has_changed = false;

    for(;;){
        set_user_event_count = uxSemaphoreGetCount(set_user_event); // Tomo la cantidad de veces quese presiono el boton del encoder

        xTaskNotifyWait(0,0, &e, 0);

        if(e == 1){
            user_view = user;
            e = 0;
        }

        if(xQueueReceive(encoder_event, &encoder_increment, 0)); // En caso de rotar el encoder recibo el valor en ticks de la rotacion

        user_view.menu = uxSemaphoreGetCount(change_event) % 3; // Tomo la cantidad de veces que se presiono el boton de cambio de menu
        user.menu = user_view.menu;

        xQueuePeek(q_rtc, &rtc, portMAX_DELAY);

        // if(user.menu == config_menu){ // No es la manera mas eficiente ya que el reloj no cambia en menos de un segundo pero es la mas simple
        user.hour = rtc.time.hours;
        user.minute = rtc.time.minutes;
        user.sencond = rtc.time.seconds;
        user.year = rtc.time.year;
        user.month = rtc.time.month;
        user.day = rtc.time.date;    
        // }
        
        if(rtc_get_absolute_diff_seconds(rtc_last_time, rtc_get_asbolute_seconds(&rtc))>= (5*60)){ // Se registra un log cada 5 minutos
            rtc_last_time = rtc_get_asbolute_seconds(&rtc);
            usr_log.time = rtc.time;
            usr_log.lux = lux;
            xQueueSend(q_to_storage, &usr_log, portMAX_DELAY);
            // printf("log! %lu, %lu\n", rtc_get_absolute_diff_seconds(last_time, rtc_get_asbolute_seconds(&rtc)), rtc_get_asbolute_seconds(&rtc));
        }

        if(set_user_event_count%2 == 1){ // Entra en modo cambio de parametros
            if(prev_set != set_user_event_count){
                user_view = user;
            }
            motion = true;
            user_view.change_value_mode = true;

            if(user_view.menu == params_menu){
                params_has_changed = true;
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

            if(user_view.menu == config_menu){
                ui.p_user = &user_view;
                config_has_changed = true;
                now_rtc = rtc;

                switch (user_view.select)
                {
                    case hour_config:
                        if(user_view.hour + encoder_increment >0)
                            user_view.hour += encoder_increment;
                        user_view.hour %= 24;
                        user_view.minute = user.minute;
                        user_view.sencond = user.sencond;
                        now_rtc.time.hours = user_view.hour;
                    break;
                    case second_config:
                        if(user_view.sencond + encoder_increment >0)
                            user_view.sencond += encoder_increment;
                        user_view.sencond %= 60;
                        user_view.hour = user.hour;
                        user_view.minute = user.minute;
                        now_rtc.time.seconds = user_view.sencond;
                    break;
                    case minute_config:
                        if(user_view.minute + encoder_increment >0)
                            user_view.minute += encoder_increment;
                        user_view.minute %= 60;
                        user_view.hour = user.hour;
                        user_view.sencond = user.sencond;
                        now_rtc.time.minutes = user_view.minute;
                    break;
                    case day_config:
                        if(user_view.day + encoder_increment > 0)
                            user_view.day += encoder_increment;
                        user_view.day %= 32;
                        now_rtc.time.date = user_view.day;
                    break;
                    case month_config:
                        if(user_view.month + encoder_increment > 0)
                            user_view.month += encoder_increment;
                        user_view.month %= 13;
                        now_rtc.time.month = user_view.month;
                    break;
                    case year_config:
                        if(user_view.year + encoder_increment > 0)
                            user_view.year += encoder_increment;
                        now_rtc.time.year = user_view.year;
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

            if(user.menu == log_menu){
                switch (user.select)
                {
                case 1:
                    xSemaphoreGive(read_logs_event);
                    break;
                default:
                    xSemaphoreGive(erase_logs_event);
                    break;
                }
            }

            encoder_increment = 0;

            prev_set = set_user_event_count;
        }else{
            motion = false;
            user_view.change_value_mode = false;
            user.sp = user_view.sp_0;

            if(config_has_changed){
                // printf("NOW_RTC: %02d;%02d;%02d-%02d/%02d/%02d\n", now_rtc.time.hours, now_rtc.time.minutes, now_rtc.time.seconds, now_rtc.time.date, now_rtc.time.month, now_rtc.time.year);
                // printf("RTC: %02d;%02d;%02d-%02d/%02d/%02d\n", rtc.time.hours, rtc.time.minutes, rtc.time.seconds, rtc.time.date, rtc.time.month, rtc.time.year);
                // printf("USER: %02d;%02d;%02d-%02d/%02d/%02d\n", user.hour, user.minute, user.sencond, user.day, user.month, user.year);
                xQueueSend(q_rtc_config, &now_rtc, portMAX_DELAY);
                config_has_changed = false;
            }
            if(params_has_changed){
                // printf("Sys: Params cambio\n");
                // xQueueSend(q_user_config, &user, portMAX_DELAY); // Copio el usuario en la cola para no tener problemas de concurrencia

                params_has_changed = false; 
            }

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
            xQueueSend(q_user_config, &user, portMAX_DELAY);
            xQueueReset(set_user_event);
        }

        set_point = user.sp;
        set_point_0 = user.sp_0;
        set_point_final = user.sp_f;
                    
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
    const TickType_t debounce_time = pdMS_TO_TICKS(100); // Tiempo de debounce (10ms)

    // Variables para calcular velocidad de rotación
    TickType_t last_edge_time = xTaskGetTickCount();
    TickType_t current_edge_time;
    TickType_t rotation_period;

    int32_t increment = 10; // Valor base de incremento
    int user_increment = 0; // Variable para almacenar el set point del usuario
    int user_select = set_sp;

    int count = 0;

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
        // encoder.prev_clk = encoder.clk; // Guardo el estado anterior del clk
        // encoder.prev_dt = encoder.dt;

        if(!gpio_get(PIN_BTN)){
            xSemaphoreGive(change_event);
            printf("change %d\n");
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if(!gpio_get(PIN_BTN_CTRL)){
            xSemaphoreGive(toggle_control_event);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if(!encoder.sw) { // Si el boton esta presionado
            xSemaphoreGive(set_user_event);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#define FILE_SIZE 64*1024
#define BUF_WRDS (1024 / sizeof(uint32_t))

typedef struct {
    float kp;
    float ki;
    float kd;
    
    uint16_t sp_0;
    uint16_t sp_f;
    uint16_t rise_time_ms;
    uint16_t min;
    uint16_t max;
    
} user_storage_t;

static uint32_t buf[BUF_WRDS];
static const char usr_config_file[] = "user.cfg";
static const char log_file[] = "lux.log";
static int16_t to_save_buff[8];
static uint8_t to_read_buff[2*8];
static int16_t log_to_save[7];
static uint8_t log_to_read[2*7];
user_storage_t user_to_storage;

static void inline save_usr_params(user_t *f_user, lfs_t *lfs, lfs_file_t *file){

    char msg[128];

    int err = lfs_file_open(lfs, file, usr_config_file, LFS_O_RDWR | LFS_O_CREAT);

    control_params_t pid;

    if(err != LFS_ERR_OK){
        lfs_unmount(lfs);
        panic("failed to open file");
    }     

    sys_print("Guardado\n");

    xQueuePeek(q_control_params, &pid, portMAX_DELAY);

    user_to_storage.sp_0 = f_user->sp_0;

    user_to_storage.sp_f = f_user->sp_f;

    user_to_storage.rise_time_ms = f_user->rise_time_ms;

    user_to_storage.min = f_user->min;

    user_to_storage.max = f_user->max;

    user_to_storage.kp = pid.kp;

    user_to_storage.ki = pid.ki;

    user_to_storage.kd = pid.kd;

    err = lfs_file_write(lfs, file, &user_to_storage, sizeof(user_to_storage));
    
    if( err < LFS_ERR_OK){
        lfs_file_close(lfs, file);
        lfs_unmount(lfs);
        panic("failed to write file");
    }

    #if DEBUG_FLASH
    printf("Bytes escritos %d\n", err);
    #endif

    lfs_file_close(lfs, file);
}

static inline void read_usr_params(lfs_t *lfs, lfs_file_t *file){
    
    int err = lfs_file_open(lfs, file, usr_config_file, LFS_O_RDWR | LFS_O_CREAT);

    if(err != LFS_ERR_OK){
        lfs_unmount(lfs);
        panic("failed to open file");
    }     

    err = lfs_file_read(lfs, file, &user_to_storage, sizeof(user_to_storage));

    sys_print("Leido\n");

    control_params_t pid;
    char msg[128];
    vTaskSuspend(user_task_handler);
    user.sp_0 = user_to_storage.sp_0;
    user.sp = user_to_storage.sp_0;
    user.sp_f = user_to_storage.sp_f;
    user.rise_time_ms = user_to_storage.rise_time_ms;
    user.min = user_to_storage.min;
    user.max = user_to_storage.max;
    sprintf(msg, "setPoint: %d, setPointFinal: %d, riseTime: %d, min: %d, max: %d\n", user.sp_0, user.sp_f, user.rise_time_ms, user.min, user.max);
    sys_print(msg);
    vTaskResume(user_task_handler);
    pid.kp = user_to_storage.kp;
    pid.ki = user_to_storage.ki;
    pid.kd = user_to_storage.kd;

    xQueueSend(q_control, &user.sp, portMAX_DELAY);

    sprintf(msg, "kp: %f, ki: %f, kd: %f\n", pid.kp, pid.ki, pid.kd);
    sys_print(msg);

    xQueueOverwrite(q_control_params, &pid);

    if(err < LFS_ERR_OK){
        lfs_file_close(lfs, file);
        lfs_unmount(lfs);
        panic("failed to open file");
    } 

    #if DEBUG_FLASH
    printf("Bytes leidos %d\n", err);
    printf("Datos leidos: %d %d %d %d %d\n", to_save_buff[0], to_save_buff[1], to_save_buff[2], to_save_buff[3], to_save_buff[4]);
    #endif

    lfs_file_close(lfs, file);
}

static inline void save_log(log_t *log, lfs_t *lfs, lfs_file_t *file){
    int err = lfs_file_open(lfs, file, log_file, LFS_O_RDWR | LFS_O_APPEND);

    if(err != LFS_ERR_OK){
        err = lfs_file_open(lfs, file, log_file, LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);
    }

    if(err != LFS_ERR_OK){
        lfs_unmount(lfs);
        panic("failed to open file");
    }     

    // err = lfs_file_write(lfs, file, log, sizeof(log));
    // err = lfs_file_write(lfs, file, log, sizeof(log));
    log_to_save[0] = log->lux;
    log_to_save[1] = log->time.hours;
    log_to_save[2] = log->time.minutes;
    log_to_save[3] = log->time.seconds;
    log_to_save[4] = log->time.date;
    log_to_save[5] = log->time.month;
    log_to_save[6] = log->time.year;

    err = lfs_file_write(lfs, file, log_to_save, sizeof(log_to_save));
    
    if( err < LFS_ERR_OK){
        lfs_file_close(lfs, file);
        lfs_unmount(lfs);
        panic("failed to write file");
    }

    #if DEBUG_FLASH
    printf("Bytes escritos %d\n", err);
    #endif

    lfs_file_close(lfs, file);
}

ssize_t read_next_log(lfs_t *lfs_handle, lfs_file_t *file_handle, log_t *output_log) {
    // Lectura del tamaño exacto de la estructura
    ssize_t bytes_read = lfs_file_read(lfs_handle, file_handle, log_to_read, sizeof(log_to_save));

    output_log->lux = log_to_read[0] | (log_buffer[2] << 8);
    output_log->time.hours = log_to_read[2] | (log_buffer[3] << 8);
    output_log->time.minutes = log_to_read[4] | (log_buffer[5] << 8);
    output_log->time.seconds = log_to_read[6] | (log_buffer[7] << 8);
    output_log->time.date = log_to_read[8] | (log_buffer[9] << 8);
    output_log->time.month = log_to_read[10] | (log_buffer[11] << 8);
    output_log->time.year = log_to_read[12] | (log_buffer[13] << 8);

    // Si bytes_read es 0, hemos llegado al final del archivo (EOF).
    // Si bytes_read es < 0, es un error.
    // Si bytes_read > 0 y != sizeof(log_t), es un error de alineación o truncamiento.

    return bytes_read;
}

static inline void read_all_logs(lfs_t *lfs, lfs_file_t *file){
    log_t current_log;
    
    // 1. Abrir el archivo en modo de sólo lectura
    int err = lfs_file_open(lfs, file, log_file, LFS_O_RDONLY);

    if (err != LFS_ERR_OK) {
        printf("No se pudo abrir el archivo de log para leer.\n");
        return;
    }

    printf("Iniciando la lectura de logs...\n");

    ssize_t bytes_read;
    int log_count = 0;

    // 2. Leer secuencialmente hasta el final del archivo (EOF)
    while (true) {
        bytes_read = read_next_log(lfs, file, &current_log);
        
        if (bytes_read == sizeof(log_to_save)) {
            // Log leído exitosamente
            log_count++;
            
            // 3. Presentar los datos al usuario (por UART, pantalla, etc.)
            printf("Log: id:%d, %u, %u/%u/%u-%u;%u;%u\n", 
                   log_count, 
                   current_log.lux, 
                   current_log.time.date, current_log.time.month, current_log.time.year,
                   current_log.time.hours, current_log.time.minutes, current_log.time.seconds);

        } else if (bytes_read < 0) {
            // Manejo de errores de LFS (ej. LFS_ERR_IO)
            printf("ERROR de LFS durante la lectura: %ld\n", bytes_read);
            break;
        } else if (bytes_read == 0) {
            // Final del archivo (EOF) alcanzado
            printf("Fin de los logs. Total de logs leídos: %d\n", log_count);
            break;
        }
    }

    // 4. Cerrar el archivo
    lfs_file_close(lfs, file);
}

static inline void read_logs(lfs_t *lfs, lfs_file_t *file, int last_cant){
    log_t current_log;
    char msg[64];
    
    int err = lfs_file_open(lfs, file, log_file, LFS_O_RDONLY);

    if (err != LFS_ERR_OK) {
        printf("No se pudo abrir el archivo de log para leer.\n");
        return;
    }
    
    int file_size = lfs_file_size(lfs, file);
    int log_cant = file_size / sizeof(log_to_save);

    // printf("Tamaño de archivo: %d\n", file_size);

    if (last_cant > log_cant) {
        last_cant = log_cant;
    }
    
    // Si log_cant=161 y last_cant=5, 'from' debe ser 156.
    int from = log_cant - last_cant; 
    int to = log_cant;
    
    // printf("Iniciando la lectura de logs...\n");

    // Desplazo el puntero hacia el log "from"
    err = lfs_file_seek(lfs, file,  from * sizeof(log_to_save), LFS_SEEK_SET);

    if(err<0){
        printf("Log no existente!\n");
        return;
    }

    ssize_t bytes_read;
    int log_count = from;

    for(int i=0; i<=last_cant; i++) {
        bytes_read = read_next_log(lfs, file, &current_log);
        
        if (bytes_read == sizeof(log_to_save)) {
            // Log leído exitosamente
            log_count++;
            
            sprintf(msg,"log:%d, lux:%u, time:%u/%u/%u-%u;%u;%u\n", 
                   log_count, 
                   current_log.lux, 
                   current_log.time.date, current_log.time.month, current_log.time.year,
                   current_log.time.hours, current_log.time.minutes, current_log.time.seconds);

            sys_print(msg);

        } else if (bytes_read < 0) {
            // Manejo de errores de LFS (ej. LFS_ERR_IO)
            sprintf(msg,"ERROR de LFS durante la lectura: %ld\n", bytes_read);
            sys_print(msg);
            break;
        } else if (bytes_read == 0) {
            // Final del archivo (EOF) alcanzado
            // printf("Fin de los logs. Total de logs leídos: %d\n", log_count - from);
            break;
        }
    }

    // 4. Cerrar el archivo
    lfs_file_close(lfs, file);
}

#define FS_SIZE 64*1024

/**
 * @brief Tarea encargada de manejar la memoria flash
 * 
 * @param params 
 */
void storage_task(void *params) {
    char msg[128];
    log_t log;
    user_t user_config;
    flash_err_t r;
    lfs_t lfs;
    struct lfs_config *lfs_cfg;
    lfs_file_t file;
    control_params_t pid;

    /* Near the beginning of your program initialize LFS */

    lfs_cfg = pico_lfs_init(PICO_FLASH_SIZE_BYTES - FS_SIZE, FS_SIZE);
    if (!lfs_cfg) panic("MEM: out of memory");

    /* Format new FS if needed */

    int err = lfs_mount(&lfs, lfs_cfg);
    if (err != LFS_ERR_OK) {
      /* Initialize new filesystem */
      err = lfs_format(&lfs, lfs_cfg);
      if (err != LFS_ERR_OK) panic("MEM: failed to format filesystem");
      err = lfs_mount(&lfs, lfs_cfg);
      if (err != LFS_ERR_OK) panic("MEM: failed to mount new filesystem");
    }
    
    // err = lfs_remove(&lfs, usr_config_file);
    // err = lfs_remove(&lfs, log_file);

    #if FLASH_RESET
    #endif

    // printf("MEM: Ready\n");
    uint32_t e;
    int log_cant;
    int log_count = 0;

    read_usr_params(&lfs, &file);

    while(true){
        if(xQueueReceive(q_to_storage, &log, 0)){
            // sprintf(msg,"log:%d, lux:%u, time:%u/%u/%u-%u;%u;%u\n", 
            //        log_count, 
            //        log.lux, 
            //        log.time.date, log.time.month, log.time.year,
            //        log.time.hours, log.time.minutes, log.time.seconds);
            // sys_print(msg);
            save_log(&log, &lfs, &file);

            log_count++;

            // read_all_logs(&lfs, &file);
            #ifdef DEBUG_LOGS
            #endif
        }
        if(xSemaphoreTake(read_logs_event, 0) || xTaskNotifyWait(0, 0, &e,0)){
            if(e == 0){
                read_logs(&lfs, &file, 20);
            }
            read_logs(&lfs, &file, e);
            e = 0;
            // read_all_logs(&lfs, &file);
        }
        if(xQueueReceive(q_user_config, &user_config, 0)){
            #if DEBUG_FLASH
            #endif
            sprintf(msg,"setPoint: %d, setPointFinal: %d, riseTime: %d, min: %d, max: %d\n", user_config.sp_0, user_config.sp_f, user_config.rise_time_ms, user_config.min, user_config.max);
            sys_print(msg);
            xQueuePeek(q_control_params, &pid, portMAX_DELAY);
            sprintf(msg, "kp: %.2f, ki: %.2f, kd: %.2f\n", pid.kp, pid.ki, pid.kd);
            sys_print(msg);
            save_usr_params(&user_config, &lfs, &file);

            #if DEBUG_FLASH
            printf("USER READ: %d %d %d %d %d\n", user_config.sp, user_config.sp_f, user_config.rise_time_ms, user.min, user.max);
            #endif

        }
        // if(xSemaphoreTake(erase_logs_event, 0)){
        //     erase_all_logs();
        //     save_u16_as_bytes((uint16_t)500, 0);
        //     save_u16_as_bytes((uint16_t)0, 1);
        //     save_u16_as_bytes((uint16_t)1000, 2);
        // }
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

    gpio_init(PIN_BTN_CTRL);
    gpio_set_dir(PIN_BTN_CTRL, GPIO_IN);
    gpio_pull_up(PIN_BTN_CTRL);

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
    // pwm_config_set_clkdiv(&config, clk_sys/PWM_FREQ);

    pwm_init(slice_num, &config, true);

    pwm_set_wrap(slice_num, PWM_WRAP);

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
    q_rtc_config = xQueueCreate(1, sizeof(ds1307_t));

    q_to_storage = xQueueCreate(10, sizeof(log_t));
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
    q_to_print = xQueueCreate(5, sizeof(char)*RESPONSE_MAX_LEN);
    q_buff_to_print = xQueueCreate(1, sizeof(buff_to_print_t));

    set_user_event = xSemaphoreCreateCounting(2,0);
    change_event = xSemaphoreCreateCounting(100,0);
    encoder_event = xQueueCreate(2, sizeof(int));
    toggle_control_event = xSemaphoreCreateBinary();
    read_logs_event = xSemaphoreCreateBinary();
    erase_logs_event = xSemaphoreCreateBinary();

    adc_config();
    i2c_config();
    bh1750_init();
    gpio_config();
    uart_init(uart0, 115200);
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

    #if ERASE_FLASH
        flash_err_t err = erase_pages(0, MAX_PAGES);
        if(err != FLASH_OK) printf("Error en borrado de flash\n");
        else printf("Flash borrada con exito\n");
    #endif

    
    // read_state(&user);

    // for(int i = 0; i < 3; i++) {
        //     values[i] = read_log_u16(i);
        // }

        // user.sp = values[0];
        // user.rise_time_ms = values[1];
        // user.sp_f = values[2];
    printf("Iniciando sistema ....\n");
        
    // pico_mount(true);
    // struct pico_fsstat_t stat;
    // pico_fsstat(&stat);
    // printf("FS: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size,(int)stat.blocks_used);
    
    #if SAFE_STATE_SAVE
        // save_state(&user);
        save_usr_params(&user);
    #endif
    // read_usr_params(&user);
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

    xTaskCreate(
        storage_task,
        "storage_task",
        configMINIMAL_STACK_SIZE*6,
        NULL,
        tskIDLE_PRIORITY+4,
        &storage_task_handler
    );

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
        configMINIMAL_STACK_SIZE*4,
        NULL,
        tskIDLE_PRIORITY + 3,
        NULL
    );

    xTaskCreate(
        cli_task,
        "cli_task",
        configMINIMAL_STACK_SIZE*4,
        NULL,
        tskIDLE_PRIORITY + 3,
        NULL
    );

    vTaskStartScheduler();

    while (1) {
    }
}
