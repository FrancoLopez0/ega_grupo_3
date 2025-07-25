#include "ssd1306/ssd1306.h"
#include "stdio.h"
#include "string.h"

#define MAX_SP          600
#define MAX_LUX         1500
#define OLED_ADDR       0x3C

#define DEFAULT_BAR {.x = 0,.y = 7,.h = 10,.w = 128,.percent = 0}

typedef struct{
    uint8_t x;
    uint8_t y;
    uint8_t w;
    uint8_t h;
    uint8_t percent;
}bar_t;

typedef struct{
    uint32_t sp;
    uint32_t sp_f;
    uint32_t rise_time_ms;
    bool mode;
    uint8_t select;
    uint32_t lux;
}user_t;

enum select_enum{
    set_lux,
    set_sp,
    set_time,
    set_sp_f,
    not_show
};

/**
 * @brief Inicializa la interfaz de usuario
 * 
 * @param p_oled 
 * @param i2c 
 * @param p_user 
 */
void ui_init(ssd1306_t *p_oled, i2c_inst_t *i2c, user_t *p_user);

/**
 * @brief Dibuja los parametros
 * 
 * @param p_oled 
 * @param p_user 
 * @param lux 
 */
void ui_update_params(ssd1306_t *p_oled, user_t *p_user);

/**
 * @brief Actualiza la interfaz de usuario
 * 
 * @param p_oled 
 * @param p_user 
 */
void ui_update(ssd1306_t *p_oled, user_t *p_user);

/**
 * @brief Dibuja la barra con el set point
 * 
 * @param p_oled 
 * @param p_bar 
 * @param sp 
 */
void ui_update_user_sp(ssd1306_t *p_oled, bar_t *p_bar, uint32_t sp);

/**
 * @brief Dibuja la barra indicadora
 * 
 * @param p_oled 
 * @param p_bar 
 * @param p_user 
 */
void ui_update_bar(ssd1306_t *p_oled, bar_t *p_bar, user_t *p_user);