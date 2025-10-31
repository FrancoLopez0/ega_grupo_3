#define CLK 19
#define DT 18
#define SW 20

#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17

#define PIN_TEMT6000 28

#define PIN_PWM 1

#define MAX_SET_POINT 1500
#define MIN_SET_POINT 100
#define MAX_RISE_TIME 10000 // 10 segundos
#define PWM_WRAP      4095

#define PIN_LED_RED 2

#define PIN_LED_GREEN 4

#define PIN_BTN 21

#define DISPLAY_OLED 1

#define HARD_CODE 0

#define LUX_CALIBRATION 0

#define PRINT_CONTROL 0
#define SET_POINT 1000.f
#define PRINT_SIZE 500

#define PIN_RX    13 
#define PIN_TX    12

#define PRINT_RAW_DATA 0

//==========================CONTROL=====================================

#define KP 14.f
#define KI 5.f//0.60f
#define KD 0.0f//0.006f

//===========================DEBUG======================================

// #define PRINT_VALUES_MODE
// #define DEBUG_CONTROL
// #define DEBUG_LEDS
// #define DEBUG_GET_LUX
// #define DEBUG_I2C
// #define DEBUG_LOGS