#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

#define MS_TO_TICKS(ms)         (ms / portTICK_PERIOD_MS)
#define TICKS_TO_MS(ticks)      (ticks * portTICK_PERIOD_MS)

#define PCNT_HIGH_LIMIT         1024
#define PCNT_LOW_LIMIT         -1
#define PCNT_THRESH1_VAL        5
#define PCNT_THRESH0_VAL       -5

#define PWM_FREQUENCY           25
#define PWM_MIN_DUTY            10
#define PWM_MAX_DUTY            100

#define MOTOR_COUNT             4

#define SENSOR_PIN1             35
#define SENSOR_PIN2             34
#define SENSOR_PIN3             39
#define SENSOR_PIN4             36

#define MOTOR1_PIN1             21
#define MOTOR1_PIN2             19
#define MOTOR2_PIN1             32
#define MOTOR2_PIN2             33
#define MOTOR3_PIN1             23
#define MOTOR3_PIN2             22
#define MOTOR4_PIN1             25
#define MOTOR4_PIN2             26

#define MOTOR_CURVE_COEF1       7.064
#define MOTOR_CURVE_COEF2       60.113
#define MOTOR_CURVE_COEF3      -403.601
#define MOTOR_CURVE_COEF4       2741.545
#define MOTOR_CURVE_COEF5      -7267.514
#define MOTOR_CURVE_COEF6       9074.591
#define MOTOR_CURVE_SIZE        6

#define MOTOR1_CORR             898
#define MOTOR2_CORR             945
#define MOTOR3_CORR             879
#define MOTOR4_CORR             1000

#define WHEEL_RADIUS            214 / 1000.0
#define SENSOR_RESOLUTION       40

#define GET_MOTOR_COEF(i)       i == 0 ? MOTOR_CURVE_COEF1 : (i == 1 ? MOTOR_CURVE_COEF2 : (i == 2 ? MOTOR_CURVE_COEF3 : (i == 3 ? MOTOR_CURVE_COEF4 : (i == 4 ? MOTOR_CURVE_COEF5 : MOTOR_CURVE_COEF6))))

static const char *TAG = "esp-car motor";

struct motor_t {
    uint8_t pin1;
    uint8_t pin2;
    uint16_t correction;
    mcpwm_unit_t mcpwm_unit;
    mcpwm_timer_t mcpwm_timer;
    mcpwm_io_signals_t io_signal1;
    mcpwm_io_signals_t io_signal2;
    uint8_t pin_sensor;
    pcnt_unit_handle_t pcnt_unit;
    TickType_t lastCalcTicks;
    float speed;
};

void mcpwm_motor_init(struct motor_t *motor);
void motor_sensor_init(struct motor_t *motor);
void motor_sensor_calc(struct motor_t *motor);
void motor_speed_set(struct motor_t *motor, float speed);
