#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/mcpwm.h"
#include "esp_log.h"

static const char *TAG = "esp-car";

#define PCNT_HIGH_LIMIT         1024
#define PCNT_LOW_LIMIT         -1
#define PCNT_THRESH1_VAL        5
#define PCNT_THRESH0_VAL       -5

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

#define WHEEL_RADIUS            214 / 1000.0
#define SENSOR_RESOLUTION       40

#define MS_TO_TICKS(ms)         (ms / portTICK_PERIOD_MS)
#define TICKS_TO_MS(ticks)      (ticks * portTICK_PERIOD_MS)

struct motor_t {
    uint8_t pin1;
    uint8_t pin2;
    mcpwm_unit_t mcpwm_unit;
    mcpwm_timer_t mcpwm_timer;
    mcpwm_io_signals_t io_signal1;
    mcpwm_io_signals_t io_signal2;
    uint8_t pin_sensor;
    pcnt_unit_handle_t pcnt_unit;
    TickType_t lastCalcTicks;
    float speed;
};

static void motor_sensor_init(struct motor_t *motor)
{
    ESP_LOGI(TAG, "initialize pcnt unit %d", motor->pin_sensor);

    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &motor->pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(motor->pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = motor->pin_sensor,
        .level_gpio_num = -1,
    };

    ESP_LOGI(TAG, "initialize pcnt channel %d", motor->pin_sensor);
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(motor->pcnt_unit, &chan_a_config, &pcnt_chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));

    ESP_LOGI(TAG, "enable, clear, and start pcnt unit %d", motor->pin_sensor);
    ESP_ERROR_CHECK(pcnt_unit_enable(motor->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(motor->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(motor->pcnt_unit));
}

static void motor_sensor_calc(struct motor_t *motor) {
  TickType_t tickDiff = xTaskGetTickCount() - motor->lastCalcTicks;
  motor->lastCalcTicks += tickDiff;

  int count = 0;
  ESP_ERROR_CHECK(pcnt_unit_get_count(motor->pcnt_unit, &count));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(motor->pcnt_unit));

  motor->speed = WHEEL_RADIUS / SENSOR_RESOLUTION * count / (TICKS_TO_MS(tickDiff) / 1000.0);
}

static void mcpwm_motor_init(struct motor_t *motor) {
    ESP_LOGI(TAG, "initialize mcpwm unit %d, signal1 %d, signal2 %d", motor->mcpwm_unit, motor->io_signal1, motor->io_signal2);
    ESP_ERROR_CHECK(mcpwm_gpio_init(motor->mcpwm_unit, motor->io_signal1, motor->pin1));
    ESP_ERROR_CHECK(mcpwm_gpio_init(motor->mcpwm_unit, motor->io_signal2, motor->pin2));

    mcpwm_config_t pwm_config = {
        .frequency = 25,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    ESP_LOGI(TAG, "initialize mcpwm unit %d, timer %d", motor->mcpwm_unit, motor->mcpwm_timer);
    ESP_ERROR_CHECK(mcpwm_init(motor->mcpwm_unit, motor->mcpwm_timer, &pwm_config));
}

void test_loop(struct motor_t motors[4]) {
    while (1) {
        vTaskDelay(MS_TO_TICKS(500));

        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            motor_sensor_calc(&motors[i]);
        }

        mcpwm_set_duty(motors[0].mcpwm_unit, motors[0].mcpwm_timer, MCPWM_OPR_A, 20.0);
        mcpwm_set_duty(motors[1].mcpwm_unit, motors[1].mcpwm_timer, MCPWM_OPR_A, 20.0);
        mcpwm_set_duty(motors[2].mcpwm_unit, motors[2].mcpwm_timer, MCPWM_OPR_A, 20.0);
        mcpwm_set_duty(motors[3].mcpwm_unit, motors[3].mcpwm_timer, MCPWM_OPR_A, 20.0);

        ESP_LOGI(TAG, "motor speeds: %f, %f, %f, %f", motors[0].speed, motors[1].speed, motors[2].speed, motors[3].speed);
    }
}

void test_speed_loop(struct motor_t motors[4]) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        struct motor_t *motor = &motors[i];
        float duty = 0.0;

        while (duty < 100.0) {
            float results[100] = {};
            
            mcpwm_set_duty(motor->mcpwm_unit, motor->mcpwm_timer, MCPWM_OPR_A, duty);

            vTaskDelay(MS_TO_TICKS(500));
            
            for (uint8_t j = 0; j < 100; j++) {
                vTaskDelay(MS_TO_TICKS(250));

                motor_sensor_calc(motor);
                results[j] = motor->speed;
            }

            float sum = 0.0;
            for (uint8_t j = 0; j < 100; j++) {
                sum += results[j];
            }
            ESP_LOGI(TAG, "%d,%f,%f", i, duty, sum/100.0);

            duty += 0.5;
        }
    } 
}

void app_main()
{
    struct motor_t motors[MOTOR_COUNT] = {
        {MOTOR1_PIN1, MOTOR1_PIN2, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, SENSOR_PIN1, NULL, 0, 0.0},
        {MOTOR2_PIN1, MOTOR2_PIN2, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B, SENSOR_PIN2, NULL, 0, 0.0},
        {MOTOR3_PIN1, MOTOR3_PIN2, MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM2A, MCPWM2B, SENSOR_PIN3, NULL, 0, 0.0},
        {MOTOR4_PIN1, MOTOR4_PIN2, MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, SENSOR_PIN4, NULL, 0, 0.0}
    };

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        mcpwm_motor_init(&motors[i]);
        motor_sensor_init(&motors[i]);
    }

    test_speed_loop(motors);
}
