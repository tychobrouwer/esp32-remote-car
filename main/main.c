#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define MS_TO_TICKS(ms)         (ms / portTICK_PERIOD_MS)
#define TICKS_TO_MS(ticks)      (ticks * portTICK_PERIOD_MS)

#include "motor.h"

void test_loop(struct motor_t motors[4]) {
    while (1) {
        vTaskDelay(MS_TO_TICKS(500));

        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            motor_sensor_calc(&motors[i]);

            mcpwm_set_duty(motors[i].mcpwm_unit, motors[i].mcpwm_timer, MCPWM_OPR_A, 20.0);
        }

        ESP_LOGI(TAG, "motor speeds: %f, %f, %f, %f", motors[0].speed, motors[1].speed, motors[2].speed, motors[3].speed);
    }
}

void test_speed_curve_loop(struct motor_t motors[4]) {
    float duty = 0.0;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        mcpwm_set_duty(motors[i].mcpwm_unit, motors[i].mcpwm_timer, MCPWM_OPR_A, 0.0);
    }

    while (duty <= 100.0) {
        float sum[MOTOR_COUNT] = {0.0};

        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            mcpwm_set_duty(motors[i].mcpwm_unit, motors[i].mcpwm_timer, MCPWM_OPR_A, duty);
        }

        vTaskDelay(MS_TO_TICKS(500));

        for (uint8_t j = 0; j < 100; j++) {
            vTaskDelay(MS_TO_TICKS(250));

            for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
                motor_sensor_calc(&motors[i]);
                sum[i] += motors[i].speed;
            }
        }

        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            ESP_LOGI(TAG, "%d,%f,%f", i, duty, sum[i]/100.0);
        }

        duty += 0.5;
    }
}

void test_speed_loop(struct motor_t motors[4]) {
    float speed = 0.0;

    while (speed <= 0.4) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            motor_speed_set(&motors[1] , speed);
        }
        
        vTaskDelay(MS_TO_TICKS(1000));

        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            motor_sensor_calc(&motors[1]);
        }

        ESP_LOGI(TAG, "%f - motor speeds: %f,%f,%f,%f", speed, motors[0].speed, motors[1].speed, motors[2].speed, motors[3].speed);

        speed += 0.01;
    }
}

void app_main()
{
    struct motor_t motors[MOTOR_COUNT] = {
        {MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_CORR, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, SENSOR_PIN1, NULL, 0, 0.0},
        {MOTOR2_PIN1, MOTOR2_PIN2, MOTOR2_CORR, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B, SENSOR_PIN2, NULL, 0, 0.0},
        {MOTOR3_PIN1, MOTOR3_PIN2, MOTOR3_CORR, MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM2A, MCPWM2B, SENSOR_PIN3, NULL, 0, 0.0},
        {MOTOR4_PIN1, MOTOR4_PIN2, MOTOR4_CORR, MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, SENSOR_PIN4, NULL, 0, 0.0}
    };

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        mcpwm_motor_init(&motors[i]);
        motor_sensor_init(&motors[i]);
    }

    vTaskDelay(MS_TO_TICKS(1000));

    test_speed_loop(motors);
}
