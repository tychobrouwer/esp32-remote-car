#include "motor.h"

void mcpwm_motor_init(struct motor_t *motor) {
    ESP_LOGI(TAG, "initialize mcpwm unit %d, signal1 %d, signal2 %d", motor->mcpwm_unit, motor->io_signal1, motor->io_signal2);
    ESP_ERROR_CHECK(mcpwm_gpio_init(motor->mcpwm_unit, motor->io_signal1, motor->pin1));
    ESP_ERROR_CHECK(mcpwm_gpio_init(motor->mcpwm_unit, motor->io_signal2, motor->pin2));

    mcpwm_config_t pwm_config = {
        .frequency = PWM_FREQUENCY,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };

    mcpwm_set_duty(motor->mcpwm_unit, motor->mcpwm_timer, MCPWM_OPR_A, 0.0);
    mcpwm_set_duty(motor->mcpwm_unit, motor->mcpwm_timer, MCPWM_OPR_B, 0.2);

    ESP_LOGI(TAG, "initialize mcpwm unit %d, timer %d", motor->mcpwm_unit, motor->mcpwm_timer);
    ESP_ERROR_CHECK(mcpwm_init(motor->mcpwm_unit, motor->mcpwm_timer, &pwm_config));
}

void motor_sensor_init(struct motor_t *motor)
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

void motor_sensor_calc(struct motor_t *motor) {
    TickType_t tickDiff = xTaskGetTickCount() - motor->lastCalcTicks;
    motor->lastCalcTicks += tickDiff;

    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(motor->pcnt_unit, &count));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(motor->pcnt_unit));

    motor->speed = WHEEL_RADIUS / SENSOR_RESOLUTION * count / (TICKS_TO_MS(tickDiff) / 1000.0);
}

void motor_speed_set(struct motor_t *motor, float speed) {
    speed = speed * motor->correction / 1000.0;
    float duty = 0.0;

    for (uint8_t i = 0; i < MOTOR_CURVE_SIZE; i++) {
        duty += GET_MOTOR_COEF(i) * pow(speed, i);
    }

    duty = duty < PWM_MIN_DUTY ? PWM_MIN_DUTY : (duty > PWM_MAX_DUTY ? PWM_MAX_DUTY : duty);
    mcpwm_set_duty(motor->mcpwm_unit, motor->mcpwm_timer, MCPWM_OPR_A, duty);
}
