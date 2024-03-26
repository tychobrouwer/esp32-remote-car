#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

static const char *TAG = "esp-car";

#define PCNT_HIGH_LIMIT         1024
#define PCNT_LOW_LIMIT         -1
#define PCNT_THRESH1_VAL        5
#define PCNT_THRESH0_VAL       -5

#define MOTOR_COUNT             4

#define SENSOR_PIN1             34
#define SENSOR_PIN2             35
#define SENSOR_PIN3             36
#define SENSOR_PIN4             39

#define MOTOR1_PIN1             19
#define MOTOR1_PIN2             21
#define MOTOR2_PIN1             22
#define MOTOR2_PIN2             23
#define MOTOR3_PIN1             25
#define MOTOR3_PIN2             26
#define MOTOR4_PIN1             32
#define MOTOR4_PIN2             33

#define GPIO_OUTPUT_PIN_SEL    ((1ULL<<MOTOR1_PIN1) | \
                                (1ULL<<MOTOR1_PIN2) | \
                                (1ULL<<MOTOR2_PIN1) | \
                                (1ULL<<MOTOR2_PIN2) | \
                                (1ULL<<MOTOR3_PIN1) | \
                                (1ULL<<MOTOR3_PIN2) | \
                                (1ULL<<MOTOR4_PIN1) | \
                                (1ULL<<MOTOR4_PIN2))

#define WHEEL_RADIUS            214 / 1000.0
#define SENSOR_RESOLUTION       40

struct motor_t {
  uint8_t pin1;
  uint8_t pin2;
  uint8_t pin_sensor;
  pcnt_unit_handle_t pcnt_unit;
  uint32_t lastRefreshTime;
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

static void motor_speed_update(struct motor_t *motor) {
  uint8_t diff = xTaskGetTickCount() - motor->lastRefreshTime;
  motor->lastRefreshTime += diff;

  uint8_t count = 0;
  pcnt_unit_get_count(motor->pcnt_unit, &count);
  pcnt_unit_clear_count(motor->pcnt_unit);

  motor->speed = WHEEL_RADIUS / SENSOR_RESOLUTION * count / (diff * portTICK_PERIOD_MS / 1000.0);
}

static void motor_init(struct motor_t *motor) {
    gpio_set_level(motor->pin1, 0);
    gpio_set_level(motor->pin2, 0);
}

static void gpio_config_init() {
    ESP_LOGI(TAG, "initialize gpio");
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
}

void app_main()
{
    struct motor_t motors[MOTOR_COUNT] = {
        {MOTOR1_PIN1, MOTOR1_PIN2, SENSOR_PIN1, NULL, 0, 0.0},
        {MOTOR2_PIN1, MOTOR2_PIN2, SENSOR_PIN2, NULL, 0, 0.0},
        {MOTOR3_PIN1, MOTOR3_PIN2, SENSOR_PIN3, NULL, 0, 0.0},
        {MOTOR4_PIN1, MOTOR4_PIN2, SENSOR_PIN4, NULL, 0, 0.0}
    };

    gpio_config_init();

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motor_init(&motors[i]);
        motor_sensor_init(&motors[i]);
    }

    while (1) {
        vTaskDelay(250 / portTICK_PERIOD_MS);

        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            motor_speed_update(&motors[i]);
        }

        ESP_LOGI(TAG, "motor speeds: %f, %f, %f, %f", motors[0].speed, motors[1].speed, motors[2].speed, motors[3].speed);
    }
}
