#ifndef MOTOR_CONSTANTS_H
#define MOTOR_CONSTANTS_H

#include <Arduino.h>

// Motor pins
const byte MOTOR_1_IN1 = 13;
const byte MOTOR_1_IN2 = 12;
const byte MOTOR_2_IN1 = 27;
const byte MOTOR_2_IN2 = 14;
const byte MOTOR_3_IN1 = 26;
const byte MOTOR_3_IN2 = 25;
const byte MOTOR_4_IN1 = 33;
const byte MOTOR_4_IN2 = 32;

// Motor PWM channels
const byte MOTOR_1_IN1_CHANNEL = 0;
const byte MOTOR_1_IN2_CHANNEL = 1;
const byte MOTOR_2_IN1_CHANNEL = 2;
const byte MOTOR_2_IN2_CHANNEL = 3;
const byte MOTOR_3_IN1_CHANNEL = 4;
const byte MOTOR_3_IN2_CHANNEL = 5;
const byte MOTOR_4_IN1_CHANNEL = 6;
const byte MOTOR_4_IN2_CHANNEL = 7;

// Motor PWM channels and pins
const byte MOTOR_CHANNELS[8] = {
    MOTOR_1_IN1_CHANNEL, MOTOR_1_IN2_CHANNEL,
    MOTOR_2_IN1_CHANNEL, MOTOR_2_IN2_CHANNEL,
    MOTOR_3_IN1_CHANNEL, MOTOR_3_IN2_CHANNEL,
    MOTOR_4_IN1_CHANNEL, MOTOR_4_IN2_CHANNEL};
const byte MOTOR_PINS[8] = {
    MOTOR_1_IN1, MOTOR_1_IN2,
    MOTOR_2_IN1, MOTOR_2_IN2,
    MOTOR_3_IN1, MOTOR_3_IN2,
    MOTOR_4_IN1, MOTOR_4_IN2};

// Motor driver PWM constants
const unsigned int PWM_FREQ = 8000;
const byte PWM_RESOLUTION = 8;

// Motor speed constants
const byte MOTOR_STARTUP_LENGTH = 50;
const byte MOTOR_MINIMUM_SPEED = 100;
const byte MOTOR_MINIMUM_DIFF = 10;

#endif
