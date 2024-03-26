#ifndef MOTOR_SENSORS_H
#define MOTOR_SENSORS_H

#include <Arduino.h>

#include "../constants/wheels.h"

extern byte motorSensorCounter[4];
extern unsigned int wheelSpeed[4];

// Counter functions for motor speed calculation
void motorSensorCount1();
void motorSensorCount2();
void motorSensorCount3();
void motorSensorCount4();

// Function for calculating wheel speed
void calcSpeed();

#endif
