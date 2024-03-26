#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

#include "../constants/motors.h"
#include "motor_sensors.h"
#include "../timer/neotimer.h"

extern Neotimer motorTimers[4];

extern byte motorSpeeds[4];
extern bool motorForwards[4];

void writeMotorSpeed(byte motorIdx, byte speed, bool forward);

// Function for updating motor speed
void updateMotorSpeed(byte motorIdx, byte speed, bool forward);

// Function for setting motor speed after startup time has elapsed
void checkMotorSpeeds();

#endif
