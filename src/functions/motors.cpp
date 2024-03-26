#include "motors.h"

// Motor timers for startup
Neotimer motorTimers[4] = {
    Neotimer(MOTOR_STARTUP_LENGTH),
    Neotimer(MOTOR_STARTUP_LENGTH),
    Neotimer(MOTOR_STARTUP_LENGTH),
    Neotimer(MOTOR_STARTUP_LENGTH)};

// Motor speed and direction
byte motorSpeeds[4] = {0, 0, 0, 0};
bool motorForwards[4] = {true, true, true, true};

// Function for writing motor speed
void writeMotorSpeed(byte motorIdx, byte speed, bool forward)
{
  // Write speed to motor
  ledcWrite(MOTOR_CHANNELS[motorIdx * 2 + !forward], speed);
  ledcWrite(MOTOR_CHANNELS[motorIdx * 2 + forward], 0);
}

// Function for updating motor speed
void updateMotorSpeed(byte motorIdx, byte speed, bool forward)
{
  if (abs(motorSpeeds[motorIdx] - speed) > MOTOR_MINIMUM_DIFF)
  {
    // If motor is stopped, start it at full speed for a short time
    if (wheelSpeed[motorIdx] == 0)
    {
      // Set motor speed to 200 for a short time
      writeMotorSpeed(motorIdx, 200, forward);

      // Start timer for writing correct motor speed after startup
      motorTimers[motorIdx].start();
      motorSpeeds[motorIdx] = speed;
      motorForwards[motorIdx] = forward;
    }
    else
    {
      // Set motor speed
      writeMotorSpeed(motorIdx, speed, forward);
    }
  }
}

// Function for setting motor speed after startup time has elapsed
void checkMotorSpeeds()
{
  for (int motorIdx = 0; motorIdx < 4; motorIdx++)
  {
    if (motorTimers[motorIdx].done())
    {
      // Reset timer
      motorTimers[motorIdx].reset();
      // Write correct motor speed
      writeMotorSpeed(motorIdx, motorSpeeds[motorIdx], motorForwards[motorIdx]);
    }
  }
}
