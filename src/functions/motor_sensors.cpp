#include "motor_sensors.h"

byte motorSensorCounter[4] = {0, 0, 0, 0};
unsigned int wheelSpeed[4] = {0, 0, 0, 0};

// Counter functions for motor speed calculation
void motorSensorCount1()
{
  motorSensorCounter[0]++;
}
void motorSensorCount2()
{
  motorSensorCounter[1]++;
}
void motorSensorCount3()
{
  motorSensorCounter[2]++;
}
void motorSensorCount4()
{
  motorSensorCounter[3]++;
}

// Function for calculating wheel speed
void calcSpeed()
{
  for (int i = 0; i < 4; i++)
  {
    // Calculate wheel speed in mm/s based on wheel circumference, number of steps, and time interval
    wheelSpeed[i] = int(motorSensorCounter[i] * WHEEL_CIRCUM / WHEEL_STEPS * WHEEL_CALC_INTERVAL);
    // Reset light slot counter
    motorSensorCounter[i] = 0;
  }

  // Serial.print("speed 1 [mm/s]: ");
  // Serial.print(wheelSpeed[0]);
  // Serial.print("  |  speed 2 [mm/s]: ");
  // Serial.print(wheelSpeed[1]);
  // Serial.print("  |  speed 3 [mm/s]: ");
  // Serial.print(wheelSpeed[2]);
  // Serial.print("  |  speed 4 [mm/s]: ");
  // Serial.println(wheelSpeed[3]);
}
