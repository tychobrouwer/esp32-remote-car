#include <Arduino.h>
#include <WiFi.h>

#include "webserver/index.h"
#include "timer/neotimer.h"

#include "constants/wifi_credentials.h"
#include "constants/motors.h"
#include "constants/sensors.h"
#include "constants/wheels.h"
#include "constants/controlling.h"

#include "functions/motor_sensors.h"
#include "functions/websocket.h"
#include "functions/motors.h"

// Timer for calculating wheel speed
Neotimer speedTimer = Neotimer(WHEEL_CALC_INTERVAL);

void setup()
{
  Serial.begin(115200);

  // Connect to WiFi
  Serial.println("");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Set up websocket server
  Serial.println("Starting server...");
  ws.onEvent(onEvent);
  // Set up HTTP server
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", MAIN_page, processor); });
  server.begin();

  // Set up motor sensors for interrupts
  Serial.println("Setting up interrupts...");
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_SENSOR), motorSensorCount1, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_SENSOR), motorSensorCount2, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_SENSOR), motorSensorCount3, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_4_SENSOR), motorSensorCount4, RISING);

  // Set up timer for calculating wheel speeds
  speedTimer.set(WHEEL_CALC_INTERVAL);

  // Set up motors for PWM
  Serial.println("Setting up motors...");
  for (int i = 0; i < 8; i++)
  {
    ledcSetup(MOTOR_CHANNELS[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PINS[i], MOTOR_CHANNELS[i]);
  }

  Serial.println("Finished setup!");
}

void loop()
{
  // Calculate wheel speeds at regular intervals
  if (speedTimer.repeat())
  {
    calcSpeed();
  }

  // Check if motor speed needs to be updated after startup
  checkMotorSpeeds();

  // Cleanup disconnected clients
  ws.cleanupClients();

  // Variables for determining the motor speeds
  bool forward = robotSpeed >= 0;
  byte motorSpeed = abs(robotSpeed);

  // Calculate value difference between left and right motors
  byte steeringValue = abs(robotSteering / STEERING_DIVISOR);
  // Calculate high side value for motors
  byte motorSpeedHighSide = constrain(motorSpeed + steeringValue, 0, 255);
  // Calculate low side value for motors to ensure the difference is correct
  byte motorSpeedLowSide =
      motorSpeedHighSide == 255 ? motorSpeed - (motorSpeed + steeringValue) % 255 : motorSpeed;

  // Map motor speeds to correct range
  motorSpeedHighSide = map(motorSpeedHighSide, 0, 255, MOTOR_MINIMUM_SPEED, 255);
  motorSpeedLowSide = map(motorSpeedLowSide, 0, 255, MOTOR_MINIMUM_SPEED, 255);

  // Correction for motor speed based on wheel speed
  int motorCorrection12 = ((int)wheelSpeed[1] - (int)wheelSpeed[2]) / SPEED_CORRECTION_DIVISOR;
  int motorCorrection34 = ((int)wheelSpeed[3] - (int)wheelSpeed[4]) / SPEED_CORRECTION_DIVISOR;

  // Set motor speeds based on steering direction
  if (robotSteering > 0)
  {
    Serial.print("motorSpeed: ");
    Serial.println(motorSpeedLowSide - motorCorrection12);

    updateMotorSpeed(0, motorSpeedLowSide - motorCorrection12, forward);
    updateMotorSpeed(1, motorSpeedLowSide + motorCorrection12, forward);
    updateMotorSpeed(2, motorSpeedHighSide - motorCorrection34, forward);
    updateMotorSpeed(3, motorSpeedHighSide + motorCorrection34, forward);
  }
  else
  {
    updateMotorSpeed(0, motorSpeedHighSide - motorCorrection12, forward);
    updateMotorSpeed(1, motorSpeedHighSide + motorCorrection12, forward);
    updateMotorSpeed(2, motorSpeedLowSide - motorCorrection34, forward);
    updateMotorSpeed(3, motorSpeedLowSide + motorCorrection34, forward);
  }
}
