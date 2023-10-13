#include <Arduino.h>
#include <neoTimer.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// index.h contains the HTML code for the webserver
#include "index.h"
// wifi_credentials.h contains the credentials for the wifi network
#include "wifi_credentials.h"

// Webserver and websocket objects
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Wheel constants
const float WHEEL_CIRCUM = 0.21;
const byte WHEEL_STEPS = 20;
const unsigned int WHEEL_CALC_INTERVAL = 500;

// Motor light slot sensor pins
const byte MOTOR_1_SENSOR = 16;
const byte MOTOR_2_SENSOR = 17;
const byte MOTOR_3_SENSOR = 5;
const byte MOTOR_4_SENSOR = 18;

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

// Multiplier for calculating steering values based on inputted steering
const byte STEERING_DIVISOR = 2;

// Multiplier for correcting motor speed based on wheel speed
const byte SPEED_CORRECTION_DIVISOR = 20;

// Motor driver PWM constants
const unsigned int PWM_FREQ = 16000;
const byte PWM_RESOLUTION = 8;

// Motor constants
const byte MOTOR_STARTUP_LENGTH = 50;
const byte MOTOR_MINIMUM_SPEED = 100;
const byte MOTOR_MINIMUM_DIFF = 10;

// Timer for calculating wheel speed
Neotimer speedTimer = Neotimer(WHEEL_CALC_INTERVAL);

// Variables for storing wheel speed
byte motorSensorCounter[4] = {0, 0, 0, 0};
unsigned int wheelSpeed[4] = {0, 0, 0, 0};

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

// Motor timers for startup
Neotimer motorTimers[4] = {
    Neotimer(MOTOR_STARTUP_LENGTH),
    Neotimer(MOTOR_STARTUP_LENGTH),
    Neotimer(MOTOR_STARTUP_LENGTH),
    Neotimer(MOTOR_STARTUP_LENGTH)};

// Motor speed and direction
byte motorSpeeds[4] = {0, 0, 0, 0};
bool motorForwards[4] = {true, true, true, true};

// Inputted robot speed and steering
int robotSpeed = 0;
int robotSteering = 0;

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

  Serial.print("speed 1 [mm/s]: ");
  Serial.print(wheelSpeed[0]);
  Serial.print("  |  speed 2 [mm/s]: ");
  Serial.print(wheelSpeed[1]);
  Serial.print("  |  speed 3 [mm/s]: ");
  Serial.print(wheelSpeed[2]);
  Serial.print("  |  speed 4 [mm/s]: ");
  Serial.println(wheelSpeed[3]);
}

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

void notifyClients()
{
  int updateSpeed = 100.0 - (255.0 - robotSpeed) / 510.0 * 100.0;
  int updateSteering = (robotSteering + 255.0) / 510.0 * 100.0;

  String message = String(updateSpeed) + "," + String(updateSteering);

  // Send message to all connected websocket clients
  ws.textAll(message);
}

// Function for handling incoming websocket messages
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    String message = (char *)data;

    // Extract speed and steering values from message (format: "speed,steering")
    robotSpeed = message.substring(0, message.indexOf(",")).toInt();
    robotSteering = message.substring(message.indexOf(",") + 1).toInt();

    // Notify connected clients of updated values
    notifyClients();
  }
}

// Function for handling websocket events
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

// Function for handling template variables in HTML
String processor(const String &var)
{
  if (var == "SPEED_PERCENTAGE")
  {
    // Calculate percentage value for speed
    int value = (255.0 - robotSpeed) / 510.0 * 100.0;
    return String(value) + "%";
  }
  else if (var == "STEERING_PERCENTAGE")
  {
    // Calculate percentage value for steering
    int value = (robotSteering + 255.0) / 510.0 * 100.0;
    return String(value) + "%";
  }

  return String();
}

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
