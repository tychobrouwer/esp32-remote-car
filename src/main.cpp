#include <Arduino.h>
#include <neoTimer.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <math.h>

#include "index.h"
#include "wifi_credentials.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const float WHEEL_CIRCUM = 0.21;
const byte WHEEL_STEPS = 20;
const unsigned int WHEEL_CALC_INTERVAL = 1000;

const byte MOTOR_1_SENSOR = 16;
const byte MOTOR_2_SENSOR = 17;
const byte MOTOR_3_SENSOR = 5;
const byte MOTOR_4_SENSOR = 18;

const byte MOTOR_1_IN1 = 13;
const byte MOTOR_1_IN2 = 12;
const byte MOTOR_2_IN1 = 27;
const byte MOTOR_2_IN2 = 14;
const byte MOTOR_3_IN1 = 26;
const byte MOTOR_3_IN2 = 25;
const byte MOTOR_4_IN1 = 33;
const byte MOTOR_4_IN2 = 32;

const byte MOTOR_1_IN1_CHANNEL = 0;
const byte MOTOR_1_IN2_CHANNEL = 1;
const byte MOTOR_2_IN1_CHANNEL = 2;
const byte MOTOR_2_IN2_CHANNEL = 3;
const byte MOTOR_3_IN1_CHANNEL = 4;
const byte MOTOR_3_IN2_CHANNEL = 5;
const byte MOTOR_4_IN1_CHANNEL = 6;
const byte MOTOR_4_IN2_CHANNEL = 7;

const byte MOTOR_STARTUP_LENGTH = 50;

const unsigned int PWM_FREQ = 5000;
const byte PWM_RESOLUTION = 8;

Neotimer speedTimer = Neotimer(WHEEL_CALC_INTERVAL);

byte motorSensorCounter[4] = {0, 0, 0, 0};
unsigned int motorSpeed[4] = {0, 0, 0, 0};

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

Neotimer motorTimers[4] = {
    Neotimer(MOTOR_STARTUP_LENGTH),
    Neotimer(MOTOR_STARTUP_LENGTH),
    Neotimer(MOTOR_STARTUP_LENGTH),
    Neotimer(MOTOR_STARTUP_LENGTH)};

byte motorSpeeds[4] = {0, 0, 0, 0};
bool motorForwards[4] = {true, true, true, true};

int robotSpeed = 0;
int robotSteering = 0;

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

void calcSpeed()
{
  for (int i = 0; i < 4; i++)
  {
    motorSpeed[i] = int(motorSensorCounter[i] * WHEEL_CIRCUM / WHEEL_STEPS * WHEEL_CALC_INTERVAL);
    motorSensorCounter[i] = 0;
  }

  // Serial.print("speed 1 [mm/s]: ");
  // Serial.print(motorSpeed[0]);
  // Serial.print("  |  speed 2 [mm/s]: ");
  // Serial.print(motorSpeed[1]);
  // Serial.print("  |  speed 3 [mm/s]: ");
  // Serial.print(motorSpeed[2]);
  // Serial.print("  |  speed 4 [mm/s]: ");
  // Serial.println(motorSpeed[3]);
}

void writeMotorSpeed(byte motorIdx, byte speed, bool forward)
{
  ledcWrite(MOTOR_CHANNELS[motorIdx * 2 + !forward], speed);
  ledcWrite(MOTOR_CHANNELS[motorIdx * 2 + forward], 0);
}

void updateMotorSpeed(byte motorIdx, byte speed, bool forward)
{
  if (motorSpeeds[motorIdx] != speed)
  {
    writeMotorSpeed(motorIdx, 255, forward);

    motorTimers[motorIdx].start();
    motorSpeeds[motorIdx] = speed;
    motorForwards[motorIdx] = forward;
  }
}

void checkMotorSpeeds()
{
  for (int motorIdx = 0; motorIdx < 4; motorIdx++)
  {
    if (motorTimers[motorIdx].done())
    {
      motorTimers[motorIdx].reset();
      writeMotorSpeed(motorIdx, motorSpeeds[motorIdx], motorForwards[motorIdx]);
    }
  }
}

void notifyClients()
{
  int updateSpeed = 100.0 - (255.0 - robotSpeed) / 510.0 * 100.0;
  int updateSteering = (robotSteering + 255.0) / 510.0 * 100.0;
  String message = String(updateSpeed) + "," + String(updateSteering);

  ws.textAll(message);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    String message = (char *)data;
    robotSpeed = message.substring(0, message.indexOf(",")).toInt();
    robotSteering = message.substring(message.indexOf(",") + 1).toInt();

    // Serial.print("speed: ");
    // Serial.print(robotSpeed);
    // Serial.print("  |  steering: ");
    // Serial.println(robotSteering);

    notifyClients();
  }
}

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

String processor(const String &var)
{
  if (var == "SPEED_PERCENTAGE")
  {
    int value = 100.0 - (255.0 - robotSpeed) / 510.0 * 100.0;
    return String(value) + "%";
  }
  else if (var == "STEERING_PERCENTAGE")
  {
    int value = (robotSteering + 255.0) / 510.0 * 100.0;
    return String(value) + "%";
  }

  return String();
}

void setup()
{
  Serial.begin(115200);

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

  Serial.println("Starting server...");
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", MAIN_page, processor); });
  server.begin();

  Serial.println("Setting up interrupts...");
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_SENSOR), motorSensorCount1, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_SENSOR), motorSensorCount2, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_SENSOR), motorSensorCount3, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_4_SENSOR), motorSensorCount4, RISING);
  speedTimer.set(WHEEL_CALC_INTERVAL);

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
  if (speedTimer.repeat())
  {
    calcSpeed();
  }

  checkMotorSpeeds();

  ws.cleanupClients();

  bool forward = robotSpeed >= 0;
  byte motorSpeed = abs(robotSpeed);

  byte steeringMultiplier = abs(robotSteering / 10.0);
  byte steeringSpeed = constrain(motorSpeed + steeringMultiplier, 0, 255);
  byte negativeSteeringSpeed =
      steeringSpeed == 255 ? motorSpeed - (motorSpeed + steeringMultiplier) % 255 : motorSpeed;

  if (robotSteering > 0)
  {
    updateMotorSpeed(0, negativeSteeringSpeed, forward);
    updateMotorSpeed(1, negativeSteeringSpeed, forward);
    updateMotorSpeed(2, steeringSpeed, forward);
    updateMotorSpeed(3, steeringSpeed, forward);
  }
  else
  {
    updateMotorSpeed(0, steeringSpeed, forward);
    updateMotorSpeed(1, steeringSpeed, forward);
    updateMotorSpeed(2, negativeSteeringSpeed, forward);
    updateMotorSpeed(3, negativeSteeringSpeed, forward);
  }
}
