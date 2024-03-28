#include <ESP32MX1508.h>

#define RES 8
#define FREQ 5000

static const int sensorPins[4]   = {34, 35, 36, 39};
static const int motorPins[8]    = {19, 21, 22, 23, 25, 26, 32, 33};

static const float radius        = 21.4 / 100;
static const unsigned int steps  = 40;

bool valsOld[4]                  = {0, 0, 0, 0};
float speeds[4]                  = {0, 0, 0, 0};
unsigned long lastRefreshTime[4] = {0, 0, 0, 0};
unsigned int sensorCounters[4]   = {0, 0, 0, 0};

float results = 0;
unsigned int i = 0;
unsigned int j = 0;
float averaged[255] = {};

MX1508 motor0(motorPins[6], motorPins[7], 6, 7, RES, FREQ);
MX1508 motor1(motorPins[1], motorPins[0], 0, 1, RES, FREQ);
MX1508 motor2(motorPins[4], motorPins[5], 4, 5, RES, FREQ);
MX1508 motor3(motorPins[3], motorPins[2], 2, 3, RES, FREQ);

void setup() {
  pinMode(sensorPins[0], INPUT);
  pinMode(sensorPins[1], INPUT);
  pinMode(sensorPins[2], INPUT);
  pinMode(sensorPins[3], INPUT);

  Serial.begin(921600);
}

void speed(int motorIndex) {
  unsigned long diff = millis() - lastRefreshTime[motorIndex];
  float speed = radius / steps / ((float)diff / 1000);

  if (speed < 1) speeds[motorIndex] = speed;

  sensorCounters[motorIndex] = 0;
  lastRefreshTime[motorIndex] += diff;
}

void arrayCopy(bool* src, bool* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
}

void loop() {

  bool vals[4] = {
    digitalRead(sensorPins[0]) == HIGH,
    digitalRead(sensorPins[1]) == HIGH,
    digitalRead(sensorPins[2]) == HIGH,
    digitalRead(sensorPins[3]) == HIGH
  };

  if (vals[0] != valsOld[0]) speed(0);
  if (sensorCounters[0] > 200) { speeds[0] = 0; sensorCounters[0] = 0; };
  sensorCounters[0]++;

  if (vals[1] != valsOld[1]) speed(1);
  if (sensorCounters[1] > 200) { speeds[1] = 0; sensorCounters[1] = 0; };
  sensorCounters[1]++;

  if (vals[2] != valsOld[2]) speed(2);
  if (sensorCounters[2] > 200) { speeds[2] = 0; sensorCounters[2] = 0; };
  sensorCounters[2]++;

  if (vals[3] != valsOld[3]) speed(3);
  if (sensorCounters[3] > 200) { speeds[3] = 0; sensorCounters[3] = 0; };
  sensorCounters[3]++;

  arrayCopy(vals, valsOld, 4);

  motor0.motorGo(j);
  results += speeds[0];
  i++;

  if (j > 255) {
    j = 0;
  }

  if (i > 100000) {
    float average = results / 100000;
    results = 0;
    i = 0;
    j++;

    Serial.print("average speed for PWM ");
    Serial.print(j);
    Serial.print(" is ");
    Serial.println(average);
  } 
	
  // Serial.print(speeds[0]);
  // Serial.print(",");
  // Serial.print(speeds[1]);
  // Serial.print(",");
  // Serial.print(speeds[2]);
  // Serial.print(",");
  // Serial.println(speeds[3]);
}
