#include <Arduino.h>

// Multiplier for calculating steering values based on inputted steering
const byte STEERING_DIVISOR = 2;

// Multiplier for correcting motor speed based on wheel speed
const byte SPEED_CORRECTION_DIVISOR = 20;
