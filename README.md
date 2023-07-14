# ESP32 code for remote controlled robot

This project is about learning to work with microcontrollers, electrical components, and writing the code for them.

The robot is controlled by an ESP32-WROOM-32 microcontroller, and uses two MX1508 DC-motor drivers to drive the four 5V cheap motors. The speed of the wheels is tracked using four light slot sensors.

The robot can be controlled using a webserver hosted on the ESP's IP-address. The webinterface communicates back to the microcontroller using a websocket connection.

## Components

- ESP32-WROOM-32 microcontroller
- Battery holder 4xAA
- 2x MX1508 DC-motor drivers
- 4x 3-6V 70mA DC-motor with a 1:48 gear ratio
- 4x Wheel with 7cm diameter
- Generic AC/DC power adapter 5V-1A

## Installation

1. Install and configure PlatformIO as a VS Code extension
2. Build the code (bottom control bar)
3. In ```.pio/libdeps/ESP Async WebServer/WebResponseImpl.h``` change the ```TEMPLATE_PLACEHOLDER``` variable to ```'?'```.
