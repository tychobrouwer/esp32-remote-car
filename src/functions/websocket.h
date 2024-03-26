#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>

extern AsyncWebServer server;
extern AsyncWebSocket ws;

extern int robotSpeed;
extern int robotSteering;

void notifyClients();

// Function for handling incoming websocket messages
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);

// Function for handling websocket events
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len);

// Function for handling template variables in HTML
String processor(const String &var);

#endif
