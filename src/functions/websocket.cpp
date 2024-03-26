#include "websocket.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

int robotSpeed = 0;
int robotSteering = 0;

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

    // Serial.print("Speed: ");
    // Serial.print(robotSpeed);
    // Serial.print("  |  Steering: ");
    // Serial.println(robotSteering);

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
