//////////////////////////////////////////////////////////////////////////////////////
//LIBRARIES
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "AsyncJson.h"
#include "ArduinoJson.h"
//////////////////////////////////////////////////////////////////////////////////////

//VARIABLES AND PORTS
// Define the UART pins (ESP32 default pins)
#define RXD2 16 // RX pin for UART2
#define TXD2 17 // TX pin for UART2

// WiFi Credentials
const char* ssid = "IDMindYouship";
const char* password = "IDMind2000!";
//Server Address
const char* serverAddress = "192.168.8.109"; 

// Define the AsyncWebServer port
AsyncWebServer server(80);

// Timer to track last received message time
unsigned long lastMessageTime = 0;  // Stores the time when the last message was received
unsigned long timeoutInterval = 500; // 500 milliseconds timeout

//////////////////////////////////////////////////////////////////////////////////////

void setup() {
//////////////////////////////////////////////////////////////////////////////////////
//INITIALIZATION
  // Initialize Serial for debugging
  Serial.begin(115200);
  // Initialize Serial2 for UART communication
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Set ESP32 WiFi to Station Mode
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println("Connecting to WiFi...");

//////////////////////////////////////////////////////////////////////////////////////
//CONNECTION

  Serial.println("Connecting to WiFi...");
  // Waiting for the connection
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 20) {
    Serial.printf("WiFi Connection Attempt %d Failed. Retrying...\n", ++attempt);
    delay(1000);
  }

  // Check connection result
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); // IP Address
  } else {
    Serial.println("WiFi connection failed. Please check your credentials and try again.");
    return; // Exit if connection failed
  }
//////////////////////////////////////////////////////////////////////////////////////
//TESTING

  //Test GET request
  server.on("/get_message/test", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Test GET successful");
  });
  //Test POST request
  server.on("/post_message/test", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Test POST successful");
  });

//////////////////////////////////////////////////////////////////////////////////////

  // Handle POST request for sending sonar data
  server.on("/post_message", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("message", true)) {
      String message = request->getParam("message", true)->value();
      // Serial.printf("Received message: %s\n", message.c_str());
      request->send(200, "text/plain", "Message sent successfully to ESP32");
      Serial2.println(message);

      // Reset the last message time
      lastMessageTime = millis();
    } 
    else {
      request->send(400, "text/plain", "Message parameter missing");
    }
  });

//////////////////////////////////////////////////////////////////////////////////////

  // Start server
  server.begin();
}

void loop() {
  // Check if more than 500ms have passed since the last message
  if (millis() - lastMessageTime >= timeoutInterval) {
    // Send the message b'\x86\x00\x00\x00\x00' to the Arduino
    Serial.println("Timeout: Sending b'\\x86\\x00\\x00\\x00\\x00' to Arduino");
    Serial2.write(0x86); // Sending binary values
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x00);

    // Update lastMessageTime to prevent repeated sends
    lastMessageTime = millis();
  }
}
