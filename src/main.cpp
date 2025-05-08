#include <Arduino.h>
#include "BluetoothSerial.h"
#include <ArduinoJson.h>

BluetoothSerial SerialBT;

const int button1Pin = 12;
const int button2Pin = 27;
const int speedSensorPin = 32;

bool lastButton1State;
bool lastButton2State;

int brakeCount1 = 0;
int brakeCount2 = 0;

volatile int speedPulseCount = 0;
int lastSpeed = 0;
unsigned long lastSpeedCheck = 0;

const char* deviceID = "ESP1";

// Interrupt to count speed sensor pulses
void IRAM_ATTR onSpeedSensor() {
  speedPulseCount++;
}

// Send data in JSON format over Bluetooth and Serial
void sendJsonData() {
  StaticJsonDocument<256> doc;
  doc["device"] = deviceID;
  doc["timestamp"] = millis();
  doc["brakeCount1"] = brakeCount1;
  doc["brakeCount2"] = brakeCount2;
  doc["speed"] = lastSpeed;

  String output;
  serializeJson(doc, output);
  SerialBT.println(output);
  Serial.println(output); // Optional: log to USB serial
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BRAKE_SENSOR");

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(speedSensorPin, INPUT_PULLUP);

  lastButton1State = digitalRead(button1Pin);
  lastButton2State = digitalRead(button2Pin);

  attachInterrupt(digitalPinToInterrupt(speedSensorPin), onSpeedSensor, FALLING);
}

void loop() {
  // Check button 1
  bool button1State = digitalRead(button1Pin);
  if (button1State == LOW && lastButton1State == HIGH) {
    brakeCount1++;
    sendJsonData();
  }
  lastButton1State = button1State;

  // Check button 2
  bool button2State = digitalRead(button2Pin);
  if (button2State == LOW && lastButton2State == HIGH) {
    brakeCount2++;
    sendJsonData();
  }
  lastButton2State = button2State;

  // Speed calculation every second
  unsigned long now = millis();
  if (now - lastSpeedCheck >= 1000) {
    int currentSpeed = speedPulseCount; // Adjust unit as needed
    if (currentSpeed != lastSpeed) {
      lastSpeed = currentSpeed;
      sendJsonData();
    }
    speedPulseCount = 0;
    lastSpeedCheck = now;
  }
}
