#include <Arduino.h>
#include "BluetoothSerial.h"
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

BluetoothSerial SerialBT;
LiquidCrystal_I2C lcd(0x27, 16, 2);  

const int button1Pin = 12;
const int button2Pin = 27;
const int speedSensorPin = 34;

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

void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("B1:");
  lcd.print(brakeCount1);
  lcd.print(" B2:");
  lcd.print(brakeCount2);
  lcd.print("  ");  // Padding to clear leftover chars

  lcd.setCursor(0, 1);
  lcd.print("SPD:");
  lcd.print(lastSpeed);
  lcd.print("     ");  // Padding
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

  updateLCD(); 
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BRAKE_SENSOR");

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Starting...");

  pinMode(button1Pin, INPUT_PULLUP);   // will be changed under project testing.
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(speedSensorPin, INPUT_PULLUP);

  lastButton1State = digitalRead(button1Pin);
  lastButton2State = digitalRead(button2Pin);

  attachInterrupt(digitalPinToInterrupt(speedSensorPin), onSpeedSensor, FALLING);

  delay(1000);
  lcd.clear();
  updateLCD(); 
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
