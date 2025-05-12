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
const int touchPin1 = 5;   // GPIO5 (D5)
const int touchPin2 = 18;  // GPIO18 (D18)

// States
bool lastButton1State;
bool lastButton2State;
bool touch1WasCounted = false;
bool touch2WasCounted = false;

// Counts
int brakeCount1 = 0;
int brakeCount2 = 0;
int touchCount1 = 0;
int touchCount2 = 0;

volatile int speedPulseCount = 0;
int lastSpeed = 0;
unsigned long lastSpeedCheck = 0;

const char* deviceID = "ESP1";

// Speed sensor interrupt handler
void IRAM_ATTR onSpeedSensor() {
  speedPulseCount++;
}

// LCD update
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("FR:");
  lcd.print(brakeCount1);
  lcd.print(" FL:");
  lcd.print(brakeCount2);
  lcd.print("  ");

  lcd.setCursor(0, 1);
  lcd.print("SPD:");
  lcd.print(lastSpeed);
  lcd.print(" BH:");
  lcd.print(touchCount1);
  lcd.print(" BF:");
  lcd.print(touchCount2);
  lcd.print(" ");
}

// Send Bluetooth JSON
void sendJsonData() {
  StaticJsonDocument<256> doc;
  doc["device"] = deviceID;
  doc["timestamp"] = millis();
  doc["brakeCount1"] = brakeCount1;
  doc["brakeCount2"] = brakeCount2;
  doc["touchCount1"] = touchCount1;
  doc["touchCount2"] = touchCount2;
  doc["speed"] = lastSpeed;

  String output;
  serializeJson(doc, output);
  SerialBT.println(output);
  Serial.println(output);

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

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(speedSensorPin, INPUT_PULLUP);
  pinMode(touchPin1, INPUT);
  pinMode(touchPin2, INPUT);

  lastButton1State = digitalRead(button1Pin);
  lastButton2State = digitalRead(button2Pin);

  attachInterrupt(digitalPinToInterrupt(speedSensorPin), onSpeedSensor, FALLING);

  delay(1000);
  lcd.clear();
  updateLCD();
}

void loop() {
  // Button brake detections
  bool button1State = digitalRead(button1Pin);
  bool button2State = digitalRead(button2Pin);

  if (button1State == LOW && lastButton1State == HIGH) {
    brakeCount1++;
    sendJsonData();
  }
  lastButton1State = button1State;

  if (button2State == LOW && lastButton2State == HIGH) {
    brakeCount2++;
    sendJsonData();
  }
  lastButton2State = button2State;

  // TOUCH 1 (D5)
  bool touch1State = digitalRead(touchPin1);
  if (touch1State == HIGH && !touch1WasCounted) {
    touchCount1++;
    Serial.println("Touched D5");
    sendJsonData();
    touch1WasCounted = true;
  }
  if (touch1State == LOW) {
    touch1WasCounted = false;
  }

  // TOUCH 2 (D18)
  bool touch2State = digitalRead(touchPin2);
  if (touch2State == HIGH && !touch2WasCounted) {
    touchCount2++;
    Serial.println("Touched D18");
    sendJsonData();
    touch2WasCounted = true;
  }
  if (touch2State == LOW) {
    touch2WasCounted = false;
  }

  // Speed check every second
  unsigned long now = millis();
  if (now - lastSpeedCheck >= 1000) {
    int currentSpeed = speedPulseCount;
    if (currentSpeed != lastSpeed) {
      lastSpeed = currentSpeed;
      sendJsonData();
    }
    speedPulseCount = 0;
    lastSpeedCheck = now;
  }
}
