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
const int touchPin1 = 5;    
const int touchPin2 = 18;  

#define WHEEL_DIAMETER 0.6  


bool lastButton1State;
bool lastButton2State;
bool touch1WasCounted = false;
bool touch2WasCounted = false;

int brakeCount1 = 0;
int brakeCount2 = 0;
int touchCount1 = 0;
int touchCount2 = 0;

volatile int speedPulseCount = 0;
int lastSpeed = 0;
unsigned long lastSpeedCheck = 0;

unsigned long lastBrake1Time = 0;
unsigned long lastBrake2Time = 0;
unsigned long lastTouch1Time = 0;
unsigned long lastTouch2Time = 0;

const unsigned long debounceInterval = 2000; 
const char* deviceID = "ESP1";

void IRAM_ATTR onSpeedSensor() {
  speedPulseCount++;
}

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
  SerialBT.begin("ESP32_BRAKE");

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
  unsigned long now = millis();

  // === Brake Buttons ===
  bool button1State = digitalRead(button1Pin);
  bool button2State = digitalRead(button2Pin);

  if (button1State == HIGH && lastButton1State == LOW) {
    if (now - lastBrake1Time >= debounceInterval) {
      brakeCount1++;
      lastBrake1Time = now;
      Serial.println("Released Button 1 (Debounced)");
      sendJsonData();
    }
  }
  lastButton1State = button1State;

  if (button2State == HIGH && lastButton2State == LOW) {
    if (now - lastBrake2Time >= debounceInterval) {
      brakeCount2++;
      lastBrake2Time = now;
      Serial.println("Released Button 2 (Debounced)");
      sendJsonData();
    }
  }
  lastButton2State = button2State;

  // === Touch Sensors with 2s Debounce ===
  bool touch1State = digitalRead(touchPin1);
  if (touch1State == HIGH && !touch1WasCounted) {
    if (now - lastTouch1Time >= debounceInterval) {
      touchCount1++;
      lastTouch1Time = now;
      Serial.println("Touched D5 (Debounced)");
      sendJsonData();
    }
    touch1WasCounted = true;
  }
  if (touch1State == LOW) {
    touch1WasCounted = false;
  }

  bool touch2State = digitalRead(touchPin2);
  if (touch2State == HIGH && !touch2WasCounted) {
    if (now - lastTouch2Time >= debounceInterval) {
      touchCount2++;
      lastTouch2Time = now;
      Serial.println("Touched D18 (Debounced)");
      sendJsonData();
    }
    touch2WasCounted = true;
  }
  if (touch2State == LOW) {
    touch2WasCounted = false;
  }

if (now - lastSpeedCheck >= 1000) {
  int currentRevolutions = speedPulseCount;

  float wheelCircumference = PI * WHEEL_DIAMETER;  
  float speed_mps = currentRevolutions * wheelCircumference;  
  float speed_kmph = speed_mps * 3.6;  

  lastSpeed = (int)speed_kmph;  
  sendJsonData();

  speedPulseCount = 0;
  lastSpeedCheck = now;
}

}
