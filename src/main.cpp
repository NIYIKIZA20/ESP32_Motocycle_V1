#include <Arduino.h>
#include "BluetoothSerial.h"
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

BluetoothSerial SerialBT;
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 cols x 2 rows

const int button1Pin = 12;
const int button2Pin = 27;
const int speedSensorPin = 34;
const int touchPin1 = 5;
const int touchPin2 = 18;

#define WHEEL_DIAMETER 0.6  // meters
#define MAX_SPEED 60.0      // Max speed for full bar

bool lastButton1State;
bool lastButton2State;
bool touch1WasCounted = false;
bool touch2WasCounted = false;

int brakeCount1 = 0;
int brakeCount2 = 0;
int touchCount1 = 0;
int touchCount2 = 0;

volatile int speedPulseCount = 0;
float lastSpeed = 0.0;
unsigned long lastSpeedCheck = 0;

unsigned long lastBrake1Time = 0;
unsigned long lastBrake2Time = 0;
unsigned long lastTouch1Time = 0;
unsigned long lastTouch2Time = 0;

const unsigned long debounceInterval = 2000;
const char* deviceID = "ESP1";

byte barChars[5][8] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F},
  {0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x1F},
  {0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F},
  {0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F,0x1F},
  {0x00,0x00,0x00,0x1F,0x1F,0x1F,0x1F,0x1F}
};

void IRAM_ATTR onSpeedSensor() {
  speedPulseCount++;
}

void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("FR:");
  lcd.print(brakeCount1);
  lcd.print(" FL:");
  lcd.print(brakeCount2);
  lcd.print(" ");

  lcd.setCursor(0, 1);
  lcd.print("T1:");
  lcd.print(touchCount1);
  lcd.print(" T2:");
  lcd.print(touchCount2);

  delay(300);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("SPD:");
  lcd.print(lastSpeed, 1);
  lcd.print("km/h  ");

  lcd.setCursor(0, 1);

  int blocks = 10;
  float ratio = min(lastSpeed / MAX_SPEED, 1.0);
  int full = (int)(ratio * blocks);
  int part = (int)((ratio * blocks - full) * 5);

  for (int i = 0; i < full; i++) lcd.write(255);
  if (full < blocks) {
    lcd.write(part);
    for (int i = full + 1; i < blocks; i++) lcd.print(" ");
  }

  delay(700);
  lcd.clear();
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

  for (int i = 0; i < 5; i++) lcd.createChar(i, barChars[i]);

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

  if (button1State == HIGH && lastButton1State == LOW && now - lastBrake1Time >= debounceInterval) {
    brakeCount1++;
    lastBrake1Time = now;
    sendJsonData();
  }
  lastButton1State = button1State;

  if (button2State == HIGH && lastButton2State == LOW && now - lastBrake2Time >= debounceInterval) {
    brakeCount2++;
    lastBrake2Time = now;
    sendJsonData();
  }
  lastButton2State = button2State;

  // === Touch Sensors ===
  bool touch1State = digitalRead(touchPin1);
  if (touch1State == HIGH && !touch1WasCounted && now - lastTouch1Time >= debounceInterval) {
    touchCount1++;
    lastTouch1Time = now;
    sendJsonData();
    touch1WasCounted = true;
  }
  if (touch1State == LOW) touch1WasCounted = false;

  bool touch2State = digitalRead(touchPin2);
  if (touch2State == HIGH && !touch2WasCounted && now - lastTouch2Time >= debounceInterval) {
    touchCount2++;
    lastTouch2Time = now;
    sendJsonData();
    touch2WasCounted = true;
  }
  if (touch2State == LOW) touch2WasCounted = false;

  // === Speed Calculation Every 1s ===
  if (now - lastSpeedCheck >= 1000) {
    int currentRevs = speedPulseCount;
    float circumference = PI * WHEEL_DIAMETER;
    float speed_mps = currentRevs * circumference;
    lastSpeed = speed_mps * 3.6;

    speedPulseCount = 0;
    lastSpeedCheck = now;
    sendJsonData();
  }
}
