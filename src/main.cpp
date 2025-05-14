#include <Arduino.h>
#include "BluetoothSerial.h"
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

BluetoothSerial SerialBT;
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int button1Pin = 12; //right foot
const int button2Pin = 27; // leftfoot
const int proximityPin = 23;  // From proximity sensor
const int touchPin1 = 5;    //brake foot
const int touchPin2 = 18;  // brake hand

#define WHEEL_DIAMETER 0.6
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)

bool lastButton1State;
bool lastButton2State;
bool lastProximityState = HIGH; 
bool touch1WasCounted = false;
bool touch2WasCounted = false;

int brakeCount1 = 0;
int brakeCount2 = 0;
int touchCount1 = 0;
int touchCount2 = 0;

volatile int revolutionCount = 0;
float currentSpeed = 0.0;
unsigned long lastSpeedCheck = 0;

unsigned long lastBrake1Time = 0;
unsigned long lastBrake2Time = 0;
unsigned long lastTouch1Time = 0;
unsigned long lastTouch2Time = 0;

const unsigned long debounceInterval = 3000;
const char* deviceID = "ESP1";

void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("F.R:");
  lcd.print(brakeCount1);
  lcd.print(" F.L:");
  lcd.print(brakeCount2);
  lcd.print("  ");

  lcd.setCursor(0, 1);
  lcd.print("SPD:");
  lcd.print(currentSpeed, 1);
  lcd.print(" B.F:");
  lcd.print(touchCount1);
  lcd.print(" B.H:");
  lcd.print(touchCount2);
}


void sendJsonData() {
  StaticJsonDocument<256> doc;
  doc["device"] = deviceID;
  doc["timestamp"] = millis();
  doc["rightFoot"] = brakeCount1;
  doc["leftFoot"] = brakeCount2;
  doc["footBrake"] = touchCount1;
  doc["handBrake"] = touchCount2;
  doc["speed"] = currentSpeed;

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
  pinMode(proximityPin, INPUT_PULLUP);  // Assumes proximity sensor pulls LOW when detecting metal
  pinMode(touchPin1, INPUT);
  pinMode(touchPin2, INPUT);

  lastButton1State = digitalRead(button1Pin);
  lastButton2State = digitalRead(button2Pin);

  delay(1000);
  lcd.clear();
  updateLCD();
}

void loop() {
  unsigned long now = millis();

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

  bool touch1State = digitalRead(touchPin1);
  if (touch1State == HIGH && !touch1WasCounted && now - lastTouch1Time >= debounceInterval) {
    touchCount1++;
    lastTouch1Time = now;
    touch1WasCounted = true;
    sendJsonData();
  } else if (touch1State == LOW) {
    touch1WasCounted = false;
  }

  bool touch2State = digitalRead(touchPin2);
  if (touch2State == HIGH && !touch2WasCounted && now - lastTouch2Time >= debounceInterval) {
    touchCount2++;
    lastTouch2Time = now;
    touch2WasCounted = true;
    sendJsonData();
  } else if (touch2State == LOW) {
    touch2WasCounted = false;
  }

  bool currentProximityState = digitalRead(proximityPin);
  if (currentProximityState == LOW && lastProximityState == HIGH) {
    revolutionCount++;
    Serial.print("Revolution Detected! Count = ");
    Serial.println(revolutionCount);
  }
  lastProximityState = currentProximityState;

 
  if (now - lastSpeedCheck >= 1000) {
    int revs = revolutionCount;
    revolutionCount = 0;

    float distance_m = revs * WHEEL_CIRCUMFERENCE; // meters per second
    currentSpeed = (distance_m * 3600.0) / 1000.0;  // km/h

    currentSpeed = round(currentSpeed * 10) / 10.0; // Round to 1 decimal
    lastSpeedCheck = now;
    // Serial.println(revs);
    // Serial.println(currentSpeed);

    sendJsonData(); // Send speed and other data
  }
}
