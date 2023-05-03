#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <MPU6050.h>

MPU6050 mpu;
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// motor driver pins
#define ENA 6
#define ENB 5
#define IN1 22
#define IN2 23
#define IN3 24
#define IN4 25

// ultrasonic sensor pins
#define trigPin 9
#define echoPin 10

// PID constants
float kp = 12.5;
float ki = 0.5;
float kd = 0.5;
float error = 0;
float integral = 0;
float derivative = 0;
float previous_error = 0;

// motor speed variables
int motorSpeedA = 0;
int motorSpeedB = 0;

// ultrasonic sensor variables
long duration;
int distance;

// time interval variables
unsigned long currentMillis;
unsigned long previousMillis = 0;
const unsigned long interval = 100; // interval in milliseconds

// RGB LED variables
int redPin = 13;
int greenPin = 7;
int bluePin = 8;

void setup() {
  // initialize LCD display
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Self-Balancing");
  lcd.setCursor(0, 1);
  lcd.print("Robot");

  // initialize MPU6050 sensor
  Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  // initialize motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // initialize RGB LED pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop() {
  currentMillis = millis();

  // read MPU6050 sensor
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // calculate pitch angle
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;

  // calculate PID output
  error = pitch;
  integral += error * interval / 1000.0;
  derivative = (error - previous_error) / (interval / 1000.0);
  float pidOutput = kp * error + ki * integral + kd * derivative;
  previous_error = error;

  // calculate motor speeds
  motorSpeedA = (int)(pidOutput + 0.5);
  motorSpeedB = (int)(pidOutput + 0.5);

  // limit motor speeds to 255
  if (motorSpeedA > 255) {
    motorSpeedA = 255;
  }
  if (motorSpeedB > 255) {
    motorSpeedB = 255;
  }

  // set motor directions and speeds
  if (motorSpeedA >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motorSpeedA);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite
