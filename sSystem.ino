#include <Wire.h>
#include <LiquidCrystal.h>
#include <MPU6050_tockn.h>

const int trigPin = 9; // Ultrasonic sensor Trig pin
const int echoPin = 10; // Ultrasonic sensor Echo pin
const int IN1 = 2; // L298N IN1
const int IN2 = 3; // L298N IN2
const int IN3 = 4; // L298N IN3
const int IN4 = 5; // L298N IN4
const int ENA = 6; // L298N ENA
const int ENB = 7; // L298N ENB
const int ledR = 11; // LED Red pin
const int ledG = 12; // LED Green pin
const int ledB = 13; // LED Blue pin

const float Kp = 40;
const float Kd = 20;

LiquidCrystal lcd(8, A0, A1, A2, A3, A4, A5);

MPU6050 mpu6050(Wire);

float angle = 0, accAngle = 0, gyroRate = 0, lastError = 0, output = 0;
long distance = 0;
unsigned long time1 = 0, time2 = 0, cycleTime = 0, lastTime = 0;

void setup() {
  Serial.begin(9600);
  mpu6050.begin();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  lcd.begin(16, 2);
  lcd.print("Self Balancing Bot");
  delay(2000);
}

void loop() {
  time1 = micros();

  // Ultrasonic sensor measurement
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  distance = pulseIn(echoPin, HIGH) * 0.034 / 2;

  // MPU6050 angle measurement
  angle = 0.98 * (angle + gyroRate * cycleTime / 1000000) + 0.02 * accAngle;
  gyroRate = mpu6050.getGyroAngleZ() / 131.0;
  accAngle = atan2(-mpu6050.getAccY(), mpu6050.getAccZ()) * 180 / PI;
  output = Kp * angle + Kd * (angle - lastError) / cycleTime;
  lastError = angle;

  // Motor control
  if (distance > 20) {
    analogWrite(ENA, abs(output));
    analogWrite(ENB, abs(output));
    if (output > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      digitalWrite(ledR, LOW);
      digitalWrite(ledG, HIGH);
      digitalWrite(ledB, LOW);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      digitalWrite(ledR, HIGH);
      digitalWrite(ledG,
