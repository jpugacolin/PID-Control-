#include <Wire.h>
#include <LiquidCrystal.h>
#include <MPU6050.h>
#include <NewPing.h>

#define TRIGGER_PIN  12
#define ECHO_PIN     13
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
MPU6050 mpu;
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

int motor1 = 9;
int motor2 = 10;
int motor3 = 11;
int motor4 = 8;
int motor_speed = 0;
int motor_speed_offset = 0;
int tilt_angle = 0;
int tilt_angle_offset = 0;
int max_motor_speed = 255;
int min_motor_speed = 0;
int max_tilt_angle = 45;
int min_tilt_angle = -45;
int target_tilt_angle = 0;
int ultrasonic_distance = 0;

void setup() {
  Serial.begin(9600);
  mpu.initialize();
  lcd.begin(16, 2);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);
}

void loop() {
  // Read MPU6050 sensor data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate tilt angle
  int acc_x = ax / 16384.0;
  int acc_z = az / 16384.0;
  tilt_angle = atan2(acc_x, acc_z) * 180.0 / PI;

  // Calculate motor speed based on tilt angle
  motor_speed = tilt_angle - tilt_angle_offset;
  motor_speed = map(motor_speed, min_tilt_angle, max_tilt_angle, min_motor_speed, max_motor_speed);
  motor_speed += motor_speed_offset;
  motor_speed = constrain(motor_speed, min_motor_speed, max_motor_speed);

  // Adjust motor speeds
  if (motor_speed > 0) {
    analogWrite(motor1, motor_speed);
    analogWrite(motor2, 0);
    analogWrite(motor3, 0);
    analogWrite(motor4, motor_speed);
  } else if (motor_speed < 0) {
    analogWrite(motor1, 0);
    analogWrite(motor2, abs(motor_speed));
    analogWrite(motor3, abs(motor_speed));
    analogWrite(motor4, 0);
  } else {
    analogWrite(motor1, 0);
    analogWrite(motor2, 0);
    analogWrite(motor3, 0);
    analogWrite
