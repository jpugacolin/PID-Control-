#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// Specify the LCD Pinout

const int rs = 3;
const int en = 4;
const int d4 = 5;
const int d5 = 6;
const int d6 = 7;
const int d7 = 8;

int cols = 16;
const int rows = 2;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

MPU6050 mpu6050(Wire);

float offsetAngleY = 0.0f;
uint32_t timer = 0;

/*
Motor A truth table
ENA	IN1	IN2	 Description
0	  N/A	N/A	 Motor A is off
1	  0	  0	   Motor A is stopped (brakes)
1	  0	  1	   Motor A is on and turning backwards
1	  1	  0	   Motor A is on and turning forwards
1	  1	  1	   Motor A is stopped (brakes)
*/

#define MIN_SPEED_MOTOR_A 0 // 70
#define MAX_SPEED_MOTOR_A 255
#define MIN_SPEED_MOTOR_B 0 // 70
#define MAX_SPEED_MOTOR_B 255
#define MIN_SPEED_MOTOR 55
#define MAX_SPEED_MOTOR 150

// Initializing all of our pins for the L298N driver module
const int ENA = 44;
const int IN1 = 23;
const int IN2 = 25;
const int ENB = 46;
const int IN3 = 27;
const int IN4 = 29;

// Initializing motor speed variables
int motorSpeed_A = 0;
int motorSpeed_B = 0;
int motorDirection = 0;

// Initialize controller parameters
float desiredSetpoint = 0.0f;
float error = 0.0f;

float Kp = 1.25f;
float proportional = 0.0f;
unsigned long lastTime = 0.0f;
float output = 55.0f;
bool controllerSwitch = 1;

unsigned long timingInterval = 25;
float acceptableRange = 0.0f;

float kd = 0.025f;
float Kd = kd / (timingInterval / 1000.0f);
float derivative = 0.0f;
float lastError = 0.0f;

// float k = 1.0f;

// NOTES
// Startup motor 50
// Can go down to 35
// But must start at 50
// TUrning direction does not work for both motors equally at 30
// Right motor spins faster than left motor
// 70 min turn on for left motor
// 70 min turn on for right motor

// Initializing Liquid Crystal Display Pinout

void displayDataToLCD(uint8_t column, const char *str, int decimalPlacement, float data, float lastData);

void setup()
{
  // Initialize Serial communication
  Serial.begin(115200);

  lcd.begin(16, 2);

  // while (!Serial)
  //   ; // Leonardo: wait for serial monitor

  // Serial.println("Serial port ready.");

  // Setting MCU pins to OUTPUT for Motor A
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // Initialize motor A to be off
  digitalWrite(ENA, LOW);

  // Setting MCU pins to OUTPUT for Motor B
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  // Initialize motor B to be off
  digitalWrite(ENB, LOW);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);

  offsetAngleY = mpu6050.getAngleY();

  for (int i = 0; i < 3; i++)
  {
    static bool blink = 1;
    static bool ledStatus = 1;
    static int counter = 0;

    Serial.print("Countdown time = ");
    Serial.println(3 - i);

    counter = 3 - i;

    while (blink)
    {
      digitalWrite(LED_BUILTIN, ledStatus);
      counter--;
      delay(500);
      if (counter <= 0)
      {
        blink = 0;
        ledStatus = 1;
      }
      else
      {
        ledStatus = !ledStatus;
      }
    }

    blink = 1;
  }
}

void loop()
{

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  if (elapsedTime >= timingInterval)
  {
    mpu6050.update();

    // Compute
    float actual = mpu6050.getAngleY() - offsetAngleY;

    // float actual = (float)(kalAngleY)-offsetAngleY_kalman;

    // sp = 10 
    // act = 5 
    // err = 10 - 5 = 5 

    // sp = -10 
    // act = 5 
    // errpr -10 - 5 = -15 

    error = desiredSetpoint - actual;
    proportional = Kp * error;
    derivative = Kd * (error - lastError);

    output = output + proportional + derivative;
    output = constrain(output, MIN_SPEED_MOTOR, MAX_SPEED_MOTOR);

    lastError = error;

    if (actual > acceptableRange)
    {
      // If actual is > 0 then leaning back , move backwards

      motorDirection = 0;

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

      // k = 0.8;
    }
    else if (actual < -1 * acceptableRange)
    {
      // If actual is < 0 then leaning forward, move forward

      motorDirection = 1;

      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);

      // k = 0.9;
    }
    else
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(ENA, 0);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      digitalWrite(ENB, 0);
    }

    // static float lastActual = 0.0f;
    // static float lastDesiredSetpoint = 0.0f;
    // displayDataToLCD(0, "pv=", 100, actual, lastActual);
    // displayDataToLCD(9, "sp=", 100, desiredSetpoint, lastDesiredSetpoint);

    int decimalPlacement = 100;
    int truncatedActual = actual * decimalPlacement;
    int truncatedDesiredSetpoint = desiredSetpoint * decimalPlacement;
    int truncatedError = error * decimalPlacement;
    int truncatedOutput = output * decimalPlacement; 


    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("pv=");
    lcd.print(truncatedActual / decimalPlacement);
    lcd.print(".");
    lcd.print(abs(truncatedActual) % decimalPlacement); // Display the decimal digit

    lcd.setCursor(0, 1);
    lcd.print("sp=");
    lcd.print(truncatedDesiredSetpoint / decimalPlacement);
    lcd.print(".");
    lcd.print(abs(truncatedDesiredSetpoint) % decimalPlacement); // Display the decimal digit

    lcd.setCursor(8, 0);
    lcd.print("err=");
    lcd.print(truncatedError/ decimalPlacement);
    lcd.print(".");
    lcd.print(abs(truncatedError) % decimalPlacement); // Display the decimal digit

    lcd.setCursor(8, 1);
    lcd.print("out=");
    lcd.print(truncatedOutput / decimalPlacement);
    lcd.print(".");
    lcd.print(abs(truncatedOutput) % decimalPlacement); // Display the decimal digit

    // Serial.print("sp=");
    // Serial.print(desiredSetpoint);
    // Serial.print("\t");

    // Serial.print("act=");
    // Serial.print(actual);
    // Serial.print("\t");

    // Serial.print("dir=");
    // Serial.print(motorDirection);
    // Serial.print("\t");

    // Serial.print("err=");
    // Serial.print(error);
    // Serial.print("\t");

    // Serial.print("prop=");
    // Serial.print(proportional);
    // Serial.print("\t");

    // Serial.print("der=");
    // Serial.print(derivative);
    // Serial.print("\t");

    // Serial.print("out=");
    // Serial.print(output);
    // Serial.println();

    analogWrite(ENA, static_cast<int>(abs(output)));          // l and o (left motor)
    analogWrite(ENB, static_cast<int>(abs(output * 0.925f))); // right motor

    lastTime = millis();
  }
}

void displayDataToLCD(uint8_t column, const char *str, int decimalPlacement, float data, float lastData)
{

  float number = data;
  int truncated = number * decimalPlacement;
  lcd.setCursor(0, column);                     // Set the cursor to the second line
  lcd.print(str);                               // Print a label
  lcd.print(truncated / decimalPlacement);      // Display the truncated number
  lcd.print(".");                               // Print the decimal point
  lcd.print(abs(truncated) % decimalPlacement); // Display the decimal digit
}
