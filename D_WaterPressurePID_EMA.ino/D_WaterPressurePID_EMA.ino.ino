#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//------------------------------------------------------------------------------------
//---16x2 LCD (I2C)---
//------------------------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

//------------------------------------------------------------------------------------
//---DC Motor Driver L293D---
//------------------------------------------------------------------------------------
//  Motor A (Left)
#define forwardPin_A   8
#define backwardPin_A  9
#define enablePin_A   10

//  Motor B (Right)
#define forwardPin_B  12
#define backwardPin_B 13
#define enablePin_B   11

//------------------------------------------------------------------------------------
//---Potentiometers---
//------------------------------------------------------------------------------------
#define potPin A0
int potVal = 0;

#define potKp A1
#define potKi A2
#define potKd A3

//------------------------------------------------------------------------------------
//---Water Pressure Sensor---
//------------------------------------------------------------------------------------
#define pressurePin A6

//------------------------------------------------------------------------------------
//---PID Global Variables---
//------------------------------------------------------------------------------------
long previousTime = 0;
float errorSum = 0;
float previousError = 0;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//------------------------------------------------------------------------------------
//--- Kalman Filter ---
//------------------------------------------------------------------------------------

// --- Kalman Filter Parameters ---
float Q = 0.05;// Process noise covariance (smoothness)
float R = 15.0;// Measurement noise covariance (noise level)
float P = 1.0;// Estimation error covariance
float X = 0.0;// Filtered value

float kalmanFilter(float measurement) {
  // Prediction update
  P = P + Q;

  // Measurement update
  float K = P / (P + R);
  X = X + K * (measurement - X);
  P = (1 - K) * P;

  return X;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~SETUP~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // LCD Init
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Start");

  // Motor Driver Pins
  pinMode(forwardPin_A, OUTPUT);
  pinMode(backwardPin_A, OUTPUT);
  pinMode(enablePin_A, OUTPUT);

  pinMode(forwardPin_B, OUTPUT);
  pinMode(backwardPin_B, OUTPUT);
  pinMode(enablePin_B, OUTPUT);

  pinMode(pressurePin, INPUT);

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~LOOP~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() {

  long currentTime = micros();
  float deltaTime = ((float)(currentTime - previousTime)) / 1e6;

  //---Measure Pressure---
  int rawValue = analogRead(pressurePin);
  float voltage = rawValue * (5.0 / 1024.0);

  float offset = 0.5;
  float voltage_comp = voltage - offset;
  float pressure_kPa = (voltage_comp) * 250;

  /*
  float pressureMeasured_raw = pressure_kPa; 
  if(pressureMeasured_raw <= 0.0){
    pressureMeasured_raw = 0.0;
  }

  // Initialize EMA filter variables
  float alpha = 0.60;          // smoothing factor (0.0–1.0)

  // ---- Apply EMA Filter ----
  float pressureEMA = alpha * pressureMeasured_raw + (1 - alpha) * pressureEMA;

  // Use filtered value as the real measurement
  float pressureMeasured = pressureEMA;
  */

  float pressureRaw = pressure_kPa;
  if(pressureRaw <= 0.0){
    pressureRaw = 0.0;
  }
  float pressureMeasured = kalmanFilter(pressureRaw);

// -------- VARIABLES --------
float pressureDesired;
//float pressureMeasured;

float error = 0;
float previousError = 0;
float errorSum = 0;

float Kp = 0;
float Ki = 0;
float Kd = 0;

float Ku = 0;
float Pu = 0;

float feedForward = 0.0;
//float deltaTime = 0.1;

bool autoTune = true;

float KpStep = 0.05;   // how fast Kp increases
float oscillationAmplitude;

unsigned long lastCrossTime = 0;
//unsigned long currentTime = 0;

float lastMeasured = 0;

int potVal;

// -------- SETPOINT --------
potVal = analogRead(potPin);
pressureDesired = map(potVal, 0, 1024, 0, 61);

// -------- ERROR --------
error = pressureDesired - pressureMeasured;


// =======================================
// AUTO TUNE (ZIEGLER-NICHOLS METHOD)
// =======================================

if (autoTune)
{
    Ki = 0;
    Kd = 0;

    // Gradually increase proportional gain
    Kp += KpStep;

    // Detect oscillation crossing
    if ((lastMeasured < pressureDesired && pressureMeasured >= pressureDesired) ||
        (lastMeasured > pressureDesired && pressureMeasured <= pressureDesired))
    {
        currentTime = millis();

        if (lastCrossTime != 0)
        {
            Pu = (currentTime - lastCrossTime) / 1000.0; // oscillation period
        }

        lastCrossTime = currentTime;
    }

    // Detect sustained oscillation
    oscillationAmplitude = abs(pressureMeasured - pressureDesired);

    if (oscillationAmplitude > 2.0 && Pu > 0)
    {
        Ku = Kp;

        // Ziegler–Nichols PID formulas
        Kp = 0.6 * Ku;
        Ki = 1.2 * Ku / Pu;
        Kd = 0.075 * Ku * Pu;

        autoTune = false;   // stop tuning
    }
}

lastMeasured = pressureMeasured;


// =======================================
// PID CONTROLLER
// =======================================

if (error < 1)
    errorSum = 0;
else
    errorSum += error * deltaTime;

float dErr = (error - previousError) / deltaTime;

float PIDinput = Kp * error + Ki * errorSum + Kd * dErr;

// Saturation
if (PIDinput > 255) PIDinput = 255;
if (PIDinput < 0) PIDinput = 0;

previousError = error;

  // Motor Direction
  digitalWrite(forwardPin_A, HIGH);
  digitalWrite(backwardPin_A, LOW);

  digitalWrite(forwardPin_B, HIGH);
  digitalWrite(backwardPin_B, LOW);

  // Actuate motors
  float controlInputU_A = PIDinput + feedForward;
  float controlInputU_B = -PIDinput + feedForward;

  analogWrite(enablePin_A, controlInputU_A);
  analogWrite(enablePin_B, controlInputU_B);

  //---Display on LCD---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("M:");
  lcd.print(pressureMeasured, 0);
  lcd.print(" T:");
  lcd.print(pressureDesired, 0);
  lcd.print(" E:");
  lcd.print(error, 0);

  lcd.setCursor(0, 1);
  lcd.print("U:");
  lcd.print((int)controlInputU_A);
  lcd.print(" P:");
  lcd.print(Kp, 0);
  lcd.print(" I:");
  lcd.print(Ki, 0);
  lcd.print(" D:");
  lcd.print(Kd, 0);

  // Serial Debug
  Serial.print("pressureMeasured:");
  Serial.print(pressureMeasured);
  Serial.print(", pressureRaw:");
  Serial.print(pressureRaw);
  Serial.print(", pressureDesired:");
  Serial.println(pressureDesired);

  previousError = error;
  previousTime = currentTime;
  //delay(10);
}