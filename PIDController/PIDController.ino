#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "bearingCalc.h"

float latDest = 0;
float lonDest = 0;

const float Kp = .4f; //2.0
const float Ki = 2.0f; //4.0
const float Kd = 1.0f; //100.00

float lastError = 0;
const float oldValWeight = 0.99f;
const float newValWeight = 0.01f;


int rightMotorSpeed;
int leftMotorSpeed;

// Motorer:
const int motor1A = 2;
const int motor1B = 4;
const int motor2A = 6;
const int motor2B = 7;

//PVM  setting
const int PWM1 = 3;
const int PWM2 = 5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupThis();
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
}

void loop() {

  while (!gpsSignal()) {};
  forward(350);
  Serial.println("end");

}

//Motor forward start

void forward(int allSpeed) {

  calculateWeight(getPID(getErrorMargin(latDest, lonDest)), allSpeed);
  //Write to digital pin, for rotating motors forward
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);

  straightForward(allSpeed);

  //Write the calculated speed to analog pins.
  analogWrite(PWM1, rightMotorSpeed);
  analogWrite(PWM2, leftMotorSpeed);

  delay(3000);
}

void straightForward(int allSpeed){

  analogWrite(PWM1, allSpeed / 2);
  analogWrite(PWM2, allSpeed / 2);

delay(2000);
  
  }


//Motor farword end

//Calculate motor weight

void calculateWeight(float pid, int allSpeed) {
  Serial.println(pid);
  float straight = 0.5f;
  float smallTurnA = 0.6f;
  float smallTurnB = 0.4f;
  float largeTurnA = 0.8f;
  float largeTurnB = 0.2f;
  float rightMotorWeight;
  float leftMotorWeight;


  if (pid > 5.0f && pid < 10.0f) {
    Serial.println("1");
    //drej lidt
    rightMotorWeight = smallTurnA;
    leftMotorWeight = smallTurnB;
  }
  else if (pid > 10.0f) {
    Serial.println("2");
    //drej meget
    rightMotorWeight = largeTurnA;
    leftMotorWeight = largeTurnB;
  }
  else if (pid > -5.0f && pid < -10.0f) {
    Serial.println("3");
    //drej lidt
    rightMotorWeight = smallTurnB;
    leftMotorWeight = smallTurnA;
  }
  else if (pid > -10.0f) {
    Serial.println("4");
    //drej meget
    rightMotorWeight = largeTurnB;
    leftMotorWeight = largeTurnA;
  }
  else { //limits: (pid<5.0f && pid>-5.0f)
    Serial.println("5");
    //lige meget
    rightMotorWeight = straight;
    leftMotorWeight = straight;
  }


  rightMotorSpeed = allSpeed * rightMotorWeight;
  leftMotorSpeed = allSpeed * leftMotorWeight;
  if (rightMotorSpeed > 255) {
    rightMotorSpeed = 255;
  }

  if (leftMotorSpeed > 255) {
    leftMotorSpeed = 255;
  }
  if (rightMotorSpeed < 100) {
    rightMotorSpeed = 100;
  }

  if (leftMotorSpeed < 100) {
    leftMotorSpeed = 100;
  }



}

//PID START

float getProportional(float error) {
  return Kp * error;
}

float getIntegral(float error) {
  return Ki * ((lastError * oldValWeight) + (error * newValWeight));
}

float getDerivative(float lastError, float error) {
  return Kd * (error - lastError);
}

float getPID(float error) {
  //Step 1
  float PIDVal = getProportional(error) + getIntegral(error);
  //Step 2
  //float PIDVal = getProportional(error) + getIntegral(error) + getDerivative(lastError, error);
  lastError = error;

  return PIDVal;
}

//PID END

