#include <Servo.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

const int LeftMotorPin = A0;
const int RightMotorPin = A1;
const int RCPinForward = A2;
const int RCPinTurn = A3;


float Speed;
float Rotation;
float LeftMotorSpeed;
float RightMotorSpeed;

Servo LeftMotor;
Servo RightMotor;

typedef struct {
  volatile unsigned long startTime;
  volatile unsigned long pulseWidth;
} PWMInput;

PWMInput inputForward;
PWMInput inputTurn;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  LeftMotor.attach(LeftMotorPin);
  LeftMotor.writeMicroseconds(1500);
  RightMotor.attach(RightMotorPin);
  RightMotor.writeMicroseconds(1500);
  delay(5000);
  Serial.println("Controllers Calibrated");

  pinMode(RCPinForward, INPUT);
  pinMode(RCPinTurn, INPUT);

  attachInterrupt(digitalPinToInterrupt(RCPinForward), handleInterruptForward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinTurn), handleInterruptTurn, CHANGE);
  
}



void loop() {

  noInterrupts();
  unsigned long pulseWidthForward = inputForward.pulseWidth;
  unsigned long pulseWidthTurn = inputTurn.pulseWidth;

  Serial.println(pulseWidthForward);
   

  //set left motor speed
  LeftMotorSpeed = Speed + Rotation;
  if(LeftMotorSpeed > 1){
    LeftMotorSpeed = 1;
  }
  if(LeftMotorSpeed < -1){
    LeftMotorSpeed = -1;
  }
  //set right motor
  RightMotorSpeed = Speed + Rotation;
  if(RightMotorSpeed > 1){
    RightMotorSpeed = 1;
  }
  if(RightMotorSpeed < -1){
    RightMotorSpeed = -1;
  }

  LeftMotor.writeMicroseconds(1500);
  RightMotor.writeMicroseconds(1500 + RightMotorSpeed * 500);
  }

void handleInterruptForward() {
  unsigned long currentTime = micros();
  if (digitalRead(RCPinForward) == HIGH) {
    inputForward.startTime = currentTime;
  } else {
    inputForward.pulseWidth = currentTime - inputForward.startTime;
  }
}
void handleInterruptTurn() {
  unsigned long currentTime = micros();
  if (digitalRead(RCPinTurn) == HIGH) {
    inputTurn.startTime = currentTime;
  } else {
    inputTurn.pulseWidth = currentTime - inputTurn.startTime;
  }
}

