#include <Servo.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

RF24 radio(9,10 );
const byte address[10] = "ADDRESS01";

const int LeftMotorPin = 0;
const int RightMotorPin = 1;
float Speed;
float Rotation;
float LeftMotorSpeed;
float RightMotorSpeed;

Servo LeftMotor;
Servo RightMotor;


void setup() {
  // put your setup code here, to run once:
  LeftMotor.attach(LeftMotorPin);
  RightMotor.attach(RightMotorPin);

  radio.begin();
  radio.openReadingPipe(0, address); 
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

}

void loop() {

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

  LeftMotor.writeMicroseconds(1500 + LeftMotorSpeed * 500);
  RightMotor.writeMicroseconds(1500 + RightMotorSpeed * 500);
  }



