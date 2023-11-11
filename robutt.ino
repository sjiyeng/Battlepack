#include <Servo.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

const int LeftMotorPin = A0;
const int RightMotorPin = A1;
const int RCPinForward = 2;
const int RCPinTurn = 3;

volatile long ForwardStartTime = 0;
volatile long ForwardCurrentTime = 0;
volatile long ForwardPulses = 0;
int ForwardPulseWidth = 0;

volatile long TurnStartTime = 0;
volatile long TurnCurrentTime = 0;
volatile long TurnPulses = 0;
int TurnPulseWidth = 0;

float Speed;
float Turn;
float LeftMotorSpeed;
float RightMotorSpeed;

bool debugPWM = false;
bool debugMotorOutputs = false;
bool debugJoysticks = false;

Servo LeftMotor;
Servo RightMotor;



void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:


  //setup pulse width stuff
  pinMode(RCPinForward, INPUT_PULLUP);
  pinMode(RCPinTurn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt (RCPinForward), ForwardPulseTimer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinTurn), TurnPulseTimer, CHANGE);
  
  //setup motor stuff and calibrate controllers
  LeftMotor.attach(LeftMotorPin);
  LeftMotor.writeMicroseconds(1500);
  RightMotor.attach(RightMotorPin);
  RightMotor.writeMicroseconds(1500);
  Serial.println("Starting Calibration");
  Serial.println("5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  Serial.println("Controllers Calibrated");

  
}



void loop() { 

  //calculate pulse widths
  if(ForwardPulses < 2050){
    ForwardPulseWidth = ForwardPulses;
  }
  if(TurnPulses < 2050){
    TurnPulseWidth = TurnPulses;
  }



  //calculate speed joystick value based off of PWM input
  Speed = (ForwardPulseWidth-1500) / 500.;
  //calculate turn joystick value based off of PWM input
  Turn = (TurnPulseWidth-1500) / 500.;
  //cap joystick values from -1 to 1
  if(Speed > 1){
    Speed = 1;
  }
  if(Speed < -1){
    Speed = -1;
  }
  if(Turn > 1){
    Turn = 1;
  }
  if(Turn < -1){
    Turn = -1;
  }

  //middle deadbands
  if(Speed < .04 && Speed > -.04){
    Speed = 0;
  }
  if(Turn < .04 && Turn > -.04){
    Turn = 0;
  }

  //top end deadbands
  if(Speed > .95){
    Speed = 1;
  }
  if(Turn > .95){
    Turn = 1;
  }

  //bottom end deadbands

  if(Speed < -.95){
    Speed = -1;
  }
  if(Turn < -.95){
    Turn = -1;
  }

  

  //set left motor speed
  LeftMotorSpeed = Speed + Turn;
  if(LeftMotorSpeed > 1){
    LeftMotorSpeed = 1;
  }
  if(LeftMotorSpeed < -1){
    LeftMotorSpeed = -1;
  }
  //set right motor
  RightMotorSpeed = (-(Speed - Turn));
  if(RightMotorSpeed > 1){
    RightMotorSpeed = 1;
  }
  if(RightMotorSpeed < -1){
    RightMotorSpeed = -1;
  }

  LeftMotor.writeMicroseconds(1500 + LeftMotorSpeed * 500);
  RightMotor.writeMicroseconds(1500 + RightMotorSpeed * 500);
  
  //debug stuff
  if(debugPWM == true){
  Serial.print("Forward:");
  Serial.print(ForwardPulseWidth);
  Serial.print("|");
  Serial.print("Turn:");
  Serial.println(TurnPulseWidth);
  }

  if(debugMotorOutputs == true){
  Serial.print("Left Motor Speed:");
  Serial.print(LeftMotorSpeed);
  Serial.print("|");
  Serial.print("Right Motor Speed:");
  Serial.println(RightMotorSpeed);
  }

  if(debugJoysticks == true){
  Serial.print("Left:");
  Serial.print(Speed);
  Serial.print("|");
  Serial.print("Right:");
  Serial.println(Turn);
  }
  
  }

  void ForwardPulseTimer(){
    ForwardCurrentTime = micros();
    if( ForwardCurrentTime > ForwardStartTime){
      ForwardPulses = ForwardCurrentTime - ForwardStartTime;
      ForwardStartTime = ForwardCurrentTime;
    }
  }

  void TurnPulseTimer(){
    TurnCurrentTime = micros();
    if(TurnCurrentTime > TurnStartTime){
      TurnPulses = TurnCurrentTime - TurnStartTime;
      TurnStartTime = TurnCurrentTime;
    }
  }

