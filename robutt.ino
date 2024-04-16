#include <Servo.h>

const unsigned long TIME_THRESHOLD = 5000; // 5 seconds in milliseconds
unsigned long previousMillis = 0;
int previousPulseWidth = 0;
bool FAIL = false;

const int LeftMotorPin = A1;
const int RightMotorPin = A0;
const int WeaponMotorPin = A2;
const int RCPinForward = 8;
const int RCPinTurn = 10;
const int RCPinWeapon = 7;


volatile long ForwardStartTime = 0;
volatile long ForwardCurrentTime = 0;
volatile long ForwardPulses = 0;
int ForwardPulseWidth = 0;

unsigned long currentTime = 0;
unsigned long lastChangeTime = 0;

volatile long TurnStartTime = 0;
volatile long TurnCurrentTime = 0;
volatile long TurnPulses = 0;
int TurnPulseWidth = 0;

volatile long WeaponStartTime = 0;
volatile long WeaponCurrentTime = 0;
volatile long WeaponPulses = 0;
int WeaponPulseWidth = 0;

float Speed;
float Turn;
float Weapon;
float LeftMotorSpeed;
float RightMotorSpeed;
float WeaponMotorSpeed;
float LeftPWMOut;
float RightPWMOut;
float WeaponPWMOut;

bool debugPWM = false;
bool debugMotorOutputs = false;
bool debugJoysticks = false;
bool debugPWMOut = false;

bool weaponSwitch = false;

Servo LeftMotor;
Servo RightMotor;
Servo WeaponMotor;



void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

  //setup pulse width stuff
  pinMode(RCPinForward, INPUT_PULLUP);
  pinMode(RCPinTurn, INPUT_PULLUP);
  pinMode(RCPinWeapon,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt (RCPinForward), ForwardPulseTimer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinTurn), TurnPulseTimer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinWeapon),WeaponPulseTimer,CHANGE);

  //setup motor stuff and calibrate controllers
  LeftMotor.attach(LeftMotorPin);
  LeftMotor.writeMicroseconds(1500);
  RightMotor.attach(RightMotorPin);
  RightMotor.writeMicroseconds(1500);
  WeaponMotor.attach(WeaponMotorPin);
  WeaponMotor.writeMicroseconds(1500);
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

  currentTime = millis();

  //calculate pulse widths
  if(ForwardPulses < 2050){
    ForwardPulseWidth = ForwardPulses;
    
  }
  if(TurnPulses < 2050){
    TurnPulseWidth = TurnPulses;
  }

  if(WeaponPulses < 2050){
    WeaponPulseWidth = WeaponPulses;
  }

if (abs(ForwardPulseWidth - previousPulseWidth) <= 3) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= TIME_THRESHOLD) {
      FAIL = true;
      Serial.println("Forward pulse width has not changed for 5 seconds!");
      // You can add additional actions here if needed
    }
  } else {
    // Reset the timer and previous pulse width
    previousMillis = millis();
    previousPulseWidth = ForwardPulseWidth;
    // Reset the flag
  }

  if(WeaponPulseWidth > 1900){
    FAIL = false;
  }

  

  //calculate speed joystick value based off of PWM input
  Speed = (ForwardPulseWidth-1500) / 500.;
  //calculate turn joystick value based off of PWM input
  Turn = -(TurnPulseWidth-1500) / 500.;
  //calculate weapon joystick vlaue based off PWM input
  Weapon = (WeaponPulseWidth - 1500) / 500.;
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

  if(Weapon > 1){
    Weapon = 1;
  }
  if(Weapon < -1){
    Weapon = -1;
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


  //set weapon motor boolean
  if(Weapon > 0.9){
    weaponSwitch = true;
  }
  if(Weapon < -0.9){
    weaponSwitch = false;
  } 

  //set weapon motor speed

  if(weaponSwitch == true){
    WeaponMotorSpeed = 0.35;
  }

  if(weaponSwitch == false){
    WeaponMotorSpeed = 0.0;
  }


  

  //spin motors
  LeftPWMOut = 1500 - LeftMotorSpeed * 500;
  RightPWMOut = 1500 - RightMotorSpeed * 500;
  WeaponPWMOut = 1500 + WeaponMotorSpeed * 500;


  //Serial.println(FAIL);

  if(FAIL == 0){
  LeftMotor.writeMicroseconds(LeftPWMOut);
  RightMotor.writeMicroseconds(RightPWMOut);
  WeaponMotor.writeMicroseconds(WeaponPWMOut);
  }

  if(FAIL == 1){
  LeftMotor.writeMicroseconds(1500);
  RightMotor.writeMicroseconds(1500);
  WeaponMotor.writeMicroseconds(1500);

  }
  
  //debug stuff
  if(debugPWM == true){
  Serial.print("Weapon:");
  Serial.print(WeaponPulseWidth);
  Serial.print("|");
  Serial.print("Forward:");
  Serial.print(ForwardPulseWidth);
  Serial.print("|");
  Serial.print("Turn:");
  Serial.println(TurnPulseWidth);
  
  }

  if(debugMotorOutputs == true){
  Serial.print("Weapon Motor Speed:");
  Serial.print(WeaponMotorSpeed);
  Serial.print("|");
  Serial.print("Left Motor Speed:");
  Serial.print(LeftMotorSpeed);
  Serial.print("|");
  Serial.print("Right Motor Speed:");
  Serial.println(RightMotorSpeed);
  }

  if(debugJoysticks == true){
  Serial.print("Weapon:");
  Serial.print(Weapon);
  Serial.print("|");
  Serial.print("Left:");
  Serial.print(Speed);
  Serial.print("|");
  Serial.print("Right:");
  Serial.println(Turn);
  }
  if(debugPWMOut == true){
  Serial.print("Weapon PWM Out:");
  Serial.print(WeaponPWMOut);
  Serial.print("|");
  Serial.print("Left PWM Out:");
  Serial.print(LeftPWMOut);
  Serial.print("|");
  Serial.print("Right PWM Out:");
  Serial.println(RightPWMOut);
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
  void WeaponPulseTimer(){
    WeaponCurrentTime = micros();
    if(WeaponCurrentTime > WeaponStartTime){
      WeaponPulses = WeaponCurrentTime - WeaponStartTime;
      WeaponStartTime = WeaponCurrentTime;
    }
  }

