/* Controlling ESP32 Mecanum Wheel car by PS4 Controller
 * created by: Winston Yeung
 * revision date: 22 Oct 2022
*/
#include <esp_now.h>
#include <WiFi.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define BACKWARD_LEFT 7
#define BACKWARD_RIGHT 8
#define ROTATE_LEFT 9
#define ROTATE_RIGHT 10
#define FORWARD_FORWARD_LEFT 11
#define FORWARD_FORWARD_RIGHT 12
#define BACKWARD_BACKWARD_LEFT 13
#define BACKWARD_BACKWARD_RIGHT 14
#define STOP 0

#define BACK_RIGHT_MOTOR 1
#define BACK_LEFT_MOTOR 2
#define FRONT_RIGHT_MOTOR 3
#define FRONT_LEFT_MOTOR 4

#define MAX_MOTOR_SPEED 250 // about 50% duty cycle

unsigned long lastTimeStamp = 0;

//FRONT RIGHT MOTOR
int enableFrontRightMotor = 14; 
int FrontRightMotorPin1 = 26;
int FrontRightMotorPin2 = 27;
//BACK RIGHT MOTOR
int enableBackRightMotor=22; 
int BackRightMotorPin1=16;
int BackRightMotorPin2=17;
//FRONT LEFT MOTOR
int enableFrontLeftMotor = 32;
int FrontLeftMotorPin1 = 33;
int FrontLeftMotorPin2 = 25;
//BACK LEFT MOTOR
int enableBackLeftMotor = 23;
int BackLeftMotorPin1 = 18;
int BackLeftMotorPin2 = 19;

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8; // 8-bit
const int PWMSpeedChannel_1 = 1; // 0-15
const int PWMSpeedChannel_2 = 2; // 0-15
const int PWMSpeedChannel_3 = 3; // 0-15
const int PWMSpeedChannel_4 = 4; // 0-15


void notify()
{
  int LStickY = map( PS4.LStickY(), -128, 127, -254, 254);
  int LStickX = map( PS4.LStickX(), -128, 127, -254, 254);
  int RStickX = map( PS4.RStickX(), -128, 127, -254, 254);
  
  if (LStickY > 90 && RStickX < -90) {moveCar(FORWARD_FORWARD_LEFT);}
  else if (LStickY > 90 && RStickX > 90) {moveCar(FORWARD_FORWARD_RIGHT);}
  else if (LStickY < -90 < -30 && RStickX < -90) {moveCar(BACKWARD_BACKWARD_LEFT);}
  else if (LStickY < -90 && RStickX > 90) {moveCar(BACKWARD_BACKWARD_RIGHT);}
  else if (LStickY > 30 && LStickX < -30 ) {moveCar(FORWARD_LEFT);}
  else if (LStickY > 30 && LStickX > 30) {moveCar(FORWARD_RIGHT);}
  else if (LStickY < -30 && LStickX < -30) {moveCar(BACKWARD_LEFT);}
  else if (LStickY < -30 && LStickX > 30) {moveCar(BACKWARD_RIGHT);}
  else if (LStickY > 90) {moveCar(FORWARD);}
  else if (LStickY < -90) {moveCar(BACKWARD);}
  else if (LStickX < -90) {moveCar(LEFT);}
  else if (LStickX > 90) {moveCar(RIGHT);}
  else if (RStickX < -50) {moveCar(ROTATE_LEFT);}
  else if (RStickX > 50) {moveCar(ROTATE_RIGHT);}
  else {moveCar(STOP);}

// Print data for debugging purpose only
if (millis() - lastTimeStamp > 100)
  {
  Serial.print(LStickX);
  Serial.print(',');
  Serial.print(LStickY);
  Serial.print(',');
  Serial.print(RStickX);
  Serial.println();
  lastTimeStamp = millis();
  }
}

void onConnect()
{
  Serial.println("Connected!");
}

void onDisConnect()
{
  rotateMotor(BACK_RIGHT_MOTOR, 0);
  rotateMotor(BACK_LEFT_MOTOR, 0);
  rotateMotor(FRONT_RIGHT_MOTOR, 0);
  rotateMotor(FRONT_LEFT_MOTOR, 0);
}

void moveCar(int inputValue)
{
  switch(inputValue)
  {
    case FORWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);                  
      break;
  
    case BACKWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;
  
    case LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
      break;
  
    case RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
      break;
  
    case FORWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      // rotateMotor(BACK_RIGHT_MOTOR, STOP);
      // rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);  
      break;
      
     case FORWARD_FORWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);  
      break;
      
    case FORWARD_RIGHT:
      // rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      // rotateMotor(BACK_LEFT_MOTOR, STOP);  
      break;
    case FORWARD_FORWARD_RIGHT:
       rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
       rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);  
      break;
      
    case BACKWARD_LEFT:
      // rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      // rotateMotor(BACK_LEFT_MOTOR, STOP);   
      break;

      case BACKWARD_BACKWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
      break;
  
    case BACKWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      // rotateMotor(BACK_RIGHT_MOTOR, STOP);
      // rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;
      case BACKWARD_BACKWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;
  
    case ROTATE_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
      break;
  
    case ROTATE_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
      break;
  
    case STOP:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);    
      break;
  
    default:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);    
      break;
  }
}

void rotateMotor(int motorNumber, int motorSpeed)
{
    if (motorSpeed < 0 && motorNumber == 1)
    {
    digitalWrite(BackRightMotorPin1,LOW);
    digitalWrite(BackRightMotorPin2,HIGH);  
    }
    else if (motorSpeed < 0 && motorNumber == 2)  
    { 
    digitalWrite(BackLeftMotorPin1,LOW);
    digitalWrite(BackLeftMotorPin2,HIGH);
    }
    else if (motorSpeed < 0 && motorNumber == 3)
    {
    digitalWrite(FrontRightMotorPin1,LOW);
    digitalWrite(FrontRightMotorPin2,HIGH);  
    }
    else if (motorSpeed < 0 && motorNumber == 4)  
    { 
    digitalWrite(FrontLeftMotorPin1,LOW);
    digitalWrite(FrontLeftMotorPin2,HIGH);
    } 

    else if (motorSpeed > 0 && motorNumber == 1)
    {
    digitalWrite(BackRightMotorPin1,HIGH);
    digitalWrite(BackRightMotorPin2,LOW);
    }
    else if (motorSpeed > 0 && motorNumber == 2)
    {
    digitalWrite(BackLeftMotorPin1,HIGH);
    digitalWrite(BackLeftMotorPin2,LOW);
    }
    else if (motorSpeed > 0 && motorNumber == 3)
    {
    digitalWrite(FrontRightMotorPin1,HIGH);
    digitalWrite(FrontRightMotorPin2,LOW);  
    }
    else if (motorSpeed > 0 && motorNumber == 4)  
    { 
    digitalWrite(FrontLeftMotorPin1,HIGH);
    digitalWrite(FrontLeftMotorPin2,LOW);
    }

    else if (motorSpeed == 0 && motorNumber == 1)
    {
    digitalWrite(FrontRightMotorPin1,LOW);
    digitalWrite(FrontRightMotorPin2,LOW);     
    }
    else if (motorSpeed == 0 && motorNumber == 2)
    {
    digitalWrite(FrontLeftMotorPin1,LOW);
    digitalWrite(FrontLeftMotorPin2,LOW);
    }
    else if (motorSpeed == 0 && motorNumber == 3)
    {
    digitalWrite(BackRightMotorPin1,LOW);
    digitalWrite(BackRightMotorPin2,LOW);  
    }
    else if (motorSpeed == 0 && motorNumber == 4)
    {   
    digitalWrite(BackLeftMotorPin1,LOW);
    digitalWrite(BackLeftMotorPin2,LOW);   
    }
    ledcWrite(PWMSpeedChannel_1, abs(motorSpeed));
    ledcWrite(PWMSpeedChannel_2, abs(motorSpeed));
    ledcWrite(PWMSpeedChannel_3, abs(motorSpeed));
    ledcWrite(PWMSpeedChannel_4, abs(motorSpeed));
}

void setUpPinModes()
{
  pinMode(enableFrontRightMotor,OUTPUT);
  pinMode(FrontRightMotorPin1,OUTPUT);
  pinMode(FrontRightMotorPin2,OUTPUT);
  
  pinMode(enableFrontLeftMotor,OUTPUT);
  pinMode(FrontLeftMotorPin1,OUTPUT);
  pinMode(FrontLeftMotorPin2,OUTPUT);

  pinMode(enableBackRightMotor,OUTPUT);
  pinMode(BackRightMotorPin1,OUTPUT);
  pinMode(BackRightMotorPin2,OUTPUT);
  
  pinMode(enableBackLeftMotor,OUTPUT);
  pinMode(BackLeftMotorPin1,OUTPUT);
  pinMode(BackLeftMotorPin2,OUTPUT);

  //Set up PWM for motor speed
  ledcSetup(PWMSpeedChannel_1, PWMFreq, PWMResolution);
  ledcSetup(PWMSpeedChannel_2, PWMFreq, PWMResolution);
  ledcSetup(PWMSpeedChannel_3, PWMFreq, PWMResolution);
  ledcSetup(PWMSpeedChannel_4, PWMFreq, PWMResolution);
  ledcAttachPin(enableBackRightMotor, PWMSpeedChannel_1);
  ledcAttachPin(enableBackLeftMotor, PWMSpeedChannel_2);
  ledcAttachPin(enableFrontRightMotor, PWMSpeedChannel_3);
  ledcAttachPin(enableFrontLeftMotor, PWMSpeedChannel_4); 
    
  rotateMotor(BACK_RIGHT_MOTOR, 0);
  rotateMotor(BACK_LEFT_MOTOR, 0);
  rotateMotor(FRONT_RIGHT_MOTOR, 0);
  rotateMotor(FRONT_LEFT_MOTOR, 0);
}


void setup()
{
  //
  PS4.begin();  
  uint8_t pairedDeviceBtAddr[20][6];  
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for(int i = 0; i < count; i++) 
  {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
  Serial.println("Previous BT device pairing cleared!!!");
  delay(1000);
  //
  setUpPinModes();
  Serial.begin(115200);
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin("8c:ce:4e:a7:ad:a2");
  Serial.println("Initialization Ready!!!");
}

void loop()
{
}
