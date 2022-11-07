/* Controlling ESP32 Mecanum Wheel car by PS4 Controller
 * created by: Winston Yeung
 * revision date: 16 Oct 2022
*/
//Mecanum Wheel car and slave

#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1

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
#define STOP 0

#define FRONT_RIGHT_MOTOR 1
#define FRONT_LEFT_MOTOR 2
#define BACK_LEFT_MOTOR 3
#define BACK_RIGHT_MOTOR 4

#define MAX_MOTOR_SPEED 250 // about 50% duty cycle

//slave init
float theta_gyro;
float phi_gyro;
float psi_gyro;

typedef struct struct_message {
  //uint8_t data;
  float theta_gyro;
  float phi_gyro;
  float psi_gyro;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.println("Received  :");
  //Serial.println(myData.data);
  //newData = myData.data * 10;
  Serial.print("theta_gyro: ");
  Serial.println(myData.theta_gyro);
  Serial.print("phi_gyro: ");
  Serial.println(myData.phi_gyro);
  Serial.print("psi_gyro: ");
  Serial.println(myData.psi_gyro);
  
  theta_gyro = myData.theta_gyro;
  phi_gyro = myData.phi_gyro;
  psi_gyro = myData.psi_gyro;
  
  if (phi_gyro < -20 && theta_gyro < -20) {Serial.println("FORWARD_LEFT");moveCar(FORWARD_LEFT);}
  else if (phi_gyro < -20 && theta_gyro > 20) {Serial.println("FORWARD_RIGHT");moveCar(FORWARD_RIGHT);}
  else if (phi_gyro > 20 && theta_gyro < -20) {Serial.println("BACKWARD_LEFT");moveCar(BACKWARD_LEFT);}
  else if (phi_gyro > 20 && theta_gyro > 20) {Serial.println("BACKWARD_RIGHT");moveCar(BACKWARD_RIGHT);}
  else if (phi_gyro < -20) {Serial.println("FORWARD");moveCar(FORWARD);}
  else if (phi_gyro > 20) {Serial.println("BACKWARD");moveCar(BACKWARD);}
  else if (theta_gyro < -20) {Serial.println("LEFT");moveCar(LEFT);}
  else if (theta_gyro > 20) {Serial.println("RIGHT");moveCar(RIGHT);}
  else if (psi_gyro > 20) {Serial.println("ROTATE_LEFT");moveCar(ROTATE_LEFT);}
  else if (psi_gyro < -20) {Serial.println("ROTATE_RIGHT");moveCar(ROTATE_RIGHT);}
  else {Serial.println("STOP");moveCar(STOP);}
  Serial.println();
  //delay(100);
}

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


/*
void notify()
{
  int theta_gyro = map( theta_gyro, -127, 127, -254, 254);
  int phi_gyro = map( phi_gyro, -127, 127, -254, 254);
  int psi_gyro = map( psi_gyro, -127, 127, -254, 254);
  theta_gyro = myData.theta_gyro; // roll  stop =[-15,10],  move left < -15,    right > 10 
  phi_gyro = myData.phi_gyro;     // pitch stop = [-25,15], move forward <-25,  backward > 15
  psi_gyro = myData.psi_gyro;     // yaw  stop = [-15,20],  turn right < -15,   turn left > 20

  if (phi_gyro < -20 && theta_gyro < -20) {moveCar(FORWARD_LEFT);}
  else if (phi_gyro < -20 && theta_gyro > 20) {moveCar(FORWARD_RIGHT);}
  else if (phi_gyro > 20 && theta_gyro < -20) {moveCar(BACKWARD_LEFT);}
  else if (phi_gyro > 20 && theta_gyro > 20) {moveCar(BACKWARD_RIGHT);}
  else if (phi_gyro < -20) {moveCar(FORWARD);}
  else if (phi_gyro > 20) {moveCar(BACKWARD);}
  else if (theta_gyro < -20) {moveCar(LEFT);}
  else if (theta_gyro > 20) {moveCar(RIGHT);}
  else if (psi_gyro > 20) {moveCar(ROTATE_LEFT);}
  else if (psi_gyro < -20) {moveCar(ROTATE_RIGHT);}
  else {moveCar(STOP);}

// Print data for debugging purpose only
if (millis() - lastTimeStamp > 50)
  {
  Serial.print(phi_gyro);
  Serial.print(',');
  Serial.print(theta_gyro);
  Serial.print(',');
  Serial.print(psi_gyro);
  Serial.println();
  lastTimeStamp = millis();
  }
}
*/
void onConnect()
{
  Serial.println("Connected!");
}

void onDisConnect()
{
  //rotateMotor(0, 0);
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
  
    case RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;
  
    case LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);  
      break;
  
    case FORWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);  
      break;
  
    case FORWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, STOP);  
      break;
  
    case BACKWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, STOP);   
      break;
  
    case BACKWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
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
  /*if (motorSpeed < 0)
  {
    digitalWrite(FrontRightMotorPin1,LOW);
    digitalWrite(FrontRightMotorPin2,HIGH);
    digitalWrite(FrontLeftMotorPin1,LOW);
    digitalWrite(FrontLeftMotorPin2,HIGH);
    digitalWrite(BackRightMotorPin1,LOW);
    digitalWrite(BackRightMotorPin2,HIGH);     
    digitalWrite(BackLeftMotorPin1,LOW);
    digitalWrite(BackLeftMotorPin2,HIGH);
  }
  else if (motorSpeed > 0)
  {
    digitalWrite(FrontRightMotorPin1,HIGH);
    digitalWrite(FrontRightMotorPin2,LOW);  
    digitalWrite(FrontLeftMotorPin1,HIGH);
    digitalWrite(FrontLeftMotorPin2,LOW);
    digitalWrite(BackRightMotorPin1,HIGH);
    digitalWrite(BackRightMotorPin2,LOW);     
    digitalWrite(BackLeftMotorPin1,HIGH);
    digitalWrite(BackLeftMotorPin2,LOW);
  }
  else
  {
    digitalWrite(FrontRightMotorPin1,LOW);
    digitalWrite(FrontRightMotorPin2,LOW);     
    digitalWrite(FrontLeftMotorPin1,LOW);
    digitalWrite(FrontLeftMotorPin2,LOW);
    digitalWrite(BackRightMotorPin1,LOW);
    digitalWrite(BackRightMotorPin2,LOW);     
    digitalWrite(BackLeftMotorPin1,LOW);
    digitalWrite(BackLeftMotorPin2,LOW);   
  }*/
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
  /*ledcSetup(PWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableFrontRightMotor, PWMSpeedChannel);
  ledcAttachPin(enableFrontLeftMotor, PWMSpeedChannel); 
  ledcAttachPin(enableBackRightMotor, PWMSpeedChannel);
  ledcAttachPin(enableBackLeftMotor, PWMSpeedChannel);
  ledcWrite(PWMSpeedChannel, MAX_MOTOR_SPEED);

  rotateMotor(0, 0);*/
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

}

void loop()
{
  
}
