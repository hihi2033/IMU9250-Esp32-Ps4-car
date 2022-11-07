/* Controlling ESP32 Mecanum Wheel car by PS4 Controller
 * created by: Winston Yeung
 * revision date: 16 Oct 2022
*/
//Mecanum Wheel car and slave
#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

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

#define BACK_RIGHT_MOTOR 0
#define BACK_LEFT_MOTOR 1
#define FRONT_RIGHT_MOTOR 2
#define FRONT_LEFT_MOTOR 3

#define MAX_MOTOR_SPEED 130 
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
  Serial.print("Received  :");
  Serial.println(myData.theta_gyro);
  Serial.println(myData.phi_gyro);
  Serial.println(myData.psi_gyro);
  theta_gyro = myData.theta_gyro;
  phi_gyro = myData.phi_gyro;
  psi_gyro = myData.psi_gyro;
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
const int PWMSpeedChannel = 4; // 0-15


void notify()
{
  int theta_gyro = map( theta_gyro, -127, 127, -254, 254);
  int phi_gyro = map( phi_gyro, -127, 127, -254, 254);
  int psi_gyro = map( psi_gyro, -127, 127, -254, 254);

  if (theta_gyro > 20 && phi_gyro < -20) {moveCar(FORWARD_LEFT);}
  else if (theta_gyro > 20 && phi_gyro > 20) {moveCar(FORWARD_RIGHT);}
  else if (theta_gyro < -20 && phi_gyro < -20) {moveCar(BACKWARD_LEFT);}
  else if (theta_gyro < -20 && phi_gyro > 20) {moveCar(BACKWARD_RIGHT);}
  else if (theta_gyro > 20) {moveCar(FORWARD);}
  else if (theta_gyro < -20) {moveCar(BACKWARD);}
  else if (phi_gyro < -20) {moveCar(LEFT);}
  else if (phi_gyro > 20) {moveCar(RIGHT);}
  else if (psi_gyro < -20) {moveCar(ROTATE_LEFT);}
  else if (psi_gyro > 20) {moveCar(ROTATE_RIGHT);}
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

void onConnect()
{
  Serial.println("Connected!");
}

void onDisConnect()
{
  rotateMotor(0, 0);
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
  if (motorSpeed < 0)
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
  }
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
  ledcSetup(PWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableFrontRightMotor, PWMSpeedChannel);
  ledcAttachPin(enableFrontLeftMotor, PWMSpeedChannel); 
  ledcAttachPin(enableBackRightMotor, PWMSpeedChannel);
  ledcAttachPin(enableBackLeftMotor, PWMSpeedChannel);
  ledcWrite(PWMSpeedChannel, MAX_MOTOR_SPEED);

  rotateMotor(0, 0);
}


void setup()
{
  setUpPinModes();
  Serial.begin(115200);
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin("3c:61:05:03:e3:0a");
  Serial.println("Initialization Ready!!!");
  //
  //PS4.begin();  
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
   WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station
  if (esp_now_init() != ESP_OK){
    Serial.println("err init....");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  Serial.print("Data manipulated theta_gyro: ");
  Serial.println(theta_gyro);
  Serial.print("Data manipulated phi_gyro: ");
  Serial.println(phi_gyro);
  Serial.print("Data manipulated psi_gyroyro: ");
  Serial.println(psi_gyro);
}
