#include "MotorControl.h"
#include "Arduino.h"


MotorControl::MotorControl(){}
MotorControl::~MotorControl(){}

void MotorControl::Init(const int enA_in, const int in1_in,const int in2_in,const int enB_in,
            const int in3_in, const int in4_in,const int led_channel_0_in,
            const int led_channel_1_in, const int led_freq_in, const int led_timer_precision_in){

              enA = enA_in;
              enB = enB_in;
              in1 = in1_in;
              in2 = in2_in;
              in3 = in3_in;
              in4 = in4_in;
              led_channel_0 = led_channel_0_in;
              led_channel_1 = led_channel_1_in;
              led_freq = led_freq_in;
              led_timer_precision = led_timer_precision_in;

              // Set all the motor control pins to outputs
              ledcSetup(led_channel_0, led_freq, led_timer_precision);
              ledcAttachPin(enA, led_channel_0);

              ledcSetup(led_channel_1, led_freq, led_timer_precision);
              ledcAttachPin(enB, led_channel_1);

              pinMode(in1,OUTPUT);
              pinMode(in2,OUTPUT);
              pinMode(in3,OUTPUT);
              pinMode(in4,OUTPUT);


}

void MotorControl::setMotorA(int speed,bool dir){

  if(!dir){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    ledcWrite(led_channel_0, speed);

  }else{
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    ledcWrite(led_channel_0, speed);
  }
}

void MotorControl::setMotorB(int speed,bool dir){

  if(!dir){
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    ledcWrite(led_channel_1, speed);

  }else{
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    ledcWrite(led_channel_1, speed);
  }
}

void MotorControl::stopMotorA(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  ledcWrite(led_channel_0,0);
}

void MotorControl::stopMotorB(){
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  ledcWrite(led_channel_1,0);
}

void MotorControl::stopMotors(){
  stopMotorA();
  stopMotorB();
}

void MotorControl::emergencyStop(){
  ledcWrite(led_channel_0,0);
  ledcWrite(led_channel_1,0);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,HIGH);
}

/*
void MotorControl::processMotorCmd(float& linear_vel_cmd,float& angular_vel_cmd){


unsigned char left_spd,right_spd;
bool robot_dir_left,robot_dir_right;

motor_spd_left = abs(linear_vel_cmd);
motor_spd_right = abs(linear_vel_cmd);

if(linear_vel_cmd>0){
    
  robot_dir_left = false;
  robot_dir_right = false;
}else{
  
  robot_dir_left = true;
  robot_dir_right = true;
}

if(angular_vel_cmd<0){
  motor_spd_left = motor_spd_left + 2*angular_vel_cmd;
}else{
  motor_spd_right = motor_spd_right - 2*angular_vel_cmd;
}

if(motor_spd_left >1){
  motor_spd_left = 1;
}else if(motor_spd_left < -1){
  motor_spd_left = 1;
  robot_dir_left = true;
}else if (motor_spd_left <= 0){
  motor_spd_left = abs(motor_spd_left);
  robot_dir_left = true;
}

if(motor_spd_right > 1){
  motor_spd_right = 1;
}else if(motor_spd_right < -1){
  motor_spd_right = 1;
  robot_dir_right = true;
}else if (motor_spd_right <= 0){
  motor_spd_right = abs(motor_spd_right);
  robot_dir_right = true;
}

left_spd = static_cast<unsigned char>(motor_spd_left*255);
right_spd = static_cast<unsigned char>(motor_spd_right*255);

setMotorA(left_spd, robot_dir_left);
setMotorB(right_spd,robot_dir_right);

}*/

void MotorControl::processMotorCmd(float& linear_vel_cmd,float& angular_vel_cmd){

  float motor_spd_left, motor_spd_right;
  
  motor_spd_left = linear_vel_cmd + angular_vel_cmd;
  motor_spd_right = linear_vel_cmd - angular_vel_cmd;

  if (motor_spd_left > 1){
    motor_spd_left = 1;
  }else if (motor_spd_left < -1){
    motor_spd_left = -1;
  }

  if(motor_spd_right >1){
    motor_spd_right = 1;
  }else if(motor_spd_right < -1){
    motor_spd_right = -1;
  }  

  left_spd = static_cast<unsigned char>(abs(motor_spd_left)*255);
  right_spd = static_cast<unsigned char>(abs(motor_spd_right)*255);

  if(motor_spd_left < 0){
    dir_left = true;
  }else{
    dir_left = false;
  }

  if(motor_spd_right < 0){
    dir_right = true;
  }else{
    dir_right = false;
  }

  setMotorA(left_spd,dir_left);
  setMotorB(right_spd,dir_right);

}
