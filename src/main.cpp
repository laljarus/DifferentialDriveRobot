#include <Arduino.h>
#include "WiFi.h"
#include "ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "PID.h"
#include "MotorControl.h"
#include "ESP32Encoder.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

const char* ssid     = "KabelBox-A1B0";
const char* password = "79191001549332230052";
// Set the rosserial socket server IP address
IPAddress server(192, 168, 0, 5);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
geometry_msgs::Vector3Stamped MotorSpeeds;
geometry_msgs::Vector3 SensValues_m3;
geometry_msgs::Vector3 SensValues_m4;
float linear_vel_cmd = 0;
float angular_vel_cmd = 0;
float throttle = 0;
float steering = 0;

void motor_cmd_cb(const geometry_msgs::Vector3& msg){

  throttle = msg.x;
  steering = msg.y;
}

ros::Subscriber<geometry_msgs::Vector3> motor_cmd("motor_cmd", &motor_cmd_cb);
ros::Publisher MotSpd("MotorSpeeds", &MotorSpeeds);
ros::Publisher LeftSens("LeftSensor",&SensValues_m3);
ros::Publisher RightSens("RightSensor",&SensValues_m4);


// Motor B
const int enB = 15;
const int in3 = 2;
const int in4 = 4;
const int MotBsensA = 25;
const int MotBsensB = 33;
ESP32Encoder encoderB;

// Motor A

const int enA = 13;
const int in1 = 12;
const int in2 = 14;
const int MotAsensA = 27;
const int MotAsensB = 26;
ESP32Encoder encoderA;

float rpm_left =0;
float rpm_right = 0;
unsigned char left_motor_cmd = 0;
unsigned char right_motor_cmd = 0;

//static bool nhChange = false;
//static bool nhOldState  = false;


// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0
#define LEDC_CHANNEL_1     1

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_8_BIT  8

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     1000

// Time in seconds for the timer
float timeSeconds=0.1;

unsigned long now = millis();
unsigned long lastTrigger = 0;
volatile unsigned int count_left = 0;
volatile unsigned int count_right= 0;
int count_left_old = 0;
int count_right_old = 0;
int num_tooth_whl = 16;
int gear_ratio = 120;

int left_int_core;
int right_int_core;

MotorControl DiffDriveMotors;

void IRAM_ATTR ISR_Left(){

  count_left++;
  left_int_core = xPortGetCoreID();
}

void IRAM_ATTR ISR_Right() {
  
  count_right++;
  right_int_core = xPortGetCoreID();
}

void EncLeftInit(void *par){
  pinMode(MotAsensA, INPUT_PULLUP);  
  pinMode(MotAsensB,INPUT);  
  attachInterrupt(MotAsensA, ISR_Left, RISING);  
}

void EncRightInit(void *par){

  pinMode(MotBsensA,INPUT_PULLUP);
  pinMode(MotBsensB,INPUT);
  attachInterrupt(MotBsensA,ISR_Right,RISING);
}


void setup(){

  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("");
  Serial.print("WiFi connected");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.print(nh.getHardware()->getLocalIP());

  DiffDriveMotors.Init(enA,in1,in2,enB,in3,in4,LEDC_CHANNEL_0,LEDC_CHANNEL_1,
                      LEDC_BASE_FREQ,LEDC_TIMER_8_BIT);


  //xTaskCreatePinnedToCore(EncLeftInit,"LeftEncoderInit",100,NULL,1,NULL,0);
  //xTaskCreatePinnedToCore(EncRightInit,"RightEncoderInit",100,NULL,2,NULL,0);

  /*pinMode(MotAsensA, INPUT_PULLUP);
  pinMode(MotBsensA,INPUT_PULLUP);
  pinMode(MotAsensB,INPUT);
  pinMode(MotBsensB,INPUT);
  attachInterrupt(MotAsensA, ISR_Left, RISING);
  attachInterrupt(MotBsensA,ISR_Right,RISING);*/

  ESP32Encoder::useInternalWeakPullResistors=true;
  encoderA.clearCount();
  encoderB.clearCount();

  //encoderA.attachHalfQuad(25,33);
  encoderA.attachHalfQuad(MotAsensA,MotAsensB);
  //encoderB.attachHalfQuad(MotBsensA,MotBsensB);  
  encoderB.attachHalfQuad(MotBsensB,MotBsensA);  
  

  nh.advertise(MotSpd);
  nh.advertise(LeftSens);
  nh.advertise(RightSens);
  nh.subscribe(motor_cmd);

  MotorSpeeds.header.seq = 0;
  MotorSpeeds.header.frame_id = "Motor Speeds";

}



void loop(/* arguments */) {
  /* code */
  if(nh.connected()){
    DiffDriveMotors.processMotorCmd(throttle,steering);
    now = millis();
    if(now-lastTrigger > timeSeconds*1000){

      rpm_left = (encoderA.getCount() - count_left_old) / (gear_ratio*num_tooth_whl * timeSeconds); // rps of left motor
      rpm_right = (encoderB.getCount() - count_right_old) / (gear_ratio*num_tooth_whl * timeSeconds); // rps of left motor
      

      count_left_old = encoderA.getCount();
      count_right_old = encoderB.getCount();      
      lastTrigger = millis();     

      MotorSpeeds.header.stamp = nh.now();
      MotorSpeeds.vector.x = encoderA.getCount();
      MotorSpeeds.vector.y = encoderB.getCount();
      MotorSpeeds.vector.z = float(0.0);
      MotSpd.publish(&MotorSpeeds);
      
      /*Serial.print("\n Left Motor Speed: \t");
      Serial.println(rpm_left);
      Serial.print("\n Right Motor Speed: \t");
      Serial.println(rpm_right); 

      SensValues_m3.x = float(digitalRead(MotBsensA));
      SensValues_m3.y = float(digitalRead(MotBsensB));
      SensValues_m3.z = rpm_left;
      LeftSens.publish(&SensValues_m3); */

      SensValues_m3.x = rpm_left;
      SensValues_m3.y = rpm_right;
      SensValues_m3.z = float(0.0);
      LeftSens.publish(&SensValues_m3);

      SensValues_m4.x = float(digitalRead(MotAsensA));
      SensValues_m4.y = float(digitalRead(MotAsensB));
      SensValues_m4.z = rpm_right;
      RightSens.publish(&SensValues_m4);

    }    

    


  }else{
    DiffDriveMotors.emergencyStop();
  }
  nh.spinOnce();

}