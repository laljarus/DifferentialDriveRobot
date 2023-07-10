#include <Arduino.h>
#include "WiFi.h"
#include "ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "PID.h"
#include "MotorControl.h"
#include "ESP32Encoder.h"
#include "IMU_9250.h"
#include "Wire.h"
#include "sensor_msgs/Imu.h"

const char *ssid = "FRITZ!Box 6660 Cable AU";
const char *password = "95949519659820432227";
// Set the rosserial socket server IP address
IPAddress server(192, 168,1, 33);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
geometry_msgs::Vector3Stamped MotorSpeeds;
geometry_msgs::Vector3 SensValues_m3;
geometry_msgs::Vector3 SensValues_m4;
geometry_msgs::Vector3 Orientation;
//sensor_msgs::Imu ImuData;

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
//ros::Publisher IMU_pub("IMU_msg",&ImuData);
ros::Publisher Orientation_pub("EulerAngles",&Orientation);

//#define CherokeeRobot
//define TurtleBot
#define ESP32_S3

#if defined(CherokeeRobot)
  // Motor B
  const int enB = 32;
  const int in3 = 33;
  const int in4 = 25;
  const int MotBsensA = 35;
  const int MotBsensB = 34;
  ESP32Encoder encoderB;
  
  // Motor A
  
  const int enA = 14;
  const int in1 = 26;
  const int in2 = 27;
  const int MotAsensA = 13;
  const int MotAsensB = 12;
  ESP32Encoder encoderA;

  const int stdby = 0;
  
#elif defined(TurtleBot)
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

  const int stdby = 0;

#elif defined(ESP32_S3)
  // Motor B

  const int enB = 8;
  const int in3 = 17;
  const int in4 = 18;
  const int MotBsensA = 35;
  const int MotBsensB = 0;
  ESP32Encoder encoderB;

  // Motor A
  const int enA = 6;
  const int in1 = 7;
  const int in2 = 15;
  const int MotAsensA = 42;
  const int MotAsensB = 41;
  ESP32Encoder encoderA;

  const int stdby = 16;

  const int EncAPwr = 1;
  const int EncAGnd = 2;
  const int EncBPwr = 37;
  const int EncBGnd = 36;

#endif

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

void calc_yawrate(float& rps_left,float& rps_right,float& radius,float& yaw_rate){
  
  // radius of the robot wheel in m
  float WhlDia = 0.065;

  // robot wheel base in m
  float WhlBase = 0.170;

  // calculation of linear velocity from rps
  float vel_left = PI *WhlDia*rps_left;
  float vel_right = PI * WhlDia * rps_right;

  yaw_rate = (vel_left - vel_right)/WhlBase;

  if(yaw_rate < 0.0001){
    radius = 10000;
  }
  else{
    radius = (vel_left + vel_right) / (2 * yaw_rate);
  }
  
}

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

#define MPU_9250
//#define NXP_Precision

#if defined(MPU_9250)
  MPU9250 IMU(Wire, 0x68);
  IMU_9250 imu_9250;
#elif defined (NXP_Precision)


#endif

void EncoderSetup(){

  #if defined(ESP32_S3)

    pinMode(EncAPwr,OUTPUT);
    pinMode(EncAGnd,OUTPUT);
    pinMode(EncBPwr,OUTPUT);
    pinMode(EncBGnd,OUTPUT);

    digitalWrite(EncAPwr,HIGH);
    digitalWrite(EncAGnd,LOW);
    digitalWrite(EncBPwr,HIGH);
    digitalWrite(EncBGnd,LOW);
  
  #endif

  encoderA.clearCount();
  encoderB.clearCount();

  ESP32Encoder::useInternalWeakPullResistors = DOWN;

  
  encoderA.attachHalfQuad(MotAsensA,MotAsensB);  
  encoderB.attachHalfQuad(MotBsensB,MotBsensA);  

  left_int_core = digitalPinToInterrupt(MotAsensA);
  right_int_core = digitalPinToInterrupt(MotBsensB);

  
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

  /*
  ESP32Encoder::useInternalWeakPullResistors = DOWN;
  encoderA.clearCount();
  encoderB.clearCount();

  //encoderA.attachHalfQuad(25,33);
  encoderA.attachHalfQuad(MotAsensA,MotAsensB);
  //encoderB.attachHalfQuad(MotBsensA,MotBsensB);  
  encoderB.attachHalfQuad(MotBsensB,MotBsensA);  */

  EncoderSetup();
  
  #if defined(MPU_9250)

    #if defined(ESP32_S3)
      const int SDA = 20;
      const int SCL = 19;

      Wire.begin(SDA,SCL);
    #endif
    bool status = IMU.begin();
    if (status < 0)
    {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      nh.logwarn("IMU initialization unsuccessful");
      nh.logwarn("Check IMU wiring or try cycling power");
      nh.logwarn("Status: ");
      while (1)
      {
      }
    }
  #elif defined(NXP_Precision)

  #endif
  

  nh.advertise(MotSpd);
  nh.advertise(LeftSens);
  nh.advertise(RightSens);
  nh.subscribe(motor_cmd);
  //nh.advertise(IMU_pub);
  nh.advertise(Orientation_pub);

  MotorSpeeds.header.seq = 0;
  MotorSpeeds.header.frame_id = "Motor Speeds";

}



void loop(/* arguments */) {
  /* code */
  if(nh.connected()){

    DiffDriveMotors.processMotorCmd(throttle,steering);

    #if defined(MPU_9250)
      imu_9250.RunOnce();
    #elif defined(NXP_Precision)

    #endif

    now = millis();
    if(now-lastTrigger > timeSeconds*1000){

      rpm_left = (encoderA.getCount() - count_left_old) / (gear_ratio*num_tooth_whl * timeSeconds); // rps of left motor
      rpm_right = (encoderB.getCount() - count_right_old) / (gear_ratio*num_tooth_whl * timeSeconds); // rps of left motor
      

      count_left_old = encoderA.getCount();
      count_right_old = encoderB.getCount();      
      lastTrigger = millis();     

      float YawRate,CurRadius;

      calc_yawrate(rpm_left,rpm_right,CurRadius,YawRate);

      MotorSpeeds.header.stamp = nh.now();
      MotorSpeeds.vector.x = encoderA.getCount();
      MotorSpeeds.vector.y = encoderB.getCount();
      MotorSpeeds.vector.z = YawRate;
      MotSpd.publish(&MotorSpeeds);
      
      /*Serial.print("\n Left Motor Speed: \t");
      Serial.println(rpm_left);
      Serial.print("\n Right Motor Speed: \t");
      Serial.println(rpm_right); 

      SensValues_m3.x = float(digitalRead(MotBsensA));
      SensValues_m3.y = float(digitalRead(MotBsensB));
      SensValues_m3.z = rpm_left;
      LeftSens.publish(&SensValues_m3); */
      //bool state = IMU.readSensor();

      SensValues_m3.x = rpm_left;
      SensValues_m3.y = rpm_right;
      #if defined(MPU_9250)

        SensValues_m3.z = float(imu_9250.status);
      #elif defined(NXP_Precision)

      #endif
      LeftSens.publish(&SensValues_m3);

      float LeftCmd,RightCmd;

      if(DiffDriveMotors.getLeftDir()){
        LeftCmd = DiffDriveMotors.getLeftSpeed() * -1;
      }else{
        LeftCmd = DiffDriveMotors.getLeftSpeed();
      }

      if(DiffDriveMotors.getRightDir()){
        RightCmd = DiffDriveMotors.getRightSpeed() * -1;
      }else{
        RightCmd = DiffDriveMotors.getRightSpeed();
      }

      SensValues_m4.x = LeftCmd;
      SensValues_m4.y = RightCmd;
      #if defined(MPU_9250)
        
        SensValues_m4.z = imu_9250.gz;
      #elif defined(NXP_Precision)

      #endif
      
      RightSens.publish(&SensValues_m4);

      #if defined(MPU_9250)

        Orientation.x = imu_9250.pitch;
        Orientation.y = imu_9250.roll;
        Orientation.z = imu_9250.yaw;
        Orientation_pub.publish(&Orientation);
      #elif defined(NXP_Precision)

      #endif

      

      /*

      ImuData.header.stamp = nh.now();
      ImuData.header.frame_id = "base_link";
      ImuData.orientation.w = imu_9250.quat[0];
      ImuData.orientation.x = imu_9250.quat[1];
      ImuData.orientation.y = imu_9250.quat[2];
      ImuData.orientation.z = imu_9250.quat[3];
      ImuData.linear_acceleration.x = imu_9250.a_linear(0);
      ImuData.linear_acceleration.y = imu_9250.a_linear(1);
      ImuData.linear_acceleration.z = imu_9250.a_linear(2);
      ImuData.angular_velocity.x = imu_9250.gx;
      ImuData.angular_velocity.y = imu_9250.gy;
      ImuData.angular_velocity.z = imu_9250.gz;
      IMU_pub.publish(&ImuData);

      */
    }  
    
    //float ax = IMU.getAccelX_mss();
    //float ay = IMU.getAccelY_mss();
    //float az = IMU.getAccelZ_mss();

    
  }else{
    DiffDriveMotors.emergencyStop();
  }
  nh.spinOnce();

}