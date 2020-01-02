#ifndef _MOTORCONTROL_H_
#define _MOTORCONTROL_H_
#include "PID.h"

class MotorControl {

  public:
    MotorControl();

    void Init(const int enA_in, const int in1_in,const int in2_in,const int enB_in,
                const int in3_in, const int in4_in,const int led_channel_0_in,
                const int led_channel_1_in, const int led_freq_in, const int led_timer_precision_in);

    void setMotorA(int speed,bool dir);
    void setMotorB(int speed,bool dir);
    void stopMotorA();
    void stopMotorB();
    void stopMotors();
    void emergencyStop();
    void processMotorCmd(float linear_vel_cmd,float angular_vel_cmd);
    virtual ~MotorControl();

  private:
    int enA,enB,in1,in2,in3,in4,led_channel_0,led_channel_1,led_freq,led_timer_precision;
    PID pid_steer;

};



#endif
