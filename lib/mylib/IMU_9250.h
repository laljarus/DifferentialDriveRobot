#ifndef _IMU_9250_H_
#define _IMU_9250_H_

#include "MPU9250.h"                       // library to read data from IMU
#include "SensorFusion.h"                  // sensor fusion library
#include <vector>
#include "BasicLinearAlgebra.h"            // linear algebra library also compatible with nano ble
#include <math.h>
#include "Wire.h"

extern MPU9250 IMU;

class IMU_9250{
    public:
        IMU_9250();
        virtual ~IMU_9250();       

        float gx,gy,gz,ax,ay,az,mx,my,mz;
        int status;
        float phi,theta,psi;
        float pitch,roll,yaw;
        float deltat;
        const float g = 9.81;
        std::vector<float> quat;       

        BLA::Matrix<3> a_body,a_inertia,a_linear,a_gravity;

        void RunOnce();      


        
    private:                
        //MPU9250 IMU(Wire, 0x68);
        SF fusion;
        BLA::Matrix<3,3> Rx,Ry,Rz,R;     
               

};


#endif