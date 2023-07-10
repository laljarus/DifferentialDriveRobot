#include<IMU_9250.h>

IMU_9250::IMU_9250(){
    
    //status = IMU.begin();
    a_gravity ={ 0,0,g};

}

IMU_9250::~IMU_9250(){}

void IMU_9250::RunOnce(){
    status = IMU.readSensor();
    if(status==1){
        ax = IMU.getAccelX_mss();
        ay = IMU.getAccelY_mss();
        az = IMU.getAccelZ_mss();
        gx = IMU.getGyroX_rads();
        gy = IMU.getGyroY_rads();
        gz = IMU.getGyroZ_rads();
        mx = IMU.getMagX_uT();
        my = IMU.getMagY_uT();
        mz = IMU.getMagZ_uT();

        deltat = fusion.deltatUpdate();
        fusion.MahonyUpdate(gx, gy, gz, ax, ay, az,mx ,my ,mz ,deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
        //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az,deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
        //fusion.MadgwickUpdate(gx,gy,gz,ax,ay,az,deltat);
        pitch = fusion.getPitch();
        roll = fusion.getRoll(); //you could also use getRollRadians() ecc
        yaw = fusion.getYaw();


        phi = fusion.getRollRadians();
        theta = fusion.getPitchRadians();
        psi = fusion.getYawRadians();
        quat = fusion.getQuaternions();    

        Rz ={ cosf(psi),-1*sinf(psi),0,
              sinf(psi),cosf(psi),0,
              0,0,1};

        Ry ={ cosf(theta),0,sinf(theta),
              0,1,0,
              -1*sinf(theta),0,cosf(theta)};

        Rx ={ 1,0,0,
              0,cosf(phi),-1*sinf(phi),
              0,sinf(phi),cosf(phi)};


        R = Rz*Ry*Rx;

        a_body = {ax,ay,az};

        a_inertia = R*a_body;

        a_linear = a_inertia - a_gravity;

    }

}