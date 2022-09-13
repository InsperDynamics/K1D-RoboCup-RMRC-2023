#include <IMU.h>
cIMU IMU;
int calibrationTime = 10000;
uint32_t roll = 0;
uint32_t pitch = 0;
uint32_t yaw = 0;

void CalibrateIMU(){
    IMU.begin();
    uint32_t timeinitial = millis();
    while(millis() - timeinitial < calibrationTime){
        IMU.update();
    }
}

void UpdateIMU(){
    IMU.update();
    roll = IMU.rpy[0];
    pitch = IMU.rpy[1];
    yaw = IMU.rpy[2];
}