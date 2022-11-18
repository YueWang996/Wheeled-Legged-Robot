//
// Created by justin on 24/09/2021.
//

#ifndef DAUGI_V_0_1_IMU_H
#define DAUGI_V_0_1_IMU_H

#include "Wire.h"
#include "config.h"
#include "MPU6050/MPU6050.h"

class IMU {
public:
    IMU();
    void init(HardwareSerial *p);
    IMU_DATA getRawIMUData();
    IMU_DATA getIMUData();
    float getPitch();
    float getRoll();
    float getGyroX();
    float getGyroY();
    float getGyroZ();

    SIX_CHANNEL_BWLPF *initializeBWLPF(int order, float s, float c);
    void free_bw_low_pass(SIX_CHANNEL_BWLPF* filter);
    void ButterworthFilter(SIX_CHANNEL_BWLPF *filter);

private:
    MPU6050 mpu6050;
    HardwareSerial *hardwareSerial;
    IMU_DATA data;
};


#endif //DAUGI_V_0_1_IMU_H
