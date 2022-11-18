//
// Created by justin on 24/09/2021.
//

#include "IMU.h"

IMU::IMU() {
}

void IMU::init(HardwareSerial *p) {
    hardwareSerial = p;
    Wire.begin();
    p->println("Initializing I2C devices...");
    mpu6050.initialize();
    p->println("Testing device connections...");
    p->println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    for(int i = 0; i < 50; i++) {
        mpu6050.getMotion6(&data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz);
    }
}

IMU_DATA IMU::getRawIMUData() {
    mpu6050.getMotion6(&data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz);
    return data;
}

IMU_DATA IMU::getIMUData() {
    return data;
}

float IMU::getPitch() {
    return atanf((float) -data.ax / sqrtf((float) (data.ay * data.ay + data.az * data.az))) * RAD_TO_DEG;
}

float IMU::getRoll() {
    return atan2f((float) data.ay, (float) data.az) * RAD_TO_DEG;
}

float IMU::getGyroX() {
    return (float) data.gx / 131.0f;
}

float IMU::getGyroY() {
    return (float) data.gy / 131.0f;
}

float IMU::getGyroZ() {
    return (float) data.gz / 131.0f;
}

SIX_CHANNEL_BWLPF *IMU::initializeBWLPF(int order, float s, float c) {
    SIX_CHANNEL_BWLPF* filter = (SIX_CHANNEL_BWLPF *) malloc(sizeof(SIX_CHANNEL_BWLPF));
    filter -> n = order/2;
    filter -> A = (float *)malloc(filter -> n*sizeof(float));
    filter -> d1 = (float *)malloc(filter -> n*sizeof(float));
    filter -> d2 = (float *)malloc(filter -> n*sizeof(float));
    filter -> w10 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w11 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w12 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w20 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w21 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w22 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w30 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w31 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w32 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w40 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w41 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w42 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w50 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w51 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w52 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w60 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w61 = (float *)calloc(filter -> n, sizeof(float));
    filter -> w62 = (float *)calloc(filter -> n, sizeof(float));

    float a = tanf(M_PI * c/s);
    float a2 = a * a;
    float r;

    int i;

    for(i=0; i < filter -> n; ++i){
        r = sinf(M_PI*(2.0*i+1.0)/(4.0*filter -> n));
        s = a2 + 2.0*a*r + 1.0;
        filter -> A[i] = a2/s;
        filter -> d1[i] = 2.0*(1-a2)/s;
        filter -> d2[i] = -(a2 - 2.0*a*r + 1.0)/s;
    }
    return filter;
}

void IMU::free_bw_low_pass(SIX_CHANNEL_BWLPF* filter) {
    free(filter -> A);
    free(filter -> d1);
    free(filter -> d2);
    free(filter -> w10); free(filter -> w11); free(filter -> w12);
    free(filter -> w20); free(filter -> w21); free(filter -> w22);
    free(filter -> w30); free(filter -> w31); free(filter -> w32);
    free(filter -> w40); free(filter -> w41); free(filter -> w42);
    free(filter -> w50); free(filter -> w51); free(filter -> w52);
    free(filter -> w60); free(filter -> w61); free(filter -> w62);
    free(filter);
}

void IMU::ButterworthFilter(SIX_CHANNEL_BWLPF *filter){
    mpu6050.getMotion6(&data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz);

    for(int i = 0; i < filter->n; ++i){
        filter->w10[i] = filter->d1[i]*filter->w11[i] + filter->d2[i]*filter->w12[i] + (float) data.ax;
        data.ax = filter->A[i]*(filter->w10[i] + 2.0*filter->w11[i] + filter->w12[i]);
        filter->w12[i] = filter->w11[i];
        filter->w11[i] = filter->w10[i];
        
        filter->w20[i] = filter->d1[i]*filter->w21[i] + filter->d2[i]*filter->w22[i] + (float) data.ay;
        data.ay = filter->A[i]*(filter->w20[i] + 2.0*filter->w21[i] + filter->w22[i]);
        filter->w22[i] = filter->w21[i];
        filter->w21[i] = filter->w20[i];
        
        filter->w30[i] = filter->d1[i]*filter->w31[i] + filter->d2[i]*filter->w32[i] + (float) data.az;
        data.az = filter->A[i]*(filter->w30[i] + 2.0*filter->w31[i] + filter->w32[i]);
        filter->w32[i] = filter->w31[i];
        filter->w31[i] = filter->w30[i];
        
        filter->w40[i] = filter->d1[i]*filter->w41[i] + filter->d2[i]*filter->w42[i] + (float) data.gx;
        data.gx = filter->A[i]*(filter->w40[i] + 2.0*filter->w41[i] + filter->w42[i]);
        filter->w42[i] = filter->w41[i];
        filter->w41[i] = filter->w40[i];
        
        filter->w50[i] = filter->d1[i]*filter->w51[i] + filter->d2[i]*filter->w52[i] + (float) data.gy;
        data.gy = filter->A[i]*(filter->w50[i] + 2.0*filter->w51[i] + filter->w52[i]);
        filter->w52[i] = filter->w51[i];
        filter->w51[i] = filter->w50[i];
        
        filter->w60[i] = filter->d1[i]*filter->w61[i] + filter->d2[i]*filter->w62[i] + (float) data.gz;
        data.gz = filter->A[i]*(filter->w60[i] + 2.0*filter->w61[i] + filter->w62[i]);
        filter->w62[i] = filter->w61[i];
        filter->w61[i] = filter->w60[i];
    }
}
