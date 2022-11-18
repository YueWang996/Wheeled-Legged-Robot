//
// Created by Justin on 26/03/2022.
//

#ifndef FOCX_CANCOMMUNICATION_H
#define FOCX_CANCOMMUNICATION_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "CAN/CAN_config.h"
#include "CAN/ESP32CAN.h"
#include "SimpleFOC/SimpleFOC.h"

/// Command
#define REPORT_POSITION                 0b00100000001
#define REPORT_VELOCITY                 0b00100000011
#define SET_TARGET                      0b00100000010
#define MASTER_RESPONSE                 0b01000000001

class CANCommunication : public ESP32CAN{
public:
    explicit CANCommunication(uint32_t id);
    void init();
    void dataUnpack();
    void sendFloat(float data);
    void registerDevice();
    void registerDriver(BLDCMotor *m1, BLDCMotor *m2);
    void reportSensorPosition();
    void reportSensorVelocity();

    CAN_frame_t rx_frame;
    CAN_frame_t position_report_frame;
    CAN_frame_t velocity_report_frame;

    float *ch1_target;
    float *ch2_target;

    BLDCMotor *_foc_ch1;
    BLDCMotor *_foc_ch2;

private:
    void unpack2ChannelFloatData(float *data_ch1, float *data_ch2);

    uint32_t can_device_id;


};


#endif //FOCX_CANCOMMUNICATION_H
