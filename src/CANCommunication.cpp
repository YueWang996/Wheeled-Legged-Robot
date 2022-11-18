//
// Created by Justin on 26/03/2022.
//

#include "CANCommunication.h"

CANCommunication::CANCommunication(uint32_t id) {
    can_device_id = id;
}

void CANCommunication::init() {
    // Init CAN Module
    ESP32Can.CANInit();
}

void CANCommunication::sendFloat(float data) {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = can_device_id;
    tx_frame.FIR.B.RTR =CAN_no_RTR;
    tx_frame.FIR.B.DLC = sizeof(float);
    xthal_memcpy(tx_frame.data.u8, &data, sizeof(float));
    ESP32Can.CANWriteFrame(&tx_frame);
}

void CANCommunication::registerDevice() {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = can_device_id;
    tx_frame.FIR.B.RTR =CAN_RTR;
    tx_frame.FIR.B.DLC = 0;
    ESP32Can.CANWriteFrame(&tx_frame);
}

void CANCommunication::unpack2ChannelFloatData(float *data_ch1, float *data_ch2) {
    ((uint8_t*)data_ch1)[0] = rx_frame.data.u8[0];
    ((uint8_t*)data_ch1)[1] = rx_frame.data.u8[1];
    ((uint8_t*)data_ch1)[2] = rx_frame.data.u8[2];
    ((uint8_t*)data_ch1)[3] = rx_frame.data.u8[3];
    ((uint8_t*)data_ch2)[0] = rx_frame.data.u8[4];
    ((uint8_t*)data_ch2)[1] = rx_frame.data.u8[5];
    ((uint8_t*)data_ch2)[2] = rx_frame.data.u8[6];
    ((uint8_t*)data_ch2)[3] = rx_frame.data.u8[7];
}

void CANCommunication::dataUnpack() {
    if(rx_frame.MsgID == SET_TARGET) {
        unpack2ChannelFloatData(&_foc_ch1->target, &_foc_ch2->target);
    }
}


void CANCommunication::reportSensorPosition() {
    position_report_frame.FIR.B.FF = CAN_frame_std;
    position_report_frame.MsgID = REPORT_POSITION;
    position_report_frame.FIR.B.RTR =CAN_no_RTR;
    position_report_frame.FIR.B.DLC = 8;

    float ch1_position = _foc_ch1->shaft_angle;
    float ch2_position = _foc_ch2->shaft_angle;
    position_report_frame.data.u8[0] = ((uint8_t *) &ch1_position)[0];
    position_report_frame.data.u8[1] = ((uint8_t *) &ch1_position)[1];
    position_report_frame.data.u8[2] = ((uint8_t *) &ch1_position)[2];
    position_report_frame.data.u8[3] = ((uint8_t *) &ch1_position)[3];
    position_report_frame.data.u8[4] = ((uint8_t *) &ch2_position)[0];
    position_report_frame.data.u8[5] = ((uint8_t *) &ch2_position)[1];
    position_report_frame.data.u8[6] = ((uint8_t *) &ch2_position)[2];
    position_report_frame.data.u8[7] = ((uint8_t *) &ch2_position)[3];
    ESP32Can.CANWriteFrame(&position_report_frame);
}

void CANCommunication::reportSensorVelocity() {
    velocity_report_frame.FIR.B.FF = CAN_frame_std;
    velocity_report_frame.MsgID = REPORT_VELOCITY;
    velocity_report_frame.FIR.B.RTR =CAN_no_RTR;
    velocity_report_frame.FIR.B.DLC = 8;

    float ch1_velocity = _foc_ch1->shaft_velocity;
    float ch2_velocity = _foc_ch2->shaft_velocity;
    velocity_report_frame.data.u8[0] = ((uint8_t *) &ch1_velocity)[0];
    velocity_report_frame.data.u8[1] = ((uint8_t *) &ch1_velocity)[1];
    velocity_report_frame.data.u8[2] = ((uint8_t *) &ch1_velocity)[2];
    velocity_report_frame.data.u8[3] = ((uint8_t *) &ch1_velocity)[3];
    velocity_report_frame.data.u8[4] = ((uint8_t *) &ch2_velocity)[0];
    velocity_report_frame.data.u8[5] = ((uint8_t *) &ch2_velocity)[1];
    velocity_report_frame.data.u8[6] = ((uint8_t *) &ch2_velocity)[2];
    velocity_report_frame.data.u8[7] = ((uint8_t *) &ch2_velocity)[3];
    ESP32Can.CANWriteFrame(&velocity_report_frame);
}


void CANCommunication::registerDriver(BLDCMotor *m1, BLDCMotor *m2) {
    _foc_ch1 = m1;
    _foc_ch2 = m2;
}
