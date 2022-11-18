//
// Created by Justin on 28/03/2022.
//

#include "MotorControl.h"

CAN_frame_t MotorControl::create2ChannelMotorTargetFrame(uint32_t msg_id, float ch1_target, float ch2_target) {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.FIR.B.RTR = CAN_no_RTR;
    tx_frame.MsgID = msg_id;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = ((uint8_t *) &ch1_target)[0];
    tx_frame.data.u8[1] = ((uint8_t *) &ch1_target)[1];
    tx_frame.data.u8[2] = ((uint8_t *) &ch1_target)[2];
    tx_frame.data.u8[3] = ((uint8_t *) &ch1_target)[3];
    tx_frame.data.u8[4] = ((uint8_t *) &ch2_target)[0];
    tx_frame.data.u8[5] = ((uint8_t *) &ch2_target)[1];
    tx_frame.data.u8[6] = ((uint8_t *) &ch2_target)[2];
    tx_frame.data.u8[7] = ((uint8_t *) &ch2_target)[3];
    return tx_frame;
}

void MotorControl::connectIMU(IMU *imu) {
    _imu = imu;
}

void MotorControl::controlLoop() {
}

void MotorControl::canDataDecode(CAN_frame_t rx_frame) {
    if(rx_frame.MsgID == REPORT_VELOCITY) {
        ((uint8_t*)&ch1_motor_velocity)[0] = rx_frame.data.u8[0];
        ((uint8_t*)&ch1_motor_velocity)[1] = rx_frame.data.u8[1];
        ((uint8_t*)&ch1_motor_velocity)[2] = rx_frame.data.u8[2];
        ((uint8_t*)&ch1_motor_velocity)[3] = rx_frame.data.u8[3];
        ((uint8_t*)&ch2_motor_velocity)[0] = rx_frame.data.u8[4];
        ((uint8_t*)&ch2_motor_velocity)[1] = rx_frame.data.u8[5];
        ((uint8_t*)&ch2_motor_velocity)[2] = rx_frame.data.u8[6];
        ((uint8_t*)&ch2_motor_velocity)[3] = rx_frame.data.u8[7];
        ch1_motor_velocity *= -1.0f;
        ch2_motor_velocity *= -1.0f;
        ch1_motor_velocity = ch1_velocity_lpf.filter(ch1_motor_velocity);
        ch2_motor_velocity = ch2_velocity_lpf.filter(ch2_motor_velocity);
    }
    if(rx_frame.MsgID == REPORT_POSITION) {
        ((uint8_t*)&ch1_motor_position)[0] = rx_frame.data.u8[0];
        ((uint8_t*)&ch1_motor_position)[1] = rx_frame.data.u8[1];
        ((uint8_t*)&ch1_motor_position)[2] = rx_frame.data.u8[2];
        ((uint8_t*)&ch1_motor_position)[3] = rx_frame.data.u8[3];
        ((uint8_t*)&ch2_motor_position)[0] = rx_frame.data.u8[4];
        ((uint8_t*)&ch2_motor_position)[1] = rx_frame.data.u8[5];
        ((uint8_t*)&ch2_motor_position)[2] = rx_frame.data.u8[6];
        ((uint8_t*)&ch2_motor_position)[3] = rx_frame.data.u8[7];
        ch1_motor_position *= -1.0f;
        ch2_motor_position *= -1.0f;
    }

}

void MotorControl::initMotors(SCSCL *sc) {
    _sc = sc;
    //Serial2.begin(SERVO_BAUD_RATE); // TX: GPIO17, RX: GPIO16
    _sc->pSerial = &Serial2;
    for(int i = 0; i < 4; i++) {
        servo_position[i] = RAD_TO_SERVO_POS(0.0f);
        servo_speed_init[i] = 300;
        servo_speed[i] = 5000;
    }
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    //_sc->SyncWritePos(servo_id, 4, servo_position, servo_time, servo_speed_init);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    setRobotHeight(0, 155);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void MotorControl::inverseKinematics(float x, float z) {
    z = fast_constrain(z, Z_AXIS_HEIGHT_MIN, Z_AXIS_HEIGHT_MAX);
    float A_1_x = UPPER_SERVO_GAP / 2, A_1_z = 0;
    float A_2_x = -UPPER_SERVO_GAP / 2, A_2_z = 0;
    float e_1 = abs(A_1_z - z);
    float e_2 = abs(A_2_z - z);
    float f_1 = abs(A_1_x - x);
    float f_2 = abs(A_2_x - x);
    float sq_d1 = e_1 * e_1 + f_1 * f_1;
    float sq_d2 = e_2 * e_2 + f_2 * f_2;
    float sqrt_d1 = sqrtf(sq_d1);
    float sqrt_d2 = sqrtf(sq_d2);

    float B1_A1_P = acosf((sq_d1) / (2 * sqrt_d1 * UPPER_LEG_LEN));
    float B2_A2_P = acosf((sq_d2) / (2 * sqrt_d2 * UPPER_LEG_LEN));

    float P_A1_A2 = acosf((_SQ_SERVO_GAP + sq_d1 - sq_d2) / (2 * UPPER_SERVO_GAP * sqrt_d1));
    float A1_A2_P = acosf((_SQ_SERVO_GAP + sq_d2 - sq_d1) / (2 * UPPER_SERVO_GAP * sqrt_d2));
    float A2_P_B2 = acosf((sq_d2) / (2 * sqrt_d2 * LOWER_LEG_LEN));

    m_fl_target = M_PI_2 - (M_PI - P_A1_A2 - B1_A1_P);
    m_bl_target = M_PI_2 - (M_PI - A1_A2_P - B2_A2_P);
    m_fr_target = m_fl_target;
    m_br_target = m_bl_target;

    m_lw_offset = (A1_A2_P - A2_P_B2);
    m_rw_offset = m_lw_offset;
}

void MotorControl::setRobotHeight(float x, float height) {
    inverseKinematics(x, height);
    servo_position[0] = RAD_TO_SERVO_POS(m_fl_target);
    servo_position[1] = RAD_TO_SERVO_POS(m_fr_target * (-1));
    servo_position[2] = RAD_TO_SERVO_POS(m_bl_target * (-1));
    servo_position[3] = RAD_TO_SERVO_POS(m_br_target);
    _sc->SyncWritePos(servo_id, 4, servo_position, servo_time, servo_speed);

#ifdef ENABLE_BLDC_MOTORS
    _tx_frame = createMotorTargetFrame(BOARD_ID_1, CH_NUMBER1, 0.0);
    ESP32Can.CANWriteFrame(&_tx_frame);
    _tx_frame = createMotorTargetFrame(BOARD_ID_1, CH_NUMBER2, 0.0);
    ESP32Can.CANWriteFrame(&_tx_frame);
#endif
}

float MotorControl::PIDController(PIDControlParameters pid, float error) {
    float output;
    pid.error_sum += error;
    pid.error_sum = fast_constrain(pid.error_sum, -pid.error_sum_constrain, pid.error_sum_constrain);
    output = error * pid.kp + pid.error_sum * pid.ki + pid.kd * (error - pid.last_error);
    //printf("error:%0.2f, output:%.2f\n", error, output);
    output = fast_constrain(output, -pid.output_constrain, pid.output_constrain);
    pid.last_error = error;
    return output;
}