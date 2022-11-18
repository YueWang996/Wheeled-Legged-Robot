//
// Created by Justin on 28/03/2022.
//

#ifndef FOCX_CAN_TEST2_MOTORCONTROL_H
#define FOCX_CAN_TEST2_MOTORCONTROL_H

#include "CAN/CAN_config.h"
#include "CAN/ESP32CAN.h"
#include "config.h"
#include "IMU.h"
#include "ServoSDK/SCServo.h"
#include "LowPass.h"

#define RAD_TO_SERVO_POS(x)             ((uint16_t)((x + 3.14159265) * 4095.0f / (6.28318527)))
#define fast_constrain(x, low, high)    ((x)<(low)?(low):((x) >(high)?(high):(x)))

class MotorControl {
public:
    MotorControl() = default;
    void initMotors(SCSCL *sc);
    void inverseKinematics(float x, float z);
    void setRobotHeight(float x, float height);

    CAN_frame_t create2ChannelMotorTargetFrame(uint32_t msg_id, float ch1_target, float ch2_target);
    void connectIMU(IMU *imu);
    void canDataDecode(CAN_frame_t rx_frame);
    void controlLoop();

    static float PIDController(PIDControlParameters pid, float error);

    SCSCL *_sc;
    uint8_t servo_id[4] = {1, 3, 5, 7};
    uint16_t servo_position[4] = {};
    uint16_t servo_speed_init[4] = {};
    uint16_t servo_speed[4] = {};
    uint16_t servo_time[4] = {0};

    float ch1_motor_position, ch2_motor_position;
    float ch1_motor_velocity, ch2_motor_velocity;

    IMU *_imu;

    CAN_frame_t _tx_frame;

    float m_fl_target;      // front left motor target
    float m_fr_target;      // front right motor target
    float m_bl_target;      // back left motor target
    float m_br_target;      // back right motor target
    float m_lw_target;      // left wheel sensor target
    float m_rw_target;      // right wheel sensor target

    float m_lw_offset;
    float m_rw_offset;

    PIDControlParameters angleLoopPID;
    PIDControlParameters velocityLoopPID;
    float velocity_loop_error_sum = 0;

    LowPass<2> ch1_velocity_lpf = LowPass<2>(1, 200, true);
    LowPass<2> ch2_velocity_lpf = LowPass<2>(1, 200, true);

    float target_velocity = 0;
    float standing_loop_output = 0;
    float mechanical_zero_angle = 0;
    float dead_zone_offset = 0.0;

    float ch1_zero_position = 0;
    float ch2_zero_position = 0;
    float zero_gyroY = 0;

    float target_torque = 0;
    float velocity_loop_output = 0;

    /// Motor board status
    bool BOARD_01_IS_READY = false;

};


#endif //FOCX_CAN_TEST2_MOTORCONTROL_H
