//
// Created by Justin on 28/03/2022.
//

#ifndef FOCX_CAN_TEST2_CONFIG_H
#define FOCX_CAN_TEST2_CONFIG_H

//#define ENABLE_BLDC_MOTORS

/// Command
#define REPORT_POSITION                 0b00100000001
#define REPORT_VELOCITY                 0b00100000011
#define SET_TARGET                      0b00100000010
#define MASTER_RESPONSE                 0b01000000001

#define BOARD_ID_1                      0x001

#define SERVO_BAUD_RATE                 115200
#define Z_AXIS_HEIGHT_MAX               230.0f
#define Z_AXIS_HEIGHT_MIN               140.0f
#define UPPER_LEG_LEN                   120.0f
#define LOWER_LEG_LEN                   120.0f
#define UPPER_SERVO_GAP                 79.71f
#define _SQ_UPPER_LEG                   14400.0f
#define _SQ_LOWER_LEG                   14400.0f
#define _SQ_SERVO_GAP                   6353.6841f

/// the number of the LED pin
#define LEDR 25
#define LEDG 26
#define LEDB 27
#define R_channel 0
#define G_channel 1
#define B_channel 2
#define pwm_Frequency 5000
#define pwm_resolution 8

typedef struct {
    float kp;
    float ki;
    float kd;
    float last_error;
    float error_sum;
    float error_sum_constrain;
    float output_constrain;
} PIDControlParameters;

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} IMU_DATA;

typedef struct {
    int n;
    float *A;
    float *d1;
    float *d2;

    float *w10; float *w11; float *w12;
    float *w20; float *w21; float *w22;
    float *w30; float *w31; float *w32;
    float *w40; float *w41; float *w42;
    float *w50; float *w51; float *w52;
    float *w60; float *w61; float *w62;
} SIX_CHANNEL_BWLPF;

#endif //FOCX_CAN_TEST2_CONFIG_H
