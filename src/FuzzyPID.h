//
// Created by Justin on 11/04/2022.
//

#ifndef FOCX_MAIN_CONTROL_UNIT_FUZZYPID_H
#define FOCX_MAIN_CONTROL_UNIT_FUZZYPID_H

#define fast_constrain(x, low, high)    ((x)<(low)?(low):((x) >(high)?(high):(x)))
#include "MotorControl.h"

class FuzzyPID {
public:
    FuzzyPID();

    ~FuzzyPID();

    void Get_grad_membership(float erro, float erro_c);

    float Quantization(float maximum, float minimum, float x);

    float Inverse_quantization(float maximum, float minimum, float qvalues);

    void GetSumGrad();

    void GetOUT();

    float FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, float kp_max, float kp_min, float erro,
                       float erro_c, float ki_max, float ki_min, float kd_max, float kd_min, float erro_pre,
                       float errp_ppre);

    const int num_area = 8; //划分区域个数
    //float e_max;  //误差做大值
    //float e_min;  //误差最小值
    //float ec_max;  //误差变化最大值
    //float ec_min;  //误差变化最小值
    //float kp_max, kp_min;
    float e_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3}; //输入e的隶属值
    float ec_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3};//输入de/dt的隶属值
    float kp_menbership_values[7] = {-3, -2, -1, 0, 1, 2, 3};//输出增量kp的隶属值
    float ki_menbership_values[7] = {-3, -2, -1, 0, 1, 2, 3}; //输出增量ki的隶属值
    float kd_menbership_values[7] = {-3, -2, -1, 0, 1, 2, 3};  //输出增量kd的隶属值
    float fuzzyoutput_menbership_values[7] = {-3, -2, -1, 0, 1, 2, 3};

    //int menbership_values[7] = {-3,-};
    float kp;                       //PID参数kp
    float ki;                       //PID参数ki
    float kd;                       //PID参数kd
    float qdetail_kp;               //增量kp对应论域中的值
    float qdetail_ki;               //增量ki对应论域中的值
    float qdetail_kd;               //增量kd对应论域中的值
    float qfuzzy_output;
    float detail_kp;                //输出增量kp
    float detail_ki;                //输出增量ki
    float detail_kd;                //输出增量kd
    float fuzzy_output;
    float qerro;                    //输入e对应论域中的值
    float qerro_c;                  //输入de/dt对应论域中的值
    float errosum;
    float e_gradmembership[2];      //输入e的隶属度
    float ec_gradmembership[2];     //输入de/dt的隶属度
    int e_grad_index[2];            //输入e隶属度在规则表的索引
    int ec_grad_index[2];           //输入de/dt隶属度在规则表的索引
    float gradSums[7] = {0, 0, 0, 0, 0, 0, 0};
    float KpgradSums[7] = {0, 0, 0, 0, 0, 0, 0};   //输出增量kp总的隶属度
    float KigradSums[7] = {0, 0, 0, 0, 0, 0, 0};   //输出增量ki总的隶属度
    float KdgradSums[7] = {0, 0, 0, 0, 0, 0, 0};   //输出增量kd总的隶属度
    int N_B = -3, N_M = -2, N_S = -1, Z_O = 0, P_S = 1, P_M = 2, P_B = 3; //论域隶属值

    int Kp_rule_list[7][7] = {{P_B, P_B, P_M, P_M, P_S, Z_O, Z_O},     //kp规则表
                              {P_B, P_B, P_M, P_S, P_S, Z_O, N_S},
                              {P_M, P_M, P_M, P_S, Z_O, N_S, N_S},
                              {P_M, P_M, P_S, Z_O, N_S, N_M, N_M},
                              {P_S, P_S, Z_O, N_S, N_S, N_M, N_M},
                              {P_S, Z_O, N_S, N_M, N_M, N_M, N_B},
                              {Z_O, Z_O, N_M, N_M, N_M, N_B, N_B}};

    int Ki_rule_list[7][7] = {{N_B, N_B, N_M, N_M, N_S, Z_O, Z_O},     //ki规则表
                              {N_B, N_B, N_M, N_S, N_S, Z_O, Z_O},
                              {N_B, N_M, N_S, N_S, Z_O, P_S, P_S},
                              {N_M, N_M, N_S, Z_O, P_S, P_M, P_M},
                              {N_M, N_S, Z_O, P_S, P_S, P_M, P_B},
                              {Z_O, Z_O, P_S, P_S, P_M, P_B, P_B},
                              {Z_O, Z_O, P_S, P_M, P_M, P_B, P_B}};

    int Kd_rule_list[7][7] = {{P_S, N_S, N_B, N_B, N_B, N_M, P_S},    //kd规则表
                              {P_S, N_S, N_B, N_M, N_M, N_S, Z_O},
                              {Z_O, N_S, N_M, N_M, N_S, N_S, Z_O},
                              {Z_O, N_S, N_S, N_S, N_S, N_S, Z_O},
                              {Z_O, Z_O, Z_O, Z_O, Z_O, Z_O, Z_O},
                              {P_B, N_S, P_S, P_S, P_S, P_S, P_B},
                              {P_B, P_M, P_M, P_M, P_S, P_S, P_B}};

    int Fuzzy_rule_list[7][7] = {{P_B, P_B, P_B, P_B, P_M, Z_O, Z_O},
                                 {P_B, P_B, P_B, P_M, P_M, Z_O, Z_O},
                                 {P_B, P_M, P_M, P_S, Z_O, N_S, N_M},
                                 {P_M, P_M, P_S, Z_O, N_S, N_M, N_M},
                                 {P_S, P_S, Z_O, N_M, N_M, N_M, N_B},
                                 {Z_O, Z_O, Z_O, N_M, N_B, N_B, N_B},
                                 {Z_O, N_S, N_B, N_B, N_B, N_B, N_B}};

    PIDControlParameters vel{.kp = kp,
            .ki = ki,
            .kd = kd,
            .last_error = 0,
            .error_sum = 0,
            .error_sum_constrain = 2.0,
            .output_constrain = 300.0};
};


#endif //FOCX_MAIN_CONTROL_UNIT_FUZZYPID_H
