#ifndef PID_H
#define PID_H 

#include "main.h"
#include "stdint.h"

/**
  * @brief  PID结构体
  * @param  Kp  比例系数
  * @param  Ki  积分系数
  * @param  Kd  微分系数
  * @param  pout    比例计算结果
  * @param  iout    积分计算结果
  * @param  dout    微分计算结果
  * @param  error   差值
  * @param  last_error  上一帧差值
  * @param  set 预期值
  * @param  fdb 实际值
  * @param  out_max 最大输出值
  * @param  out 输出值
  */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float pout;
    float iout;
    float dout;

    float error;
    float last_error;

    float set;
    float fdb;
    float out_max;
    float i_max;
    float out;

}pid_type_def;


void PID_init(pid_type_def* pid , float Kp , float Ki , float Kd , float i_max , float max_out);
float PID_Calc(pid_type_def* pid , float ref , float set);
float PID_Calc_INS(pid_type_def* pid , float ref , float set);
float PID_Calc_motorpos(pid_type_def* pid , float ref , float set);

#endif