#include "PID.h"


/**
  * @brief  PID计算
  * @param  pid  PID结构体地址
  * @param  Kp  P
  * @param  Ki  I
  * @param  Kd  D
  * @param  i_max  I最大值
  * @param  max_out  最大输出值
  */
void PID_init(pid_type_def* pid , float Kp , float Ki , float Kd , float i_max , float max_out)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->i_max = i_max;
    pid->out_max = max_out;
    pid->error = pid->last_error = 0.0f;
}


/**
  * @brief  无限制的PID计算（速度环）
  * @param  pid  PID结构体地址
  * @param  ref  实际值
  * @param  set  预期值
  * @retval PID计算结果
  */
float PID_Calc(pid_type_def* pid , float ref , float set)
{
    pid->fdb = ref;
    pid->set = set;
    pid->last_error = pid->error;
    pid->error = set - ref;

    pid->pout = pid->error * pid->Kp;
    pid->iout += pid->error * pid->Ki;
    pid->dout = (pid->error - pid->last_error) * pid->Kd;

    if(pid->iout >= pid->i_max)
    {
        pid->iout = pid->i_max;
    }
    if(pid->iout <= -pid->i_max)
    {
        pid->iout = -pid->i_max;
    }

    pid->out = pid->pout + pid->iout + pid->dout;
    
    if(pid->out >= pid->out_max)
    {
        pid->out = pid->out_max;
    }
    if(pid->out <= -pid->out_max)
    {
        pid->out = -pid->out_max;
    }
    return pid->out;
}


/**
  * @brief  位姿角度为-180~180度时PID计算
  * @param  pid  PID结构体地址
  * @param  ref  实际值
  * @param  set  预期值
  * @retval PID计算结果
  */
float PID_Calc_INS(pid_type_def* pid , float ref , float set)
{
    pid->fdb = ref;
    pid->set = set;
    pid->last_error = pid->error;
    pid->error = set - ref;

    if(pid->error > 180)
    {
        pid->error -= 360;
    }
    if(pid->error < -180)
    {
        pid->error += 360;
    }
    
    pid->pout = pid->error * pid->Kp;
    pid->iout += pid->error * pid->Ki;
    pid->dout = (pid->error - pid->last_error) * pid->Kd;

    if(pid->iout >= pid->i_max)
    {
        pid->iout = pid->i_max;
    }
    if(pid->iout <= -pid->i_max)
    {
        pid->iout = -pid->i_max;
    }

    pid->out = pid->pout + pid->iout + pid->dout;
    
    if(pid->out >= pid->out_max)
    {
        pid->out = pid->out_max;
    }
    if(pid->out <= -pid->out_max)
    {
        pid->out = -pid->out_max;
    }
    return pid->out;
}



/**
  * @brief  电机编码器角度为0-8191时的PID计算
  * @param  pid  PID结构体地址
  * @param  ref  实际值
  * @param  set  预期值
  * @retval PID计算结果
  */
float PID_Calc_motorpos(pid_type_def* pid , float ref , float set)
{
    pid->fdb = ref;
    pid->set = set;
    pid->last_error = pid->error;
    pid->error = set - ref;
    
    if(pid->error > 4096)
    {
        pid->error -= 8191;
    }
    if(pid->error < -4096)
    {
        pid->error += 8191;
    }

    pid->pout = pid->error * pid->Kp;
    pid->iout += pid->error * pid->Ki;
    pid->dout = (pid->error - pid->last_error) * pid->Kd;

    if(pid->iout >= pid->i_max)
    {
        pid->iout = pid->i_max;
    }
    if(pid->iout <= -pid->i_max)
    {
        pid->iout = -pid->i_max;
    }

    pid->out = pid->pout + pid->iout + pid->dout;
    
    if(pid->out >= pid->out_max)
    {
        pid->out = pid->out_max;
    }
    if(pid->out <= -pid->out_max)
    {
        pid->out = -pid->out_max;
    }
    return pid->out;
}