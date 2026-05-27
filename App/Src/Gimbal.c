#include "Gimbal.h"
#include "Ammo.h"

//gimbal_step_e gimbal_step = STEP_GIMBAL_CLOSE;
gimbal_step_e gimbal_step = STEP_GIMBAL_STOP;

extern data_g2c_gimbal_t data_g2c_gimbal;
extern data_ins_t data_ins;
extern data_motor_t data_motor;
extern data_referee_t data_referee;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern data_rc_t data_rc;

pid_type_def PID_Yaw_TRANS;
pid_type_def PID_Yaw_START;
extern  pid_type_def PID_ammo1_trans_vel[4];
extern  pid_type_def PID_ammo2_trans_vel[4];
extern pid_type_def PID_ammo1_vel[4];
extern pid_type_def PID_ammo2_vel[4];
extern pid_type_def PID_chassis_vel[4];
float yaw_motor_vel = 0;
float yaw_motor_pos = 0;
float yaw_motor_pos_zero = 0;
float yaw_gimbal_pos;
extern int vel_trans[6];
extern int vel_before[6];
extern int vel_last[6];
int count = 0;
int n = 0;
int m = 0;
int circle = 0;
int trans_time = 0;
int flag = 0;

static int yaw_trans_time = 0;
int16_t countBuffer[20];
uint8_t countPointer = 0;
#define MAX_DIFFERENCE 500
// countBuffer[countPointer] = data_motor.Ammo1_motor_measure[0].given_current;
// countPointer = (countPointer  +  1) % 20;
// uint16_t maxItem = 0;
// uint32_t averageItem = 0;
// for(uint16_t i = 0; i < 20; i++)
//{
//     if (abs(countBuffer[i]) > maxItem) maxItem = countBuffer[i];
//     averageItem += abs(countBuffer[i]);
//
//}
// averageItem /= 20;
// if (maxItem - averageItem > MAX_DIFFERENCE)//data_motor.Chassis_motor_measure->given_current>)
//{
//    n++;
//}

uint32_t testV = 0;
void Gimbal_Task()
{
    PID_init(&PID_Yaw_TRANS, PID_Yaw_TRANS_Kp, PID_Yaw_TRANS_Ki, PID_Yaw_TRANS_Kd, PID_Yaw_TRANS_imax, PID_Yaw_TRANS_outmax);
    PID_init(&PID_Yaw_START, PID_Yaw_START_Kp, PID_Yaw_START_Ki, PID_Yaw_START_Kd, PID_Yaw_START_imax, PID_Yaw_START_outmax);
    PID_init(&PID_ammo1_vel[0] , PID_ammo1_vel_Kp , PID_ammo1_vel_Ki , PID_ammo1_vel_Kd , PID_ammo1_vel_imax , PID_ammo1_vel_outmax);
    PID_init(&PID_ammo1_vel[1] , PID_ammo2_vel_Kp , PID_ammo2_vel_Ki , PID_ammo2_vel_Kd , PID_ammo2_vel_imax , PID_ammo2_vel_outmax);
    PID_init(&PID_ammo1_vel[2] , PID_ammo3_vel_Kp , PID_ammo3_vel_Ki , PID_ammo3_vel_Kd , PID_ammo3_vel_imax , PID_ammo3_vel_outmax);
    PID_init(&PID_ammo1_vel[3] , PID_ammo4_vel_Kp , PID_ammo4_vel_Ki , PID_ammo4_vel_Kd , PID_ammo4_vel_imax , PID_ammo4_vel_outmax);
    PID_init(&PID_ammo2_vel[0] , PID_ammo5_vel_Kp , PID_ammo5_vel_Ki , PID_ammo5_vel_Kd , PID_ammo5_vel_imax , PID_ammo5_vel_outmax);
    PID_init(&PID_ammo2_vel[1] , PID_ammo6_vel_Kp , PID_ammo6_vel_Ki , PID_ammo6_vel_Kd , PID_ammo6_vel_imax , PID_ammo6_vel_outmax);
    PID_init(&PID_ammo2_vel[2] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    PID_init(&PID_ammo2_vel[3] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    PID_init(&PID_chassis_vel[0], PID_chassis1_vel_Kp, PID_chassis1_vel_Ki, PID_chassis1_vel_Kd, PID_chassis1_vel_imax, PID_chassis1_vel_maxout);
    PID_init(&PID_chassis_vel[1], PID_chassis2_vel_Kp, PID_chassis2_vel_Ki, PID_chassis2_vel_Kd, PID_chassis2_vel_imax, PID_chassis2_vel_maxout);
    PID_init(&PID_chassis_vel[2], PID_chassis3_vel_Kp, PID_chassis3_vel_Ki, PID_chassis3_vel_Kd, PID_chassis3_vel_imax, PID_chassis3_vel_maxout);
    PID_init(&PID_chassis_vel[3], PID_chassis4_vel_Kp, PID_chassis4_vel_Ki, PID_chassis4_vel_Kd, PID_chassis4_vel_imax, PID_chassis4_vel_maxout);
    while (1)
    {
        // testV = *(uint32_t *)(&(data_referee.GameRobotState.chassis_power_limit));

        switch (gimbal_step)
        {
            //case STEP_GIMBAL_CLOSE:
            //{
            //    if(data_referee.GameRobotState.power_management_gimbal_output == 0)
            //    {
            //        gimbal_step = STEP_GIMBAL_CLOSE;
            //    }
            //    else if ((data_referee.GameRobotState.power_management_gimbal_output == 1))
            //    {
            //        gimbal_step = STEP_GIMBAL_STOP;
            //    }
//
            //    break;
            //    
            //}
            case STEP_GIMBAL_STOP: // 有电但不开启
            {
                DJ_124_ctrl(&hcan1, AMMO1_ALL_ID, 0, 0, 0, 0);
                DJ_124_ctrl(&hcan1, AMMO2_ALL_ID, 0, 0, 0, 0);
                DJ_124_ctrl(&hcan2, AMMO2_ALL_ID, 0, 0, 0, 0); // 摩擦轮不开启
                DJ_124_ctrl(&hcan2, 0x200, 0, 0, 0, 0);        // pitch、yaw、切轨道、推杆不开启

                //if(data_referee.GameRobotState.power_management_gimbal_output == 0)
                //{
                //    gimbal_step = STEP_GIMBAL_CLOSE;
                //}
                //else if ((data_referee.GameRobotState.power_management_gimbal_output == 1))
                //{
                    if (data_rc.status_gimbal == 0) // 右推杆最下全部不开启
                    {
                        gimbal_step = STEP_GIMBAL_STOP;
                    }
                    else if (data_rc.status_gimbal == 1 || data_rc.status_gimbal == 2) // 右推杆中间瞄准前哨站
                    {
                        gimbal_step = STEP_GIMBAL_TRANS;
                    }
                //}
                
                break;
            }
            case STEP_GIMBAL_TRANS: // 瞄准前哨站
            {
                DJ_124_ctrl_vel(&hcan1 , AMMO1_ALL_ID , PID_ammo1_trans_vel, data_motor.Ammo1_motor_measure ,vel_trans[0],vel_trans[1] ,vel_trans[2] ,vel_trans[3]);
                DJ_124_ctrl_vel(&hcan1, AMMO2_ALL_ID, PID_ammo2_trans_vel, data_motor.Ammo2_motor_measure, 0, vel_trans[5], 0, 0);
                DJ_124_ctrl_vel(&hcan2 , AMMO2_ALL_ID , PID_ammo2_trans_vel, data_motor.Ammo2_motor_measure ,vel_trans[4], 0, 0 , 0);

                //if(data_referee.GameRobotState.power_management_gimbal_output == 0)
                //{
                //    gimbal_step = STEP_GIMBAL_CLOSE;
                //}
                //else if ((data_referee.GameRobotState.power_management_gimbal_output == 1))
                //{
                    if(data_rc.status_gimbal == 0)
                    {
                        gimbal_step = STEP_GIMBAL_STOP;
                    }
                    else if (data_rc.status_gimbal == 1 || data_rc.status_gimbal == 2)
                    {
                        if(trans_time >= 270)
                        {
                            if (data_rc.status_gimbal == 1)
                            {
                                gimbal_step = STEP_GIMBAL_FIRE;
                            }
                            else if(data_rc.status_gimbal == 2)
                            {
                                gimbal_step = STEP_GIMBAL_FIRE_LAST;
                            }

                            trans_time = 0;
                        }
                        else if(trans_time < 270)
                        {
                            gimbal_step = STEP_GIMBAL_TRANS;
                            trans_time++;
                        }
                    }
                //}
                

                break;
            }
            case STEP_GIMBAL_FIRE: // 瞄准基地
            {
                DJ_124_ctrl_vel(&hcan1 , AMMO1_ALL_ID , PID_ammo1_vel, data_motor.Ammo1_motor_measure ,vel_before[0],vel_before[1] ,vel_before[2] ,vel_before[3]);
                DJ_124_ctrl_vel(&hcan1, AMMO2_ALL_ID, PID_ammo2_vel, data_motor.Ammo2_motor_measure, 0, vel_before[5], 0, 0);
                DJ_124_ctrl_vel(&hcan2 , AMMO2_ALL_ID , PID_ammo2_vel, data_motor.Ammo2_motor_measure ,vel_before[4], 0, 0 , 0);

                if(data_motor.Ammo1_motor_measure[0].given_current >= 5450)
                {
                    m++;
                }

                if(m > 0 && data_motor.Ammo1_motor_measure[0].given_current <= 700)
                {
                    n = 1;
                    m =0;
                }

                if(n == 1)
                {
                    for(int i = 0; i < 6; i++)
                    {
                        if(vel_before[i] > 0)
                        {
                            vel_before[i] = vel_before[i] - 1;
                        }
                        else if(vel_before[i] < 0)
                        {
                            vel_before[i] = vel_before[i] + 1;
                        }
                    }
                    n = 0;
                }


                //if(data_referee.GameRobotState.power_management_gimbal_output == 0)
                //{
                //    gimbal_step = STEP_GIMBAL_CLOSE;
                //}
                //else if ((data_referee.GameRobotState.power_management_gimbal_output == 1))
                //{
                    if (data_rc.status_gimbal == 0)
                    {
                        gimbal_step = STEP_GIMBAL_STOP;
                    }
                    else if (data_rc.status_gimbal == 1)
                    {
                        gimbal_step = STEP_GIMBAL_FIRE;
                    }
                    else if(data_rc.status_gimbal == 2)
                    {
                        gimbal_step = STEP_GIMBAL_FIRE_LAST;
                    }
                //}

                break;
            }

            case STEP_GIMBAL_FIRE_LAST: // 瞄准基地
            {
                DJ_124_ctrl_vel(&hcan1 , AMMO1_ALL_ID , PID_ammo1_vel, data_motor.Ammo1_motor_measure ,vel_last[0],vel_last[1] ,vel_last[2] ,vel_last[3]);
                DJ_124_ctrl_vel(&hcan1, AMMO2_ALL_ID, PID_ammo2_vel, data_motor.Ammo2_motor_measure, 0, vel_last[5], 0, 0);
                DJ_124_ctrl_vel(&hcan2 , AMMO2_ALL_ID , PID_ammo2_vel, data_motor.Ammo2_motor_measure ,vel_last[4], 0, 0 , 0);

                

                //if(data_referee.GameRobotState.power_management_gimbal_output == 0)
                //{
                //    gimbal_step = STEP_GIMBAL_CLOSE;
                //}
                //else if ((data_referee.GameRobotState.power_management_gimbal_output == 1))
                //{
                    if (data_rc.status_gimbal == 0)
                    {
                        gimbal_step = STEP_GIMBAL_STOP;
                    }
                    else if (data_rc.status_gimbal == 1)
                    {
                        gimbal_step = STEP_GIMBAL_FIRE;
                    }
                    else if(data_rc.status_gimbal == 2)
                    {
                        gimbal_step = STEP_GIMBAL_FIRE_LAST;
                    }
                //}
                break;
            }
        }
        osDelay(10);
    }
}
