
#ifndef GIMBAL_H
#define GIMBAL_H
#include "main.h"
#include "Chassis.h"
#include "c2c.h"
#include "PID.h"
#include "INS.h"
#include "referee.h"

typedef enum 
{
   STEP_GIMBAL_CLOSE = 0,
   STEP_GIMBAL_STOP  = 1,
   STEP_GIMBAL_TRANS = 10,
   STEP_GIMBAL_READY = 20,//前哨战
   STEP_GIMBAL_FIRE  = 21,//基地
   STEP_GIMBAL_CHANGE = 21,//基地
   STEP_GIMBAL_FIRE_LAST  = 22,
}gimbal_step_e;

/*云台Yaw电机PID参数*/
#define PID_Yaw_TRANS_Kp 0.3     //0.7
#define PID_Yaw_TRANS_Ki 0
#define PID_Yaw_TRANS_Kd 0   //0.15
#define PID_Yaw_TRANS_imax 10
#define PID_Yaw_TRANS_outmax 7.5


#define PID_Yaw_START_Kp 0.7     //0.7
#define PID_Yaw_START_Ki 0
#define PID_Yaw_START_Kd 0.1   //0.15
#define PID_Yaw_START_imax 10
#define PID_Yaw_START_outmax 10
/*云台Yaw电机PID参数*/

/*飞镖摩擦轮参数*/
#define PID_ammo_vel_Kp       0.75
#define PID_ammo_vel_Ki       0
#define PID_ammo_vel_Kd       0
#define PID_ammo_vel_imax     16000
#define PID_ammo_vel_outmax   7500
/*飞镖摩擦轮参数*/

#define PID_ammo_trans__vel_Kp       0.5
#define PID_ammo_trans__vel_Ki       0
#define PID_ammo_trans__vel_Kd       0
#define PID_ammo_trans__vel_imax     16000
#define PID_ammo_trans__vel_outmax   7500

/*飞镖速度环*/
/*yaw轴*/
//#define PID_chassis1_vel_Kp     8
//#define PID_chassis1_vel_Ki     0.625       
//#define PID_chassis1_vel_Kd     1      
//#define PID_chassis1_vel_imax   1000
//#define PID_chassis1_vel_maxout 8500
//
///*推杆*/
//#define PID_chassis2_vel_Kp     6//8
//#define PID_chassis2_vel_Ki     0.05//0.625 
//#define PID_chassis2_vel_Kd     0.1  //1
//#define PID_chassis2_vel_imax   1000
//#define PID_chassis2_vel_maxout 8500
//
///*切轨道*/
//#define PID_chassis3_vel_Kp     2//6//6//3.6
//#define PID_chassis3_vel_Ki     0.005//0.01//0.05//0.01 
//#define PID_chassis3_vel_Kd     0//1//0.1//4
//#define PID_chassis3_vel_imax   1000//1000
//#define PID_chassis3_vel_maxout 8500//8500
//
///*切轨道（no）*/
//#define PID_chassis4_vel_Kp     6
//#define PID_chassis4_vel_Ki     0.01 
//#define PID_chassis4_vel_Kd     1
//#define PID_chassis4_vel_imax   1000
//#define PID_chassis4_vel_maxout 8500
///*飞镖速度环*/

//int CHANGE_Count(int a);


#endif
