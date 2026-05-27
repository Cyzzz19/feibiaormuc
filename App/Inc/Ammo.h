#ifndef AMMO_H
#define AMMO_H

#include "main.h"
#include "remote_control.h"
#include "motor.h"
#include "Gimbal.h"
#include "PID.h"
#include "referee.h"

typedef enum 
{
   STEP_AMMO_STOP  = 1,
   STEP_AMMO_FIRE  = 20,
   STEP_AMMO_FIRE_PUSH = 21,
   STEP_AMMO_FIRE_BACK = 22,
   STEP_AMMO_CHANGE = 23,
   STEP_AMMO_COOL = 24,
   STEP_AMMO_CLOSE = 101,
}ammo_step_e;

/*拨盘电机PID参数*/
#define PID_trigger_vel_Kp       3
#define PID_trigger_vel_Ki       0.625
#define PID_trigger_vel_Kd       1
#define PID_trigger_vel_imax     1000 
#define PID_trigger_vel_outmax   7500
/*拨盘电机PID参数*/

/*飞镖摩擦轮参数*/
#define PID_ammo1_vel_Kp       4.5
#define PID_ammo1_vel_Ki       0.02
#define PID_ammo1_vel_Kd       0.7
#define PID_ammo1_vel_imax     16000
#define PID_ammo1_vel_outmax   7500

#define PID_ammo2_vel_Kp       4.5
#define PID_ammo2_vel_Ki       0.02
#define PID_ammo2_vel_Kd       0.7
#define PID_ammo2_vel_imax     16000
#define PID_ammo2_vel_outmax   7500

#define PID_ammo3_vel_Kp       3.2//5
#define PID_ammo3_vel_Ki       0//0.0092
#define PID_ammo3_vel_Kd       0.06//2.7
#define PID_ammo3_vel_imax     16000
#define PID_ammo3_vel_outmax   7500

#define PID_ammo4_vel_Kp       3.2//5//4.2
#define PID_ammo4_vel_Ki       0//0.0092//0.01
#define PID_ammo4_vel_Kd       0.06//2.7//0.7
#define PID_ammo4_vel_imax     16000
#define PID_ammo4_vel_outmax   7500

#define PID_ammo5_vel_Kp       3.2//2.1
#define PID_ammo5_vel_Ki       0
#define PID_ammo5_vel_Kd       0.06 
#define PID_ammo5_vel_imax     16000
#define PID_ammo5_vel_outmax   7500

#define PID_ammo6_vel_Kp       3.2//5
#define PID_ammo6_vel_Ki       0//0.002
#define PID_ammo6_vel_Kd       0.06//0.7
#define PID_ammo6_vel_imax     16000
#define PID_ammo6_vel_outmax   7500
/*飞镖摩擦轮参数*/

//#define PID_ammo_trans__vel_Kp       2
//#define PID_ammo_trans__vel_Ki       0.01
//#define PID_ammo_trans__vel_Kd       0.07
//#define PID_ammo_trans__vel_imax     16000
//#define PID_ammo_trans__vel_outmax   7500

/*飞镖位置环参数*/
#define PID_chassis_pos_Kp     17
#define PID_chassis_pos_Ki     0.01     
#define PID_chassis_pos_Kd     0      
#define PID_chassis_pos_imax   6000
#define PID_chassis_pos_outmax 9000
/*飞镖位置环参数*/

/*飞镖位置环内的速度环参数*/
#define PID_chassis_pos_vel_Kp 4
#define PID_chassis_pos_vel_Ki 0 
#define PID_chassis_pos_vel_Kd 2.4
#define PID_chassis_pos_vel_imax 2000
#define PID_chassis_pos_vel_outmax 2000
/*飞镖位置环内的速度环参数*/

/*飞镖速度环*/
/*yaw轴*/
#define PID_chassis1_vel_Kp     8
#define PID_chassis1_vel_Ki     0.625       
#define PID_chassis1_vel_Kd     1      
#define PID_chassis1_vel_imax   1000
#define PID_chassis1_vel_maxout 8500

/*推杆*/
#define PID_chassis2_vel_Kp     25//8
#define PID_chassis2_vel_Ki     0.001//0.625 
#define PID_chassis2_vel_Kd     0//1
#define PID_chassis2_vel_imax   5000
#define PID_chassis2_vel_maxout 12000

/*pitch轴*/
#define PID_chassis3_vel_Kp     1
#define PID_chassis3_vel_Ki     0 
#define PID_chassis3_vel_Kd     0
#define PID_chassis3_vel_imax   1000
#define PID_chassis3_vel_maxout 8500

/*切轨道*/
#define PID_chassis4_vel_Kp     8
#define PID_chassis4_vel_Ki     0.625 
#define PID_chassis4_vel_Kd     1
#define PID_chassis4_vel_imax   1000
#define PID_chassis4_vel_maxout 8500
/*飞镖速度环*/

int Circle_STOP(int i, int n);

extern float ballistic_height;

#endif