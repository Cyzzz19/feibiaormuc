
#ifndef CHASSIS_H
#define CHASSIS_H
#include "main.h"
#include "motor.h"
#include "UART_miniPC.h"
#include "c2c.h"
#include "INS.h"
#include "referee.h"
#include "math.h"


typedef enum 
{
   STEP_CHASSIS_STOP  = 0,
   STEP_CHASSIS_MOVE  = 1,
   STEP_CHASSIS_CIRCL = 2,
   STEP_CHASSIS_AUTO  = 3,
   STEP_CHASSIS_TRANS = 11,
   STEP_CHASSIS_RECO  = 12,
   STEP_CHASSIS_STAY  = 13,
   STEP_CHASSIS_RESET = 21,
   STEP_CHASSIS_ERROR = 90,
   STEP_CHASSIS_FIRST = 100,
   STEP_CHASSIS_CLOSE = 101,
}chassis_step_e;


/*底盘电机PID参数*/
#define PID_chassis1_TRANS_vel_Kp     8
#define PID_chassis1_TRANS_vel_Ki     0.625     
#define PID_chassis1_TRANS_vel_Kd     1      
#define PID_chassis1_TRANS_vel_imax   1000
#define PID_chassis1_TRANS_vel_maxout 8500

#define PID_chassis2_TRANS_vel_Kp     8
#define PID_chassis2_TRANS_vel_Ki     0.625
#define PID_chassis2_TRANS_vel_Kd     1
#define PID_chassis2_TRANS_vel_imax   1000
#define PID_chassis2_TRANS_vel_maxout 8500

#define PID_chassis3_TRANS_vel_Kp     8
#define PID_chassis3_TRANS_vel_Ki     0.625
#define PID_chassis3_TRANS_vel_Kd     1
#define PID_chassis3_TRANS_vel_imax   1000
#define PID_chassis3_TRANS_vel_maxout 8500

#define PID_chassis4_TRANS_vel_Kp     8
#define PID_chassis4_TRANS_vel_Ki     0.625
#define PID_chassis4_TRANS_vel_Kd     1
#define PID_chassis4_TRANS_vel_imax   1000
#define PID_chassis4_TRANS_vel_maxout 8500



#define PID_chassis1_TRANS_pos_Kp     75
#define PID_chassis1_TRANS_pos_Ki     0.5       
#define PID_chassis1_TRANS_pos_Kd     42.5      
#define PID_chassis1_TRANS_pos_imax   150
#define PID_chassis1_TRANS_pos_maxout 5000

#define PID_chassis2_TRANS_pos_Kp     75
#define PID_chassis2_TRANS_pos_Ki     0.5
#define PID_chassis2_TRANS_pos_Kd     42.5
#define PID_chassis2_TRANS_pos_imax   150
#define PID_chassis2_TRANS_pos_maxout 5000

#define PID_chassis3_TRANS_pos_Kp     75
#define PID_chassis3_TRANS_pos_Ki     0.5
#define PID_chassis3_TRANS_pos_Kd     42.5
#define PID_chassis3_TRANS_pos_imax   150
#define PID_chassis3_TRANS_pos_maxout 5000

#define PID_chassis4_TRANS_pos_Kp     75
#define PID_chassis4_TRANS_pos_Ki     0.5
#define PID_chassis4_TRANS_pos_Kd     42.5
#define PID_chassis4_TRANS_pos_imax   150
#define PID_chassis4_TRANS_pos_maxout 5000


#define PID_chassis1_STAY_pos_Kp     0.2
#define PID_chassis1_STAY_pos_Ki     0       
#define PID_chassis1_STAY_pos_Kd     0      
#define PID_chassis1_STAY_pos_imax   1000
#define PID_chassis1_STAY_pos_maxout 5000

#define PID_chassis2_STAY_pos_Kp     0.2
#define PID_chassis2_STAY_pos_Ki     0
#define PID_chassis2_STAY_pos_Kd     0
#define PID_chassis2_STAY_pos_imax   1000
#define PID_chassis2_STAY_pos_maxout 5000

#define PID_chassis3_STAY_pos_Kp     0.2
#define PID_chassis3_STAY_pos_Ki     0
#define PID_chassis3_STAY_pos_Kd     0
#define PID_chassis3_STAY_pos_imax   1000
#define PID_chassis3_STAY_pos_maxout 5000

#define PID_chassis4_STAY_pos_Kp     0.2
#define PID_chassis4_STAY_pos_Ki     0
#define PID_chassis4_STAY_pos_Kd     0
#define PID_chassis4_STAY_pos_imax   1000
#define PID_chassis4_STAY_pos_maxout 5000



#define PID_chassis1_MOVE_vel_Kp     8
#define PID_chassis1_MOVE_vel_Ki     0.625       
#define PID_chassis1_MOVE_vel_Kd     1      
#define PID_chassis1_MOVE_vel_imax   1000
#define PID_chassis1_MOVE_vel_maxout 8500

#define PID_chassis2_MOVE_vel_Kp     8
#define PID_chassis2_MOVE_vel_Ki     0.625 
#define PID_chassis2_MOVE_vel_Kd     1
#define PID_chassis2_MOVE_vel_imax   1000
#define PID_chassis2_MOVE_vel_maxout 8500

#define PID_chassis3_MOVE_vel_Kp     8
#define PID_chassis3_MOVE_vel_Ki     0.625 
#define PID_chassis3_MOVE_vel_Kd     1
#define PID_chassis3_MOVE_vel_imax   1000
#define PID_chassis3_MOVE_vel_maxout 8500

#define PID_chassis4_MOVE_vel_Kp     8
#define PID_chassis4_MOVE_vel_Ki     0.625 
#define PID_chassis4_MOVE_vel_Kd     1
#define PID_chassis4_MOVE_vel_imax   1000
#define PID_chassis4_MOVE_vel_maxout 8500

#define PID_chassis1_MOVE_pos_Kp     120
#define PID_chassis1_MOVE_pos_Ki     0       
#define PID_chassis1_MOVE_pos_Kd     0      
#define PID_chassis1_MOVE_pos_imax   150
#define PID_chassis1_MOVE_pos_maxout 5000

#define PID_chassis2_MOVE_pos_Kp     120
#define PID_chassis2_MOVE_pos_Ki     0
#define PID_chassis2_MOVE_pos_Kd     0
#define PID_chassis2_MOVE_pos_imax   150
#define PID_chassis2_MOVE_pos_maxout 5000

#define PID_chassis3_MOVE_pos_Kp     120
#define PID_chassis3_MOVE_pos_Ki     0
#define PID_chassis3_MOVE_pos_Kd     0
#define PID_chassis3_MOVE_pos_imax   150
#define PID_chassis3_MOVE_pos_maxout 5000

#define PID_chassis4_MOVE_pos_Kp     120
#define PID_chassis4_MOVE_pos_Ki     0
#define PID_chassis4_MOVE_pos_Kd     0
#define PID_chassis4_MOVE_pos_imax   150
#define PID_chassis4_MOVE_pos_maxout 5000



#define PID_chassis1_RESET_vel_Kp     8
#define PID_chassis1_RESET_vel_Ki     0.625       
#define PID_chassis1_RESET_vel_Kd     1      
#define PID_chassis1_RESET_vel_imax   1000
#define PID_chassis1_RESET_vel_maxout 8500

#define PID_chassis2_RESET_vel_Kp     8
#define PID_chassis2_RESET_vel_Ki     0.625 
#define PID_chassis2_RESET_vel_Kd     1
#define PID_chassis2_RESET_vel_imax   1000
#define PID_chassis2_RESET_vel_maxout 8500

#define PID_chassis3_RESET_vel_Kp     8
#define PID_chassis3_RESET_vel_Ki     0.625 
#define PID_chassis3_RESET_vel_Kd     1
#define PID_chassis3_RESET_vel_imax   1000
#define PID_chassis3_RESET_vel_maxout 8500

#define PID_chassis4_RESET_vel_Kp     8
#define PID_chassis4_RESET_vel_Ki     0.625 
#define PID_chassis4_RESET_vel_Kd     1
#define PID_chassis4_RESET_vel_imax   1000
#define PID_chassis4_RESET_vel_maxout 8500

#define PID_chassis1_RESET_pos_Kp     150
#define PID_chassis1_RESET_pos_Ki     0       
#define PID_chassis1_RESET_pos_Kd     0      
#define PID_chassis1_RESET_pos_imax   150
#define PID_chassis1_RESET_pos_maxout 5000

#define PID_chassis2_RESET_pos_Kp     150
#define PID_chassis2_RESET_pos_Ki     0
#define PID_chassis2_RESET_pos_Kd     0
#define PID_chassis2_RESET_pos_imax   150
#define PID_chassis2_RESET_pos_maxout 5000

#define PID_chassis3_RESET_pos_Kp     150
#define PID_chassis3_RESET_pos_Ki     0
#define PID_chassis3_RESET_pos_Kd     0
#define PID_chassis3_RESET_pos_imax   150
#define PID_chassis3_RESET_pos_maxout 5000

#define PID_chassis4_RESET_pos_Kp     150
#define PID_chassis4_RESET_pos_Ki     0
#define PID_chassis4_RESET_pos_Kd     0
#define PID_chassis4_RESET_pos_imax   150
#define PID_chassis4_RESET_pos_maxout 5000
/*底盘电机PID参数*/

/**
  * @brief                          电机报文结构体
  * @param Chassis_motor_measure    底盘电机报文
  * @param Pit_motor_measure        PIT电机报文
  * @param Yaw_motor_measure        Yaw电机报文
  * @param Ammo_motor_measure       发射电机报文
  */
 //typedef struct
 //{
    //motor_measure_DJ_t Chassis_motor_measure[4];
    //motor_measure_DJ_t Ammo1_motor_measure[4];
    //motor_measure_DJ_t Ammo2_motor_measure[4];
 //}data_motor_t;


#endif
