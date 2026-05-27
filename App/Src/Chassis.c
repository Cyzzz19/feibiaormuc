#include "Chassis.h"
#include "remote_control.h"
#include "Data.h"


chassis_step_e chassis_step = STEP_CHASSIS_CLOSE;
pid_type_def PID_chassis_TRANS_vel[4] = {0};
pid_type_def PID_chassis_TRANS_pos[4] = {0};
pid_type_def PID_chassis_STAY_pos[4] = {0};
pid_type_def PID_chassis_MOVE_vel[4] = {0};
pid_type_def PID_chassis_MOVE_pos[4] = {0};
pid_type_def PID_chassis_RESET_pos[4] = {0};
pid_type_def PID_chassis_RESET_vel[4] = {0};
static int chassis_RECO_time = 0;
static int chassis_TRANS_time = 0;

extern data_g2c_gimbal_t data_g2c_gimbal; 
extern data_g2c_chassis_t data_g2c_chassis;
extern data_ins_t data_ins;
extern data_referee_t data_referee;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern data_rc_t data_rc;
extern  data_motor_t data_motor;

int16_t TRANS_x_speed = 0;
int16_t TRANS_y_speed = 0;
float chassis_yaw_pos = 0;

void Chassis_Task()
{
    while(1)
    {
        PID_init(&PID_chassis_TRANS_vel[0] , PID_chassis1_TRANS_vel_Kp , PID_chassis1_TRANS_vel_Ki , PID_chassis1_TRANS_vel_Kd , PID_chassis1_TRANS_vel_imax , PID_chassis1_TRANS_vel_maxout);
        PID_init(&PID_chassis_TRANS_vel[1] , PID_chassis2_TRANS_vel_Kp , PID_chassis2_TRANS_vel_Ki , PID_chassis2_TRANS_vel_Kd , PID_chassis2_TRANS_vel_imax , PID_chassis2_TRANS_vel_maxout);
        PID_init(&PID_chassis_TRANS_vel[2] , PID_chassis3_TRANS_vel_Kp , PID_chassis3_TRANS_vel_Ki , PID_chassis3_TRANS_vel_Kd , PID_chassis3_TRANS_vel_imax , PID_chassis3_TRANS_vel_maxout);
        PID_init(&PID_chassis_TRANS_vel[3] , PID_chassis4_TRANS_vel_Kp , PID_chassis4_TRANS_vel_Ki , PID_chassis4_TRANS_vel_Kd , PID_chassis4_TRANS_vel_imax , PID_chassis4_TRANS_vel_maxout);
        PID_init(&PID_chassis_TRANS_pos[0] , PID_chassis1_TRANS_pos_Kp , PID_chassis1_TRANS_pos_Ki , PID_chassis1_TRANS_pos_Kd , PID_chassis1_TRANS_pos_imax , PID_chassis1_TRANS_pos_maxout);
        PID_init(&PID_chassis_TRANS_pos[1] , PID_chassis2_TRANS_pos_Kp , PID_chassis2_TRANS_pos_Ki , PID_chassis2_TRANS_pos_Kd , PID_chassis2_TRANS_pos_imax , PID_chassis2_TRANS_pos_maxout);
        PID_init(&PID_chassis_TRANS_pos[2] , PID_chassis3_TRANS_pos_Kp , PID_chassis3_TRANS_pos_Ki , PID_chassis3_TRANS_pos_Kd , PID_chassis3_TRANS_pos_imax , PID_chassis3_TRANS_pos_maxout);
        PID_init(&PID_chassis_TRANS_pos[3] , PID_chassis4_TRANS_pos_Kp , PID_chassis4_TRANS_pos_Ki , PID_chassis4_TRANS_pos_Kd , PID_chassis4_TRANS_pos_imax , PID_chassis4_TRANS_pos_maxout);

        PID_init(&PID_chassis_STAY_pos[0] , PID_chassis1_STAY_pos_Kp , PID_chassis1_STAY_pos_Ki , PID_chassis1_STAY_pos_Kd , PID_chassis1_STAY_pos_imax , PID_chassis1_STAY_pos_maxout);
        PID_init(&PID_chassis_STAY_pos[1] , PID_chassis2_STAY_pos_Kp , PID_chassis2_STAY_pos_Ki , PID_chassis2_STAY_pos_Kd , PID_chassis2_STAY_pos_imax , PID_chassis2_STAY_pos_maxout);
        PID_init(&PID_chassis_STAY_pos[2] , PID_chassis3_STAY_pos_Kp , PID_chassis3_STAY_pos_Ki , PID_chassis3_STAY_pos_Kd , PID_chassis3_STAY_pos_imax , PID_chassis3_STAY_pos_maxout);
        PID_init(&PID_chassis_STAY_pos[3] , PID_chassis4_STAY_pos_Kp , PID_chassis4_STAY_pos_Ki , PID_chassis4_STAY_pos_Kd , PID_chassis4_STAY_pos_imax , PID_chassis4_STAY_pos_maxout);

        PID_init(&PID_chassis_MOVE_vel[0] , PID_chassis1_MOVE_vel_Kp , PID_chassis1_MOVE_vel_Ki , PID_chassis1_MOVE_vel_Kd , PID_chassis1_MOVE_vel_imax , PID_chassis1_MOVE_vel_maxout);
        PID_init(&PID_chassis_MOVE_vel[1] , PID_chassis2_MOVE_vel_Kp , PID_chassis2_MOVE_vel_Ki , PID_chassis2_MOVE_vel_Kd , PID_chassis2_MOVE_vel_imax , PID_chassis2_MOVE_vel_maxout);
        PID_init(&PID_chassis_MOVE_vel[2] , PID_chassis3_MOVE_vel_Kp , PID_chassis3_MOVE_vel_Ki , PID_chassis3_MOVE_vel_Kd , PID_chassis3_MOVE_vel_imax , PID_chassis3_MOVE_vel_maxout);
        PID_init(&PID_chassis_MOVE_vel[3] , PID_chassis4_MOVE_vel_Kp , PID_chassis4_MOVE_vel_Ki , PID_chassis4_MOVE_vel_Kd , PID_chassis4_MOVE_vel_imax , PID_chassis4_MOVE_vel_maxout);
        PID_init(&PID_chassis_MOVE_pos[0] , PID_chassis1_MOVE_pos_Kp , PID_chassis1_MOVE_pos_Ki , PID_chassis1_MOVE_pos_Kd , PID_chassis1_MOVE_pos_imax , PID_chassis1_MOVE_pos_maxout);
        PID_init(&PID_chassis_MOVE_pos[1] , PID_chassis2_MOVE_pos_Kp , PID_chassis2_MOVE_pos_Ki , PID_chassis2_MOVE_pos_Kd , PID_chassis2_MOVE_pos_imax , PID_chassis2_MOVE_pos_maxout);
        PID_init(&PID_chassis_MOVE_pos[2] , PID_chassis3_MOVE_pos_Kp , PID_chassis3_MOVE_pos_Ki , PID_chassis3_MOVE_pos_Kd , PID_chassis3_MOVE_pos_imax , PID_chassis3_MOVE_pos_maxout);
        PID_init(&PID_chassis_MOVE_pos[3] , PID_chassis4_MOVE_pos_Kp , PID_chassis4_MOVE_pos_Ki , PID_chassis4_MOVE_pos_Kd , PID_chassis4_MOVE_pos_imax , PID_chassis4_MOVE_pos_maxout);

        PID_init(&PID_chassis_RESET_vel[0] , PID_chassis1_RESET_vel_Kp , PID_chassis1_RESET_vel_Ki , PID_chassis1_RESET_vel_Kd , PID_chassis1_RESET_vel_imax , PID_chassis1_RESET_vel_maxout);
        PID_init(&PID_chassis_RESET_vel[1] , PID_chassis2_RESET_vel_Kp , PID_chassis2_RESET_vel_Ki , PID_chassis2_RESET_vel_Kd , PID_chassis2_RESET_vel_imax , PID_chassis2_RESET_vel_maxout);
        PID_init(&PID_chassis_RESET_vel[2] , PID_chassis3_RESET_vel_Kp , PID_chassis3_RESET_vel_Ki , PID_chassis3_RESET_vel_Kd , PID_chassis3_RESET_vel_imax , PID_chassis3_RESET_vel_maxout);
        PID_init(&PID_chassis_RESET_vel[3] , PID_chassis4_RESET_vel_Kp , PID_chassis4_RESET_vel_Ki , PID_chassis4_RESET_vel_Kd , PID_chassis4_RESET_vel_imax , PID_chassis4_RESET_vel_maxout);
        PID_init(&PID_chassis_RESET_pos[0] , PID_chassis1_RESET_pos_Kp , PID_chassis1_RESET_pos_Ki , PID_chassis1_RESET_pos_Kd , PID_chassis1_RESET_pos_imax , PID_chassis1_RESET_pos_maxout);
        PID_init(&PID_chassis_RESET_pos[1] , PID_chassis2_RESET_pos_Kp , PID_chassis2_RESET_pos_Ki , PID_chassis2_RESET_pos_Kd , PID_chassis2_RESET_pos_imax , PID_chassis2_RESET_pos_maxout);
        PID_init(&PID_chassis_RESET_pos[2] , PID_chassis3_RESET_pos_Kp , PID_chassis3_RESET_pos_Ki , PID_chassis3_RESET_pos_Kd , PID_chassis3_RESET_pos_imax , PID_chassis3_RESET_pos_maxout);
        PID_init(&PID_chassis_RESET_pos[3] , PID_chassis4_RESET_pos_Kp , PID_chassis4_RESET_pos_Ki , PID_chassis4_RESET_pos_Kd , PID_chassis4_RESET_pos_imax , PID_chassis4_RESET_pos_maxout);


        osDelay(2);
        switch (chassis_step)
        {
            case STEP_CHASSIS_CLOSE:
            {
                if(data_referee.GameRobotState.power_management_ammo_output == 0)
                {
                    chassis_step = STEP_CHASSIS_CLOSE;
                }
                else if(data_referee.GameRobotState.power_management_ammo_output == 1)
                {
                    chassis_step = STEP_CHASSIS_CLOSE;
                }

                break;
            }
        }
         osDelay(1);
    }

}
