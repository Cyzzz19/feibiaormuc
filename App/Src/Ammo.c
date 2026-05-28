#include "Ammo.h"
#include "motor.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "math.h"
#include "remote_control.h"
#include "Data.h"
#include "dma.h"

#define BALLISTIC_ANGLE_DEG      26.6f
#define BALLISTIC_HORIZ_DIST     16.05f
#define BALLISTIC_GRAVITY        9.81f
#define BALLISTIC_WHEEL_RADIUS   0.044f
#define BALLISTIC_PAIR1_RPM      5000

float ballistic_height = 1.5f;

//ammo_step_e ammo_step =STEP_AMMO_CLOSE;
ammo_step_e ammo_step =STEP_AMMO_STOP;
pid_type_def PID_shoot_vel[4] = {0};
pid_type_def PID_ammo1_vel[4] = {0};
pid_type_def PID_ammo2_vel[4] = {0};
pid_type_def PID_ammo1_trans_vel[4] = {0};
pid_type_def PID_ammo2_trans_vel[4] = {0};
pid_type_def PID_chassis_pos[4] = {0};
pid_type_def PID_chassis_pos_vel[4] = {0};
pid_type_def PID_chassis_vel[4] = {0};

extern data_g2c_gimbal_t data_g2c_gimbal;
extern data_motor_t data_motor;
extern data_referee_t data_referee;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart1;
extern data_rc_t data_rc;

static int shoot_G_time = 0;
int ammo_time = 0;
int len=10;
int vel_trans[6]={2500,-2500,3000,-3000,3000,-3000};
int vel_before[6]={0};
int vel_last[6]={0};

static int ballistic_calc_rpm(void)
{
    float angle_rad = BALLISTIC_ANGLE_DEG * 3.141592654f / 180.0f;
    float cos_a = cosf(angle_rad);
    float tan_a = tanf(angle_rad);
    float v0 = BALLISTIC_HORIZ_DIST / cos_a
               * sqrtf(BALLISTIC_GRAVITY / (2.0f * (BALLISTIC_HORIZ_DIST * tan_a - ballistic_height)));
    return (int)(v0 * 60.0f / (2.0f * 3.141592654f * BALLISTIC_WHEEL_RADIUS));
}

static void ballistic_set_vel_before(int rpm_base)
{
    int pair3 = rpm_base;
    int pair2 = rpm_base + 300;

    vel_before[0] =  BALLISTIC_PAIR1_RPM;  vel_before[1] = -BALLISTIC_PAIR1_RPM;
    vel_before[2] =  pair2;               vel_before[3] = -pair2;
    vel_before[4] =  pair3;               vel_before[5] = -pair3;
}
//int vel[6]={4365,-4365,4865,-4865,5365,-5365};//25米基地固定目标
//int vel[6]={3601,-3601,4101,-4101,4601,-4601};//16米前哨站目标
//int vel[6]={3601,-3601,3601,-3601,3601,-3601};
int pos[4]={5000,0,5000,0};
int veltest=4000;
int16_t ammo_circle = 0;
int last_circle = 0;
int ammo_transtime = 0;
uint32_t total_ecd = 0;
float now_ecd = 0.0;
float set_ecd = 3500;
float back_ecd = 20;
float circle_vel = 0;

void usart_printf( const char*fmt,...)
{
	static uint8_t tx_buf[256]={0};
	static va_list ap;
	static uint16_t len;
	va_start(ap,fmt);
	len=vsprintf((char*)tx_buf,fmt,ap);
	va_end(ap);
	HAL_UART_Transmit_DMA(&huart1,tx_buf,len);
}

int Circle_STOP(int i, int n)
{
    if (data_motor.Chassis_motor_measure[i].ecd - data_motor.Chassis_motor_measure[i].last_ecd > 4096)
    {
        n--;
    }
    else if (data_motor.Chassis_motor_measure[i].ecd - data_motor.Chassis_motor_measure[i].last_ecd < -4096)
    {
        n++;
    }

    return n;
}

void Ammo_Task()
{
    float last_ballistic_height;

    PID_init(&PID_shoot_vel[0] , PID_trigger_vel_Kp , PID_trigger_vel_Ki , PID_trigger_vel_Kd , PID_trigger_vel_imax , PID_trigger_vel_outmax);
    PID_init(&PID_ammo1_vel[0] , PID_ammo1_vel_Kp , PID_ammo1_vel_Ki , PID_ammo1_vel_Kd , PID_ammo1_vel_imax , PID_ammo1_vel_outmax);
    PID_init(&PID_ammo1_vel[1] , PID_ammo2_vel_Kp , PID_ammo2_vel_Ki , PID_ammo2_vel_Kd , PID_ammo2_vel_imax , PID_ammo2_vel_outmax);
    PID_init(&PID_ammo1_vel[2] , PID_ammo3_vel_Kp , PID_ammo3_vel_Ki , PID_ammo3_vel_Kd , PID_ammo3_vel_imax , PID_ammo3_vel_outmax);
    PID_init(&PID_ammo1_vel[3] , PID_ammo4_vel_Kp , PID_ammo4_vel_Ki , PID_ammo4_vel_Kd , PID_ammo4_vel_imax , PID_ammo4_vel_outmax);
    PID_init(&PID_ammo2_vel[0] , PID_ammo5_vel_Kp , PID_ammo5_vel_Ki , PID_ammo5_vel_Kd , PID_ammo5_vel_imax , PID_ammo5_vel_outmax);
    PID_init(&PID_ammo2_vel[1] , PID_ammo6_vel_Kp , PID_ammo6_vel_Ki , PID_ammo6_vel_Kd , PID_ammo6_vel_imax , PID_ammo6_vel_outmax);
    PID_init(&PID_ammo2_vel[2] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    PID_init(&PID_ammo2_vel[3] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    PID_init(&PID_ammo1_trans_vel[0] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    PID_init(&PID_ammo1_trans_vel[1] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    PID_init(&PID_ammo1_trans_vel[2] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    PID_init(&PID_ammo1_trans_vel[3] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    PID_init(&PID_ammo2_trans_vel[0] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    PID_init(&PID_ammo2_trans_vel[1] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    PID_init(&PID_ammo2_trans_vel[2] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    PID_init(&PID_ammo2_trans_vel[3] , PID_ammo_vel_Kp , PID_ammo_vel_Ki , PID_ammo_vel_Kd , PID_ammo_vel_imax , PID_ammo_vel_outmax);
    //PID_init(&PID_chassis_pos[0] , PID_chassis_pos_Kp , PID_chassis_pos_Ki , PID_chassis_pos_Kd , PID_chassis_pos_imax , PID_chassis_pos_outmax);
    //PID_init(&PID_chassis_pos[1] , PID_chassis_pos_Kp , PID_chassis_pos_Ki , PID_chassis_pos_Kd , PID_chassis_pos_imax , PID_chassis_pos_outmax);
    //PID_init(&PID_chassis_pos[2] , PID_chassis_pos_Kp , PID_chassis_pos_Ki , PID_chassis_pos_Kd , PID_chassis_pos_imax , PID_chassis_pos_outmax);
    //PID_init(&PID_chassis_pos[3] , PID_chassis_pos_Kp , PID_chassis_pos_Ki , PID_chassis_pos_Kd , PID_chassis_pos_imax , PID_chassis_pos_outmax);
    //PID_init(&PID_chassis_pos_vel[0] ,PID_chassis_pos_vel_Kp , PID_chassis_pos_vel_Ki , PID_chassis_pos_vel_Kd , PID_chassis_pos_vel_imax , PID_chassis_pos_vel_outmax);
    //PID_init(&PID_chassis_pos_vel[1] ,PID_chassis_pos_vel_Kp , PID_chassis_pos_vel_Ki , PID_chassis_pos_vel_Kd , PID_chassis_pos_vel_imax , PID_chassis_pos_vel_outmax);
    //PID_init(&PID_chassis_pos_vel[2] ,PID_chassis_pos_vel_Kp , PID_chassis_pos_vel_Ki , PID_chassis_pos_vel_Kd , PID_chassis_pos_vel_imax , PID_chassis_pos_vel_outmax);
    //PID_init(&PID_chassis_pos_vel[3] ,PID_chassis_pos_vel_Kp , PID_chassis_pos_vel_Ki , PID_chassis_pos_vel_Kd , PID_chassis_pos_vel_imax , PID_chassis_pos_vel_outmax);
    PID_init(&PID_chassis_vel[0] , PID_chassis1_vel_Kp , PID_chassis1_vel_Ki , PID_chassis1_vel_Kd , PID_chassis1_vel_imax , PID_chassis1_vel_maxout);
    PID_init(&PID_chassis_vel[1] , PID_chassis2_vel_Kp , PID_chassis2_vel_Ki , PID_chassis2_vel_Kd , PID_chassis2_vel_imax , PID_chassis2_vel_maxout);
    PID_init(&PID_chassis_vel[2] , PID_chassis3_vel_Kp , PID_chassis3_vel_Ki , PID_chassis3_vel_Kd , PID_chassis3_vel_imax , PID_chassis3_vel_maxout);
    PID_init(&PID_chassis_vel[3] , PID_chassis4_vel_Kp , PID_chassis4_vel_Ki , PID_chassis4_vel_Kd , PID_chassis4_vel_imax , PID_chassis4_vel_maxout);
    PID_init(&PID_chassis_pos[0] , PID_chassis_pos_Kp , PID_chassis_pos_Ki , PID_chassis_pos_Kd , PID_chassis_pos_imax , PID_chassis_pos_outmax);
    PID_init(&PID_chassis_pos[1] , PID_chassis_pos_Kp , PID_chassis_pos_Ki , PID_chassis_pos_Kd , PID_chassis_pos_imax , PID_chassis_pos_outmax);
    PID_init(&PID_chassis_pos[2] , PID_chassis_pos_Kp , PID_chassis_pos_Ki , PID_chassis_pos_Kd , PID_chassis_pos_imax , PID_chassis_pos_outmax);
    PID_init(&PID_chassis_pos[3] , PID_chassis_pos_Kp , PID_chassis_pos_Ki , PID_chassis_pos_Kd , PID_chassis_pos_imax , PID_chassis_pos_outmax);

    ballistic_set_vel_before(ballistic_calc_rpm());
    last_ballistic_height = ballistic_height;


    while(1)
    {
        if (ballistic_height != last_ballistic_height)
        {
            ballistic_set_vel_before(ballistic_calc_rpm());
            last_ballistic_height = ballistic_height;
        }

        switch (ammo_step)
        {
            //case STEP_AMMO_CLOSE:
            //{
            //    if ((data_referee.GameRobotState.power_management_ammo_output == 0))
            //    {
            //        ammo_step = STEP_AMMO_CLOSE;
            //    }
            //    else if(data_referee.GameRobotState.power_management_ammo_output == 1)
            //    {
            //        ammo_step = STEP_AMMO_STOP;
            //    }
//
            //    break;
            //    
            //}
            case STEP_AMMO_STOP://关闭摩擦轮
            {
                DJ_124_ctrl(&hcan2, 0x200, 0, 0, 0, 0);

                //if ((data_referee.GameRobotState.power_management_ammo_output == 0))
                //{
                //    ammo_step = STEP_AMMO_CLOSE;
                //}
                //else if(data_referee.GameRobotState.power_management_ammo_output == 1)
                //{
                    if(data_rc.status_ammo==0)
                    {
                        ammo_step = STEP_AMMO_STOP;
                    }
                    else if(data_rc.status_ammo==1||data_rc.status_ammo==2)
                    {
                        ammo_step = STEP_AMMO_FIRE;
                    }
                //}

                break;

            }

            case STEP_AMMO_FIRE:
            {
                DJ_124_ctrl(&hcan2, 0x200, 0, 0, 0, 0);
                
                //if ((data_referee.GameRobotState.power_management_ammo_output == 0))
                //{
                //    ammo_step = STEP_AMMO_CLOSE;
                //}
                //else if(data_referee.GameRobotState.power_management_ammo_output == 1)
                //{
                    if(data_rc.status_ammo==0)
                    {
                        ammo_step = STEP_AMMO_STOP;
                    }
                    else if(data_rc.status_ammo==1 || data_rc.status_ammo==2)
                    {
                        if(ammo_transtime >= 200)
                        {
                            ammo_step = STEP_AMMO_FIRE_PUSH;
                            ammo_transtime = 0;
                        }
                        else
                        {
                            ammo_step = STEP_AMMO_FIRE;
                            ammo_transtime++;
                        }
                    }

                    break;
            }

            case STEP_AMMO_FIRE_PUSH:
            {
                int16_t push_vel = 7200;
                if (ammo_circle >= 1130 && ammo_circle <= 1443)
                {
                    push_vel = 720;
                }
                DJ_124_ctrl_vel(&hcan2, 0x200, PID_chassis_vel, data_motor.Chassis_motor_measure, 0, push_vel, 0, 0);

                if(data_rc.status_ammo == 0)
                    {
                        ammo_step = STEP_AMMO_STOP;
                    }
                    else if (data_rc.status_ammo == 1 || data_rc.status_ammo == 2)
                    {
                        if (ammo_circle >= 2062)
                        {
                            ammo_step = STEP_AMMO_FIRE_BACK;
                        }
                        else
                        {
                            ammo_step = STEP_AMMO_FIRE_PUSH;
                        }
                    }
                break;
            }
            case STEP_AMMO_FIRE_BACK:
            {
                DJ_124_ctrl_vel(&hcan2, 0x200, PID_chassis_vel, data_motor.Chassis_motor_measure, 0, -7200, 0, 0);
                //data_motor.Chassis_motor_measure[1].circle = Circle_STOP(1, ammo_circle);
                //ammo_circle = Circle_STOP(1, ammo_circle);
                //    data_motor.Chassis_motor_measure[1].circle = ammo_circle;

                if (data_motor.Chassis_motor_measure[1].ecd - data_motor.Chassis_motor_measure[1].last_ecd > 4096)
                {
                    ammo_circle--;
                }
                else if (data_motor.Chassis_motor_measure[1].ecd - data_motor.Chassis_motor_measure[1].last_ecd < -4096)
                {
                    ammo_circle++;
                }
                
                if(data_rc.status_ammo == 0)
                {
                    ammo_step = STEP_AMMO_STOP;
                }
                else if (data_rc.status_ammo == 1 || data_rc.status_ammo == 2)
                {
                    if (ammo_circle  <= -50)
                    {
                        ammo_step = STEP_AMMO_CHANGE;
                        ammo_circle = 0;
                    }
                    else
                    {
                        ammo_step = STEP_AMMO_FIRE_BACK;
                    }
                }
                break;
            }
            case STEP_AMMO_CHANGE:
            {
                DJ_124_ctrl_vel(&hcan2, 0x200, PID_chassis_vel, data_motor.Chassis_motor_measure, 0, 0, 7500, 0);
                if(data_rc.status_ammo == 0)
                {
                    ammo_step = STEP_AMMO_STOP;
                }
                else if (data_rc.status_ammo == 1 || data_rc.status_ammo == 2)
                {
                    if (data_motor.Chassis_motor_measure[2].given_current > 4600)
                    {
                        ammo_step = STEP_AMMO_COOL;
                    }
                    else 
                    {
                        ammo_step = STEP_AMMO_CHANGE;
                    }
                }
                break;
            }

            case STEP_AMMO_COOL:
            {
                DJ_124_ctrl_vel(&hcan2, 0x200, PID_chassis_vel, data_motor.Chassis_motor_measure, 0, 0, 0, 0);
                if(data_rc.status_ammo == 0)
                {
                    ammo_step = STEP_AMMO_STOP;
                }
                else if (data_rc.status_ammo == 1 || data_rc.status_ammo == 2)
                {
                    ammo_step = STEP_AMMO_COOL;
                }
                break;
            }
                    
        }
        osDelay(10);
    }
}
