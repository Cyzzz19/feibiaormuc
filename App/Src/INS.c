#include "INS.h"


data_ins_t data_ins = {0};
float ist8310_data[4];
float gyro[3],accel[3],temp;
float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static int INS_time = 0;
float yaw_D = 0;
float last_yaw = 0;
float yaw = 0;
extern int ammo_circle;
extern data_motor_t data_motor;


void INS_Task(void const * argument)
{   
    //ist8310_init();
    BMI088_init();
    //ist8310_data_read(ist8310_data);
    BMI088_read(gyro , accel , &temp);
    INS_quat[0] = 1.0f;

    while(1)
    {
      //if (data_motor.Chassis_motor_measure[1].ecd - data_motor.Chassis_motor_measure[1].last_ecd > 4096)
      //          {
      //              ammo_circle--;
      //          }
      //          else if (data_motor.Chassis_motor_measure[1].ecd - data_motor.Chassis_motor_measure[1].last_ecd < -4096)
      //          {
      //              ammo_circle++;
      //          }
      /*获取IST8310数据*/
      //ist8310_data_read(ist8310_data);
      //features_orig.ist8310_a = ist8310_data[0];
      //features_orig.ist8310_b = ist8310_data[1];
      //features_orig.ist8310_c = ist8310_data[2];
      //features_orig.temp_ist8310 = ist8310_data[3];
      /*获取IST8310数据*/

      /*获取BMI088数据*/
      BMI088_read(gyro , accel , &temp);
      data_ins.accel_x = accel[0];
      data_ins.accel_y = accel[1];
      data_ins.accel_z = accel[2];
      data_ins.gyro_p = gyro[0];
      data_ins.gyro_y = gyro[1];
      data_ins.gyro_r = gyro[2];
      data_ins.temp_imu = temp;
      /*获取BMI088数据*/

      /*BMI088温控*/
      //IMU_temp_ctrl(PID_imu_temp , temp , 30);
      /*BMI088温控*/ 

      /*四元数姿态解算*/
      
      MahonyAHRSupdateIMU(INS_quat , gyro[0] , gyro[1] , gyro[2] , accel[0] , accel[1] , accel[2]);
      get_angle(INS_quat , &data_ins.yaw , &data_ins.pitch , &data_ins.roll);
      data_ins.yaw = -data_ins.yaw;
      data_ins.pitch = -data_ins.pitch;
      /*四元数姿态解算*/

      if(INS_time >= 10000)
      {
        INS_time = 0;
        yaw = data_ins.yaw;
        yaw_D = yaw - last_yaw;
        last_yaw = yaw;
      }
      else
      {
        INS_time += 1;
      }

      
      osDelay(1);

  }
}