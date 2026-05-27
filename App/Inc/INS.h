#ifndef INS_H
#define INS_H
#include "main.h"
#include "BMI088.h"
#include "Ammo.h"
#include "Motor.h"
#include "stdbool.h"
#include "MahonyAHRS.h"

/**
  * @brief                  C板获取的原始数据信息
  * @param  key             按键状态
  * @param  ist8310_a       IST8310原始数据1
  * @param  ist8310_b       IST8310原始数据2
  * @param  ist8310_c       IST8310原始数据3
  * @param  temp_ist8310    IST8310温度
  * @param  accel_x         加速度X 
  * @param  accel_y         加速度Y
  * @param  accel_z         加速度Z
  * @param  gyro_p          角速度Pitch
  * @param  gyro_y          角速度Yaw
  * @param  gyro_r          角速度Row
  * @param  temp_imu        IMU温度
  * @param  pitch           Pitch角度
  * @param  yaw             Yaw角度
  * @param  roll            Roll角度
  */
 typedef struct
 {
     bool key;
     float ist8310_a;
     float ist8310_b;
     float ist8310_c;
     float temp_ist8310;
     float accel_x;
     float accel_y;
     float accel_z;
     float gyro_p;
     float gyro_y;
     float gyro_r;
     float temp_imu;
     float pitch;
     float yaw;
     float roll;
 }data_ins_t;

void INS_Task(void const * argument);

#endif