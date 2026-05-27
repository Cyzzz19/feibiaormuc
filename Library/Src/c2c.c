#include "c2c.h"

data_g2c_gimbal_t data_g2c_gimbal = {0};  //云台发送到底盘数据
data_g2c_chassis_t data_g2c_chassis = {0};  //云台接收自底盘数据
data_c2g_gimbal_t data_c2g_gimbal = {0};  


void g2c_chassis_fbdata(data_g2c_chassis_t *c2c , uint8_t *rx_data)
{
  c2c->c2c_info_chassis_status = (rx_data)[0];
  c2c->c2c_info_x_vel = (uint16_t)((rx_data)[1] << 8 | (rx_data)[2]);
  c2c->c2c_info_y_vel = (uint16_t)((rx_data)[3] << 8 | (rx_data)[4]);
  c2c->c2c_info_yaw_vel = (uint16_t)((rx_data)[5] << 8 | (rx_data)[6]);
}

void g2c_gimbal_fbdata(data_g2c_gimbal_t *c2c , uint8_t *rx_data)
{
  c2c->c2c_info_gimbal_status = (rx_data)[0];
  c2c->c2c_info_ammo_status = (rx_data)[1];
  c2c->c2c_info_yaw_aim_pos = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]);
  c2c->c2c_info_yaw_fre_pos = (uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);
}

void c2g_gimbal_fbdata(data_c2g_gimbal_t *c2c , uint8_t *rx_data)
{
  c2c->c2c_info_gimbal_power = (rx_data)[0];
  c2c->c2c_info_ammo_power = (rx_data)[1];
}


void g2c_send_data(CAN_HandleTypeDef *hcan)
{
    static uint8_t tx_data[8];

    tx_data[0] = data_g2c_gimbal.c2c_info_gimbal_status;
    tx_data[1] = data_g2c_gimbal.c2c_info_ammo_status;
    tx_data[2] = data_g2c_gimbal.c2c_info_yaw_aim_pos >> 8;
    tx_data[3] = data_g2c_gimbal.c2c_info_yaw_aim_pos;
    tx_data[4] = data_g2c_gimbal.c2c_info_yaw_fre_pos >> 8;
    tx_data[5] = data_g2c_gimbal.c2c_info_yaw_fre_pos;
    tx_data[6] = 6;
    tx_data[7] = 7;
    can_send_data(hcan , G2C_GIMBAL_ID , tx_data , 8);
    osDelay(1);
    tx_data[0] = data_g2c_chassis.c2c_info_chassis_status;
    tx_data[1] = data_g2c_chassis.c2c_info_x_vel >> 8;
    tx_data[2] = data_g2c_chassis.c2c_info_x_vel;
    tx_data[3] = data_g2c_chassis.c2c_info_y_vel >> 8;
    tx_data[4] = data_g2c_chassis.c2c_info_y_vel;
    tx_data[5] = data_g2c_chassis.c2c_info_yaw_vel >> 8;
    tx_data[6] = data_g2c_chassis.c2c_info_yaw_vel;
    tx_data[7] = 7;
    can_send_data(hcan , G2C_CHASSIS_ID , tx_data , 8);
}

void c2g_send_data(CAN_HandleTypeDef *hcan)
{
    static uint8_t tx_data[8];

    tx_data[0] = data_c2g_gimbal.c2c_info_gimbal_power;
    tx_data[1] = data_c2g_gimbal.c2c_info_ammo_power;
    tx_data[2] = 2;
    tx_data[3] = 3;
    tx_data[4] = 4;
    tx_data[5] = 5;
    tx_data[6] = 6;
    tx_data[7] = 7;
    can_send_data(hcan , C2G_GIMBAL_ID , tx_data , 8);
}


/**
  * @brief          can发数据
  * @param hcan     can位（&hcan1/&hcan2）
  * @param id       can id
  * @param data     发送数据
  * @param len      数据长度
  * @return         none 
  */
uint8_t can_send_data(CAN_HandleTypeDef *hcan , uint16_t id , uint8_t *data , uint32_t len)
{	
	CAN_TxHeaderTypeDef	tx_header;
	
	tx_header.StdId = id;
	tx_header.ExtId = 0;
	tx_header.IDE   = 0;
	tx_header.RTR   = 0;
	tx_header.DLC   = len;
	if(HAL_CAN_AddTxMessage(hcan , &tx_header , data , (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
    {
		if(HAL_CAN_AddTxMessage(hcan , &tx_header , data , (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) 
        {
			HAL_CAN_AddTxMessage(hcan , &tx_header , data , (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
    return 0;
}