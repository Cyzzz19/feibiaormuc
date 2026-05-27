#include "CAN_receive.h"

extern data_motor_t data_motor;
extern data_g2c_gimbal_t data_g2c_gimbal;
extern data_g2c_chassis_t data_g2c_chassis;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern int16_t ammo_circle;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {

            case CAN_AMMO1_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo1_motor_measure[0] , rx_data);break;
            case CAN_AMMO2_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo1_motor_measure[1] , rx_data);break;
            case CAN_AMMO3_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo1_motor_measure[2] , rx_data);break;
            case CAN_AMMO4_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo1_motor_measure[3] , rx_data);break;
            //case CAN_AMMO5_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo2_motor_measure[0] , rx_data);break;  
            case CAN_AMMO6_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo2_motor_measure[1] , rx_data);break;
            //case CAN_chassis_M1_ID:    DJ_motor_fbdata(&data_motor.Chassis_motor_measure[0] , rx_data);break;
            //case CAN_chassis_M2_ID:    DJ_motor_fbdata(&data_motor.Chassis_motor_measure[1] , rx_data);break;
            //case CAN_chassis_M3_ID:    DJ_motor_fbdata(&data_motor.Chassis_motor_measure[2] , rx_data);break;
            //case CAN_chassis_M4_ID:    DJ_motor_fbdata(&data_motor.Chassis_motor_measure[3] , rx_data);break;

            default: break;
        }
    }
    else if(hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
            //case CAN_AMMO1_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo1_motor_measure[0] , rx_data);break;
            //case CAN_AMMO2_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo1_motor_measure[1] , rx_data);break;
            //case CAN_AMMO3_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo1_motor_measure[2] , rx_data);break;
            //case CAN_AMMO4_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo1_motor_measure[3] , rx_data);break;
            //case CAN_AMMO5_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo2_motor_measure[0] , rx_data);break;  
            //case CAN_AMMO6_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo2_motor_measure[1] , rx_data);break;

            case CAN_chassis_M1_ID:    DJ_motor_fbdata(&data_motor.Chassis_motor_measure[0] , rx_data);break;
            case CAN_chassis_M2_ID:    DJ_motor_fbdata(&data_motor.Chassis_motor_measure[1] , rx_data);break;
            case CAN_chassis_M3_ID:    DJ_motor_fbdata(&data_motor.Chassis_motor_measure[2] , rx_data);break;
            case CAN_chassis_M4_ID:    DJ_motor_fbdata(&data_motor.Chassis_motor_measure[3] , rx_data);break;


            case CAN_AMMO5_MOTOR_ID:    DJ_motor_fbdata(&data_motor.Ammo2_motor_measure[0] , rx_data);break;  
        }
        //ammo_circle = Circle_STOP(1, ammo_circle);
        if (data_motor.Chassis_motor_measure[1].ecd - data_motor.Chassis_motor_measure[1].last_ecd > 4096)
        {
            ammo_circle--;
        }
        else if (data_motor.Chassis_motor_measure[1].ecd - data_motor.Chassis_motor_measure[1].last_ecd < -4096)
        {
            ammo_circle++;
        }
    }
}

/**
  * @brief can过滤器初始化
  * @return none 
  */
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}