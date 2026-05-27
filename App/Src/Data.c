#include "Data.h"

extern fifo_s_t referee_fifo;
extern uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
extern uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
extern data_referee_t data_referee;
extern data_c2g_gimbal_t data_c2g_gimbal;
extern data_motor_t data_motor;
extern data_rc_t data_rc;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

extern int count;

void Data_Task()
{
    can_filter_init();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    referee_Init(usart6_buf[0] , usart6_buf[1] , USART_RX_BUF_LENGHT);
    remote_control_init();
    while(1)
    {
        data_rc.RC_ctrl=get_remote_control_point();
        rc_read_data(&data_rc);
        referee_unpack_fifo_data();

        //usart_printf("%d,%d,%d\n", data_motor.Ammo2_motor_measure[0].given_current, data_motor.Ammo2_motor_measure[0].speed_rpm,count);
        osDelay(10);
    }
    
}
