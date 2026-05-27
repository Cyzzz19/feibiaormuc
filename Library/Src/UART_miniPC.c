#include "UART_miniPC.h"

fifo_s_t uart_miniPC_fifo;
uint8_t usart1_buf[2][UART_RX_BUF_LENGHT];
uint8_t uart_miniPC_fifo_buf[UART_MINIPC_FIFO_BUF_LENGTH];
uart_miniPC_info_t uart_miniPC_info = {0};
uart_miniPC_unpack_data_t uart_miniPC_unpack_obj = {0};

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;


void uart_miniPC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_LISR_TCIF1);

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
}

void uart_miniPC_unpack_fifo_data(void)
{
    uint8_t byte = 0;
    uart_miniPC_unpack_data_t *p_obj = &uart_miniPC_unpack_obj;
    p_obj->unpack_step = STEP_SOF;
    while(fifo_s_used(&uart_miniPC_fifo))
    {
        byte = fifo_s_get(&uart_miniPC_fifo);
        switch(p_obj->unpack_step)
        {
            case STEP_SOF:
            {
                if(byte == HEADER_SOF)
                {
                    p_obj->index = 0;
                    p_obj->unpack_step = STEP_LEN;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                else
                {
                    p_obj->index = 0;
                }
            }break;

            case STEP_LEN:
            {
                static uint8_t step_time = 0;
                if(step_time <= UART_RX_BUF_LENGHT && p_obj->index == UART_MINI_PC_INFO_LEN)
                {
                    p_obj->unpack_step = STEP_EOF;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                else if(step_time <= UART_RX_BUF_LENGHT && p_obj->index < UART_MINI_PC_INFO_LEN)
                {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                } 
                else
                {
                    p_obj->unpack_step = STEP_SOF;
                    p_obj->index = 0;
                } 
            }break;

            case STEP_EOF:
            {
                if(p_obj->protocol_packet[11] == HEADER_EOF)
                {
                    uart_miniPC_read_data(p_obj->protocol_packet);   
                }
                p_obj->unpack_step = STEP_SOF;
                p_obj->index = 0;
            }break;
        }
    }
}

void uart_miniPC_read_data(uint8_t *frame)
{
    memcpy(&uart_miniPC_info , frame , sizeof(uart_miniPC_info_t));
}

//void USART1_IRQHandler(void)
//{
   // static volatile uint8_t res;
    //if(USART1->SR & UART_FLAG_IDLE)
    //{
       // __HAL_UART_CLEAR_PEFLAG(&huart1);

        //static uint16_t this_time_rx_len = 0;

        //if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        //{
            //__HAL_DMA_DISABLE(huart1.hdmarx);
            //this_time_rx_len = UART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
           // __HAL_DMA_SET_COUNTER(huart1.hdmarx, UART_RX_BUF_LENGHT);
            //huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            //__HAL_DMA_ENABLE(huart1.hdmarx);
            //fifo_s_puts(&uart_miniPC_fifo, (char*)usart1_buf[0], this_time_rx_len);
        //}
        //else
        //{
            //__HAL_DMA_DISABLE(huart1.hdmarx);
            //this_time_rx_len = UART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            //__HAL_DMA_SET_COUNTER(huart1.hdmarx, UART_RX_BUF_LENGHT);
            //huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            //__HAL_DMA_ENABLE(huart1.hdmarx);
            //fifo_s_puts(&uart_miniPC_fifo, (char*)usart1_buf[1], this_time_rx_len);
        //}
    //}
//}



