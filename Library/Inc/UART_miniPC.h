#include "main.h"
#include "fifo.h"

#ifndef UART_MINIPC
#define UART_MINIPC


#define UART_MINI_PC_INFO_LEN 12

#define UART_RX_BUF_LENGHT     256
#define UART_MINIPC_FIFO_BUF_LENGTH 512
#define UART_PROTOCOL_FRAME_MAX_SIZE      128


#define HEADER_SOF 0x43
#define HEADER_EOF 0x44

typedef enum 
{
  STEP_SOF = 0,
  STEP_LEN = 1,
  STEP_EOF = 2,
} uart_miniPC_unpack_step_e;

typedef struct __attribute__((packed)) 
{
  uint8_t        protocol_packet[UART_PROTOCOL_FRAME_MAX_SIZE];
  uart_miniPC_unpack_step_e  unpack_step;
  uint16_t       index;
}uart_miniPC_unpack_data_t;

typedef struct __attribute__((packed))
{
  uint8_t sof;            //0x43
  uint8_t aim_ready;
  uint8_t fire_ready;
  uint16_t aim_yaw;
  uint16_t aim_pitch;
  uint16_t aim_x;
  uint16_t aim_y;
  uint8_t eof;            //0x44
}uart_miniPC_info_t;




void uart_miniPC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void uart_miniPC_unpack_fifo_data(void);
void uart_miniPC_read_data(uint8_t *byte);

#endif