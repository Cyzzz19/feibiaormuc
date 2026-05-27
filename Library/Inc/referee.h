#include "main.h"
#include "crc_ref.h"
#include "string.h"
#include "fifo.h"

#ifndef REFEREE_H
#define REFEREE_H


#define USART_RX_BUF_LENGHT     256
#define REFEREE_FIFO_BUF_LENGTH 512

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)  // 5?????
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))




typedef  struct __attribute__((packed)) 
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;


typedef enum 
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} referee_unpack_step_e;


typedef struct __attribute__((packed)) 
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  referee_unpack_step_e  unpack_step;
  uint16_t       index;
} referee_unpack_data_t;


/*裁判系统回包结构体*/

/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef  struct __attribute__((packed))
{ 
	uint8_t game_type; 
	uint8_t game_progress; 
	uint16_t stage_remain_time; 
	uint64_t SyncTimeStamp; 
} ext_game_state_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct __attribute__((packed))
{
	uint8_t winner;
} ext_game_result_t;

/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */
typedef struct __attribute__((packed))
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct __attribute__((packed))
{
	uint32_t event_type;
} ext_event_data_t;

/* ID: 0X0201  Byte: 13    机器人状态数据 */
typedef struct __attribute__((packed))
{
	uint8_t robot_id; 
	uint8_t robot_level; 
	uint16_t current_HP; 
	uint16_t maximum_HP; 
	uint16_t shooter_barrel_cooling_value; 
	uint16_t shooter_barrel_heat_limit; 
	uint16_t chassis_power_limit; 
	uint8_t power_management_gimbal_output : 1; 
	uint8_t power_management_chassis_output : 1; 
	uint8_t power_management_ammo_output : 1; 
} ext_game_robot_state_t;

/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef struct __attribute__((packed))
{
	uint16_t chassis_voltage;                   //寄辣
	uint16_t chassis_current;                   //寄辣
	float chassis_power;                        //寄辣
	uint16_t buffer_energy; 
	uint16_t shooter_17mm_1_barrel_heat; 
	uint16_t shooter_17mm_2_barrel_heat; 
	uint16_t shooter_42mm_barrel_heat; 
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef struct __attribute__((packed))
{
	float x;
	float y;
	float yaw;
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  6    机器人增益数据 */
typedef struct __attribute__((packed))
{
	uint8_t recovery_buff; 
	uint8_t cooling_buff; 
	uint8_t defence_buff; 
	uint8_t vulnerability_buff; 
	uint8_t attack_buff; 
    uint16_t remaining_energy;
} ext_buff_musk_t;

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef struct __attribute__((packed))
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef struct __attribute__((packed))
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;

/* ID: 0x0309  Byte:  -    自定义控制器发包 */
typedef struct __attribute__((packed))
{ 
    uint8_t data[100]; 
}custom_robot_data_t;



/* ID: 0x0104  Byte:  3    判罚等级 */
typedef struct __attribute__((packed)) 
{ 
 uint8_t level; 
 uint8_t offending_robot_id; 
 uint8_t count; 
}referee_warning_t;

/* ID: 0x0105  Byte:  3    飞镖发射情况 */
typedef struct __attribute__((packed)) 
{ 
 uint8_t dart_remaining_time; 
 uint16_t dart_info; 
}dart_info_t;

/* ID: 0x0208  Byte:  8    允许发弹量 */
typedef struct __attribute__((packed)) 
{ 
 uint16_t projectile_allowance_17mm; 
 uint16_t projectile_allowance_42mm; 
 uint16_t remaining_gold_coin; 
 uint16_t projectile_allowance_fortress; 
}projectile_allowance_t;

/* ID: 0x0209  Byte:  5    增益点 */
typedef struct __attribute__((packed))
{ 
 uint32_t rfid_status; 
 uint8_t rfid_status_2; 
}rfid_status_t;

/* ID: 0x020A  Byte:  6    飞镖云台手 */
typedef struct __attribute__((packed)) 
{ 
 uint8_t dart_launch_opening_status; 
 uint8_t reserved; 
 uint16_t target_change_time; 
 uint16_t latest_launch_cmd_time; 
}dart_client_cmd_t;

/* ID: 0x020B  Byte:  36    机器人位置 */
typedef struct __attribute__((packed)) 
{ 
 float hero_x; 
 float hero_y; 
 float engineer_x; 
 float engineer_y; 
 float standard_3_x; 
 float standard_3_y; 
 float standard_4_x; 
 float standard_4_y; 
 float reserved_01; 
 float reserved_02; 
}ground_robot_position_t;

/* ID: 0x020C  Byte:  2    易伤情况*/
typedef struct __attribute__((packed))
{ 
 uint16_t mark_progress; 
}radar_mark_data_t;

/* ID: 0x020D  Byte:  6    烧饼姿态*/
typedef struct __attribute__((packed)) 
{ 
 uint32_t sentry_info; 
 uint16_t sentry_info_2; 
} sentry_info_t;

/* ID: 0x020E  Byte:  1    雷达易伤*/
typedef struct __attribute__((packed)) 
{ 
 uint8_t radar_info; 
} radar_info_t;


typedef struct __attribute__((packed))
{
    frame_header_struct_t FrameHeader;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;                            // 0x0101
	referee_warning_t RefereeWarning;					   // 0x0104
	dart_info_t DartInfo;                                  // 0x0105
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	ext_power_heat_data_t PowerHeatData;				   // 0x0202
	ext_game_robot_pos_t GameRobotPos;					   // 0x0203
	ext_buff_musk_t BuffMusk;							   // 0x0204
	ext_robot_hurt_t RobotHurt;							   // 0x0206
	ext_shoot_data_t ShootData;							   // 0x0207
	projectile_allowance_t ProjectileAllowance;            // 0x0208
	rfid_status_t RfidStatus;                              // 0x0209
	dart_client_cmd_t DartClient;                          // 0x020A
	ground_robot_position_t GroundRobotPosition;           // 0x020B
	radar_mark_data_t RadarMark;                           // 0x020C
	sentry_info_t SentryInfo;                              // 0x020D需要
	radar_info_t RadarInfo;                                // 0x020E
	custom_robot_data_t ReceiveData;                 // 0x0309

	int init_flag;

} data_referee_t;


/*裁判系统回包结构体*/
void referee_unpack_fifo_data(void);
void referee_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void referee_read_data(uint8_t *frame);
#endif