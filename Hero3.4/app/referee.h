#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"
#include "protocol.h"

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
typedef __packed struct //0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} ext_game_state_t;

typedef __packed struct //0002
{
    uint8_t winner;
} ext_game_result_t;
typedef __packed struct
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
typedef __packed struct //0101
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;
typedef __packed struct//0x201
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_id1_17mm_cooling_rate;
	uint16_t shooter_id1_17mm_cooling_limit;
	uint16_t shooter_id1_17mm_speed_limit;
	uint16_t shooter_id2_17mm_cooling_rate;
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;
	uint16_t shooter_id1_42mm_cooling_rate;
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct //0x0202
{
    uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

typedef __packed struct //0x0205
{
    uint8_t energy_point;
    uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct //0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct
{
uint8_t graphic_name[3];
uint32_t operate_tpye:3;
uint32_t graphic_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t radius:10;
uint32_t end_x:11;
uint32_t end_y:11;
} graphic_data_struct_t;

typedef __packed struct
{
uint8_t graphic_name[3];
uint32_t operate_tpye:3;
uint32_t graphic_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t num;
} number_data_struct_t;



typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

typedef __packed struct
{
number_data_struct_t number_data_struct[7];
} ext_client_custom_number_seven_t;

typedef __packed struct
{
graphic_data_struct_t grapic_data_struct;
uint8_t data[30];
} ext_client_custom_character_t;//绘制字符
/*
typedef struct
{ 
   uint8_t graphic_name[3]; 
   uint32_t operate_tpye:3; 
   uint32_t graphic_tpye:3; 
   uint32_t layer:4; 
   uint32_t color:4; 
   uint32_t start_angle:9;
   uint32_t end_angle:9;
   uint32_t width:10; 
   uint32_t start_x:11; 
   uint32_t start_y:11;
   float graph_Float;              //浮点数据
} ext_client_custom_float_t;
*/

typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t sender_ID;
uint16_t receiver_ID;
ext_client_custom_graphic_seven_t graphic_custom;//自定义图形数据
	
}ext_student_interactive_header_data_t;

typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t sender_ID;
uint16_t receiver_ID;
ext_client_custom_character_t character_custom;//自定义字符数据
	
}ext_student_interactive_header_data_t_char;

typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t sender_ID;
uint16_t receiver_ID;
ext_client_custom_number_seven_t number_custom;//自定义数字数据
	
}ext_student_interactive_header_data_t_num;
/*
typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t sender_ID;
uint16_t receiver_ID;
ext_client_custom_float_t float_custom;//自定义浮点数据
}ext_student_interactive_header_data_t_float;
*/
typedef __packed struct
{
    uint32_t data1;
    uint32_t data2;
    uint32_t data3;
    uint32_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;

typedef __packed struct
{
	frame_header_struct_t   txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t FrameHeader;//数据段头
	custom_data_t  					clientData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_SendClientData_t;



typedef __packed struct
{
uint8_t data[50];
} robot_interactive_data_t;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);
extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

extern uint8_t get_robot_id(void);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);

void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len,uint8_t mode);
void referee_data_pack_handle_character(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len);
#endif
