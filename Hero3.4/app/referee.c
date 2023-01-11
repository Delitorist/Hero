#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "usart.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;

ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_musk_t buff_musk_t;
aerial_robot_energy_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_student_interactive_header_data_t student_interactive_data_t;

void init_referee_struct_data(void)
{
  memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
  memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

  memset(&game_state, 0, sizeof(ext_game_state_t));
  memset(&game_result, 0, sizeof(ext_game_result_t));
  memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));

  memset(&field_event, 0, sizeof(ext_event_data_t));
  memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
  memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
  memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));

  memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
  memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
  memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
  memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
  memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
  memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
  memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
  memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));

  memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_header_data_t));
}

void referee_data_solve(uint8_t *frame)
{
  uint16_t cmd_id = 0;

  uint8_t index = 0;

  memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

  index += sizeof(frame_header_struct_t);

  memcpy(&cmd_id, frame + index, sizeof(uint16_t));
  index += sizeof(uint16_t);

  switch (cmd_id)
  {
  case GAME_STATE_CMD_ID:
  {
    memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
  }
  break;
  case GAME_RESULT_CMD_ID:
  {
    memcpy(&game_result, frame + index, sizeof(game_result));
  }
  break;
  case GAME_ROBOT_HP_CMD_ID:
  {
    memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
  }
  break;

  case FIELD_EVENTS_CMD_ID:
  {
    memcpy(&field_event, frame + index, sizeof(field_event));
  }
  break;
  case SUPPLY_PROJECTILE_ACTION_CMD_ID:
  {
    memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
  }
  break;
  case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
  {
    memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
  }
  break;
  case REFEREE_WARNING_CMD_ID:
  {
    memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
  }
  break;

  case ROBOT_STATE_CMD_ID:
  {
    memcpy(&robot_state, frame + index, sizeof(robot_state));
  }
  break;
  case POWER_HEAT_DATA_CMD_ID:
  {
    memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
  }
  break;
  case ROBOT_POS_CMD_ID:
  {
    memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
  }
  break;
  case BUFF_MUSK_CMD_ID:
  {
    memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
  }
  break;
  case AERIAL_ROBOT_ENERGY_CMD_ID:
  {
    memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
  }
  break;
  case ROBOT_HURT_CMD_ID:
  {
    memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
  }
  break;
  case SHOOT_DATA_CMD_ID:
  {
    memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
  }
  break;
  case BULLET_REMAINING_CMD_ID:
  {
    memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
  }
  break;
  case STUDENT_INTERACTIVE_DATA_CMD_ID:
  {
    memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
  }
  break;
  default:
  {
    break;
  }
  }
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
//  *power = power_heat_data_t.chassis_power;
//  *buffer = power_heat_data_t.chassis_power_buffer;
}

uint8_t get_robot_id(void)
{
  return robot_state.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
//  *heat0_limit = robot_state.shooter_heat0_cooling_limit;
//  *heat0 = power_heat_data_t.shooter_heat0;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
//  *heat1_limit = robot_state.shooter_heat1_cooling_limit;
//  *heat1 = power_heat_data_t.shooter_heat1;
}
/*****************************************************************************************************************************/
#define MAX_SIZE          128    //上传数据最大的长度
#define frameheader_len  5       //帧头长度
#define cmd_len          2       //命令码长度
#define crc_len          2       //CRC16校验
#define Grapic 0;
#define Character 1;


uint8_t seq=0;

void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t mode)
{
	unsigned char i=i;
	uint16_t length;
	uint8_t tx_buff[MAX_SIZE];

	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //数据帧长度	

	memset(tx_buff,0,frame_length);  //存储数据的数组清零
	
	/*****帧头打包*****/
	tx_buff[0] = sof;//数据帧起始字节
	memcpy(&tx_buff[1],(uint8_t*)&len, sizeof(len));//数据帧中data的长度
	tx_buff[3] = seq;//包序号
	append_CRC8_check_sum(tx_buff,frameheader_len);  //帧头校验CRC8

	/*****命令码打包*****/
	memcpy(&tx_buff[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****数据打包*****/
	memcpy(&tx_buff[frameheader_len+cmd_len], p_data, len);
	append_CRC16_check_sum(tx_buff,frame_length);  //一帧数据校验CRC16

	if (seq == 0xff) seq=0;
  else seq++;
	
	/*****数据上传*****/
//	USART_ClearFlag(UART4,USART_FLAG_TC);
	if(mode == 0)
	{
		length=120;
	}
	if(mode == 1)
	{
		length=60;
	}
	HAL_UART_Transmit(&huart6,tx_buff,length,0xffff);
}