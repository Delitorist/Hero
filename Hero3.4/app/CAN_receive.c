/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"

#include "detect_task.h"
#include "string.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
//云台电机数据读取( 6020 )
#define get_gimbal_motor_measure(ptr, data)                              		\
    {                                                                          		\
        (ptr)->last_ecd = (ptr)->ecd;                                           	\
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);         				\
        (ptr)->speed_rpm = (uint16_t)(uint16_t)((data)[2] << 8 | (data)[3]); 		\
        (ptr)->given_current = (uint16_t)(uint16_t)((data)[4] << 8 | (data)[5]);    \
        (ptr)->temperate = (data)[6];                                              	\
    }


motor_measure_t motor_yaw, motor_pit, motor_pit_r, motor_trigger, motor_chassis[4], motor_extra[4];
float gimbal_gyro[3], chassis_gyro[3], gimbal_angle[3], chassis_angle[3];
uint16_t enemy_x, enemy_y;
uint8_t enemy_refresh_flag;

//yaw_angle_exterd_t chassis_yaw;
//yaw_angle_exterd_t gimbal_yaw;
	
static can_power_meter_t can_power_meter;
static can_super_cap_t can_super_cap;
	
static yaw_angle_exterd_t gimbal_yaw;
static yaw_angle_exterd_t chassis_yaw;

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static CAN_TxHeaderTypeDef extra_tx_message;
static uint8_t extra_can_send_data[8];
uint8_t exboard_data_feedback = 0;

	// 使用这个函数有问题，不知道为什么
//static void CAN_Power_Meter(CAN_HandleTypeDef *can_rx, uint8_t *rx_data)
//{
//  can_power_meter.voltage = (fp32)((uint32_t)(rx_data[1] << 8) | (rx_data[0])) / 100.0;
//  can_power_meter.current = (fp32)((uint32_t)(rx_data[3] << 8) | (rx_data[2])) / 100.0;
//  can_power_meter.power = can_power_meter.voltage * can_power_meter.current;
//}

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	if( hcan == &hcan1 )	// 如果是CAN1
	{
		switch (rx_header.StdId)
		{
//			case  CAN_POWER_METER_ID:
//			{
//				can_power_meter.voltage = (fp32)((uint32_t)(rx_data[1] << 8) | (rx_data[0])) / 100.0f;
//				can_power_meter.current = (fp32)((uint32_t)(rx_data[3] << 8) | (rx_data[2])) / 100.0f;
//				can_power_meter.power = can_power_meter.voltage * can_power_meter.current;
//				break;
//			}
			case CAN_SUPER_CAP_ID://经过测试，使用的超级电容和功率计ID是一样的，超级电容的id
			{
				fp32 detect = (float)(rx_data[7] << 8 | rx_data[6]) / 100.0f;
					if(detect>30)
					{
					can_super_cap.input_vot = (float)(rx_data[1] << 8 | rx_data[0]) / 100.0f;
					can_super_cap.cap_vot = (float)(rx_data[3] << 8 | rx_data[2]) / 100.0f;
					can_super_cap.input_cur = (float)(rx_data[5] << 8 | rx_data[4]) / 100.0f;
					can_super_cap.target_power = detect;
					}
//				break;
//				}
//			case CAN_POWER_METER_ID:
//			{
					else
					{
					can_power_meter.voltage = (fp32)((uint32_t)(rx_data[1] << 8) | (rx_data[0])) / 100.0f;
					can_power_meter.current = (fp32)((uint32_t)(rx_data[3] << 8) | (rx_data[2])) / 100.0f;
					can_power_meter.power = can_power_meter.voltage * can_power_meter.current;
					}
				break;
			}
				
				
			
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
//			case CAN_3508_M5_ID:
//			case CAN_3508_M6_ID:
			{
				static uint8_t i = 0;
				i = rx_header.StdId - CAN_3508_M1_ID;
				get_motor_measure(&motor_chassis[i], rx_data);
				break;
			}
//			case CAN_YAW_MOTOR_ID:
//			{
//				get_gimbal_motor_measure(&motor_yaw, rx_data);
//				break;
//			}
//			case CAN_PIT_MOTOR_ID:
//			{
//				get_gimbal_motor_measure(&motor_pit, rx_data);
//				break;
//			}  
//			case CAN_TRIGGER_MOTOR_ID://电机id统一处理
//			{
//				get_motor_measure(&motor_trigger, rx_data);
//				break;
//			}  
			case CAN_CHASSIS_GYRO_ID://底盘那个陀螺仪
			{
				chassis_angle[0] = (short)(rx_data[1] << 8 | rx_data[0]) / 32768.0 * 180;
				chassis_angle[1] = (short)(rx_data[3] << 8 | rx_data[2]) / 32768.0 * 180;
				chassis_angle[2] = (short)(rx_data[5] << 8 | rx_data[4]) / 32768.0 * 180;
		//	  if( chassis_angle[0]>0 )			chassis_angle[0] =chassis_angle[0] - 180;
		//	  else if( chassis_angle[0]<0 )			chassis_angle[0] = 180 + chassis_angle[0];
				//Angle_Extend( &chassis_yaw, chassis_angle[2] );				
				break;
			}
			case CAN_CHASSIS_GYRO_ID+1:
			{
				chassis_gyro[0] = (short)(rx_data[1] << 8 | rx_data[0]) / 32768.0 * 2000;
				chassis_gyro[1] = (short)(rx_data[3] << 8 | rx_data[2]) / 32768.0 * 2000;
				chassis_gyro[2] = (short)(rx_data[5] << 8 | rx_data[4]) / 32768.0 * 2000;
				break;
			}
//			case CAN_SUPER_CAP_ID:
//			{
//				can_super_cap.input_vot = (float)(rx_data[1] << 8 | rx_data[0]) / 100.0f;
//				can_super_cap.cap_vot = (float)(rx_data[3] << 8 | rx_data[2]) / 100.0f;
//				can_super_cap.input_cur = (float)(rx_data[5] << 8 | rx_data[4]) / 100.0f;
//				can_super_cap.target_power = (float)(rx_data[7] << 8 | rx_data[6]) / 100.0f;
//				break;
//			}
			case CAN_3508_M9_ID://电机id统一处理
			{
				get_motor_measure(&motor_trigger, rx_data);
				break;
			}
			
//			case CAN_GIMBAL_GYRO_ID://云台陀螺仪
//			{
//				gimbal_angle[0] = (short)(rx_data[1] << 8 | rx_data[0]) / 32768.0 * 180;
//				gimbal_angle[1] = (short)(rx_data[3] << 8 | rx_data[2]) / 32768.0 * 180;
//				gimbal_angle[2] = (short)(rx_data[5] << 8 | rx_data[4]) / 32768.0 * 180;
//		//	   if( gimbal_angle[0]>0 )			gimbal_angle[0] =gimbal_angle[0] - 180;
//		//	  else if( gimbal_angle[0]<0 )			gimbal_angle[0] = 180 + gimbal_angle[0];
//				break;
//			}
//			case CAN_GIMBAL_GYRO_ID+1:
//			{
//				gimbal_gyro[0] = (short)(rx_data[1] << 8 | rx_data[0]) / 32768.0 * 2000;
//				gimbal_gyro[1] = (short)(rx_data[3] << 8 | rx_data[2]) / 32768.0 * 2000;
//				gimbal_gyro[2] = (short)(rx_data[5] << 8 | rx_data[4]) / 32768.0 * 2000;
//				break;
//			}
			//采集完毕
			default:
			{
				break;
			}
		}
	}
	else if( hcan == &hcan2 )	// 如果是CAN2
	{
		switch (rx_header.StdId)
		{
//			case  CAN_POWER_METER_ID:
//			{
//				can_power_meter.voltage = (fp32)((uint32_t)(rx_data[1] << 8) | (rx_data[0])) / 100.0;
//			can_power_meter.current = (fp32)((uint32_t)(rx_data[3] << 8) | (rx_data[2])) / 100.0;
//			can_power_meter.power = can_power_meter.voltage * can_power_meter.current;
//				break;
//			}
//			case CAN_3508_M1_ID:
//			case CAN_3508_M2_ID:
//			case CAN_3508_M3_ID:
//			case CAN_3508_M4_ID:
			case CAN_3508_M5_ID:
			case CAN_3508_M6_ID:
			case CAN_3508_M7_ID:
			case CAN_3508_M8_ID:				
			{
				static uint8_t i = 0;
				i = rx_header.StdId - CAN_3508_M5_ID;
				get_motor_measure(&motor_extra[i], rx_data);
				break;
			}
			case CAN_YAW_MOTOR_ID:
			{
				int16_t rpm_last = motor_yaw.speed_rpm;
				get_gimbal_motor_measure(&motor_yaw, rx_data);
				if( rpm_last - motor_yaw.speed_rpm > 4 || rpm_last - motor_yaw.speed_rpm < -4 )
					motor_yaw.speed_rpm = rpm_last;
				break;
			}
			case CAN_PIT_MOTOR_ID:
			{
				get_gimbal_motor_measure(&motor_pit, rx_data);
				break;
			}
			case CAN_PIT_R_MOTOR_ID:
			{
				get_gimbal_motor_measure(&motor_pit_r, rx_data);
				break;
			} 			
//			case CAN_CHASSIS_GYRO_ID://底盘那个陀螺仪
//			{
//				chassis_angle[0] = (short)(rx_data[1] << 8 | rx_data[0]) / 32768.0 * 180;
//				chassis_angle[1] = (short)(rx_data[3] << 8 | rx_data[2]) / 32768.0 * 180;
//				chassis_angle[2] = (short)(rx_data[5] << 8 | rx_data[4]) / 32768.0 * 180;
//		//	  if( chassis_angle[0]>0 )			chassis_angle[0] =chassis_angle[0] - 180;
//		//	  else if( chassis_angle[0]<0 )			chassis_angle[0] = 180 + chassis_angle[0];
//				break;
//			}
//			case CAN_CHASSIS_GYRO_ID+1:
//			{
//				chassis_gyro[0] = (short)(rx_data[1] << 8 | rx_data[0]) / 32768.0 * 2000;
//				chassis_gyro[1] = (short)(rx_data[3] << 8 | rx_data[2]) / 32768.0 * 2000;
//				chassis_gyro[2] = (short)(rx_data[5] << 8 | rx_data[4]) / 32768.0 * 2000;
//				break;
//			}
//			case CAN_SUPER_CAP_ID://超级电容的id
//			{
//				can_super_cap.input_vot = (float)(rx_data[1] << 8 | rx_data[0]) / 100.0f;
//				can_super_cap.cap_vot = (float)(rx_data[3] << 8 | rx_data[2]) / 100.0f;
//				can_super_cap.input_cur = (float)(rx_data[5] << 8 | rx_data[4]) / 100.0f;
//				can_super_cap.target_power = (float)(rx_data[7] << 8 | rx_data[6]) / 100.0f;
//				break;
//			}
			case CAN_GIMBAL_GYRO_ID://云台陀螺仪
			{
				gimbal_angle[0] = (short)(rx_data[1] << 8 | rx_data[0]) / 32768.0 * 180;
				gimbal_angle[1] = (short)(rx_data[3] << 8 | rx_data[2]) / 32768.0 * 180;
				gimbal_angle[2] = (short)(rx_data[5] << 8 | rx_data[4]) / 32768.0 * 180;
				//Angle_Extend( &gimbal_yaw, gimbal_angle[2] );
		//	   if( gimbal_angle[0]>0 )			gimbal_angle[0] =gimbal_angle[0] - 180;
		//	  else if( gimbal_angle[0]<0 )			gimbal_angle[0] = 180 + gimbal_angle[0];
				break;
			}
			case CAN_GIMBAL_GYRO_ID+1:	//陀螺仪数据
			{
				gimbal_gyro[0] = (short)(rx_data[1] << 8 | rx_data[0]) / 32768.0 * 2000;
				gimbal_gyro[1] = (short)(rx_data[3] << 8 | rx_data[2]) / 32768.0 * 2000;
				gimbal_gyro[2] = (short)(rx_data[5] << 8 | rx_data[4]) / 32768.0 * 2000;
				break;
			}
			case CAN_GIMBAL_GYRO_ID+2:	//自瞄数据
			{
				uint16_t temp_x, temp_y;
				enemy_refresh_flag = 1;
				temp_x = (short)(rx_data[1] << 8 | rx_data[0]);
				temp_y = (short)(rx_data[3] << 8 | rx_data[2]);
				if( (temp_x <= AIM_POS_MAX_X) && (temp_x >= 0) )	enemy_x = temp_x;
				if( (temp_y <= AIM_POS_MAX_Y) && (temp_y >= 0) )	enemy_y = temp_y;
				break;
			}
			//采集完毕
			default:
			{
				break;
			}
		}
	}
  
}
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)  CAN2
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch_r: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = (rev >> 8);
  gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204) CAN1
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
//发送额外电机的控制电流到电机 CAN2
void CAN_CMD_Extra3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  extra_tx_message.StdId = CAN_USER_MOTOR_ID;
  extra_tx_message.IDE = CAN_ID_STD;
  extra_tx_message.RTR = CAN_RTR_DATA;
  extra_tx_message.DLC = 0x08;
  extra_can_send_data[0] = motor1 >> 8;
  extra_can_send_data[1] = motor1;
  extra_can_send_data[2] = motor2 >> 8;
  extra_can_send_data[3] = motor2;
  extra_can_send_data[4] = motor3 >> 8;
  extra_can_send_data[5] = motor3;
  extra_can_send_data[6] = motor4 >> 8;
  extra_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&EXTRA_3508_CAN, &extra_tx_message, extra_can_send_data, &send_mail_box);
}
//CAN1
void CAN_cmd_temporary(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&TRIGGER, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
//这里要写一个can发送命令的函数
void CAN_SEND_MESSAGE( CAN_HandleTypeDef *hcan, uint8_t *sendbuf, uint16_t addr )
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef TxMessage;
  TxMessage.StdId = addr;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.DLC = 0x08;

  HAL_CAN_AddTxMessage(hcan, &TxMessage, sendbuf, &send_mail_box);
}

// 角度扩展初始化函数
void Angle_Extend_Init( yaw_angle_exterd_t *yaw, uint8_t count_init, uint8_t count_min, uint8_t count_max, fp32 angle_min, fp32 angle_max )
{
	yaw->count_init = count_init;
	yaw->count = yaw->count_init;
	yaw->count_min = count_min;
	yaw->count_max = count_max;
	yaw->now = 0;
	yaw->last = 0;
	yaw->all = 0;
	yaw->angle_min = angle_min;
	yaw->angle_max = angle_max;
	yaw->angle_middle = (yaw->angle_max - yaw->angle_min)/2;
}
// 角度扩展函数
void Angle_Extend( yaw_angle_exterd_t *yaw, fp32 input )
{
	yaw->now = input;
	if( (yaw->now - yaw->last) > yaw->angle_middle )		yaw->count--;
	else if( (yaw->now - yaw->last) < (-yaw->angle_middle) )yaw->count++;
	
	if( yaw->count > yaw->count_max )		yaw->count -= (yaw->count_max - yaw->count_min);
	else if( yaw->count < yaw->count_min )	yaw->count += (yaw->count_max - yaw->count_min);
	
	yaw->last = yaw->now;
	yaw->all = yaw->now + yaw->count*( yaw->angle_max - yaw->angle_min );
}
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_yaw;
}

/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_pit;
}

const motor_measure_t *get_pitch_gimbal_r_motor_measure_point(void)
{
  return &motor_pit_r;
}

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_trigger;
}

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x0f)];
}

// 返回摩擦轮3508电机的指针
const motor_measure_t *get_extra_motor_measure_point(uint8_t i)
{
  return &motor_extra[(i & 0x0f)];
}

//返回功率计参数的指针
const can_power_meter_t *get_CAN_Power_Meter_Point( void )
{
  return( &can_power_meter );
}
//返回超级电容参数的指针
const can_super_cap_t *get_CAN_Super_Cap_Point( void )
{
  return( &can_super_cap );
}

// 返回敌人坐标
uint16_t *get_Enemy_Pos_x( void )
{
	return( &enemy_x );
}
uint16_t *get_Enemy_Pos_y( void )
{
	return( &enemy_y );
}
