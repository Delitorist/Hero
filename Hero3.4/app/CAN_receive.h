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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "main.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2
#define EXTRA_3508_CAN hcan2
#define TRIGGER hcan1

#define AIM_POS_MAX_X							700
#define AIM_POS_MAX_Y							500

/* CAN send and receive ID */
typedef enum
{
  CAN_CHASSIS_ALL_ID = 0x200,
	CAN_GIMBAL_ALL_ID = 0x1FF,
	CAN_USER_MOTOR_ID = 0x200,
	
	
  CAN_3508_M1_ID = 0x201,
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  CAN_3508_M4_ID = 0x204,
	CAN_3508_M9_ID = 0x205,
	//CAN_TRIGGER_MOTOR_ID = 0x206,
	
  CAN_3508_M5_ID = 0x201,
  CAN_3508_M6_ID = 0x202,
	CAN_3508_M7_ID = 0x203,
	CAN_3508_M8_ID = 0x204,
	CAN_YAW_MOTOR_ID = 0x205,
  CAN_PIT_MOTOR_ID = 0x206,
	CAN_PIT_R_MOTOR_ID = 0x207,
	
  CAN_SUPER_CAP_ID = 0x211,

  CAN_GIMBAL_GYRO_ID = 0x527,
  CAN_CHASSIS_GYRO_ID = 0x521,

  CAN_POWER_METER_ID = 0x211,
} can_msg_id_e;

//rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
} motor_measure_t;

//功率检测
typedef struct
{
  fp32 voltage;
  fp32 current;
  fp32 power;
} can_power_meter_t;

//超级电容
typedef struct
{
  fp32 input_vot;
  fp32 cap_vot;
  fp32 input_cur;
  fp32 target_power;
} can_super_cap_t;

// 陀螺仪yaw轴角度扩展
typedef struct
{
	uint8_t count;
	uint8_t count_init;
	uint8_t count_min;
	uint8_t count_max;
	fp32 now;
	fp32 last;
	fp32 all;
	fp32 angle_min;
	fp32 angle_max;
	fp32 angle_middle;
}yaw_angle_exterd_t;

//底盘陀螺仪数据转换所用到的常数
//转换成 m/s^2
#define ACCEL_3G_SEN 0.0008974358974f
#define ACCEL_6G_SEN 0.00179443359375f
#define ACCEL_12G_SEN 0.0035888671875f
#define ACCEL_24G_SEN 0.007177734375f

//转换成 rad/s
#define GYRO_2000_SEN 0.00106526443603169529841533860381f
#define GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define GYRO_500_SEN 0.00026631610900792382460383465095346f
#define GYRO_250_SEN 0.00013315805450396191230191732547673f
#define GYRO_125_SEN 0.000066579027251980956150958662738366f


extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_chassis_reset_ID(void);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_CMD_Extra3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

// 临时的一个函数
void CAN_cmd_temporary(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_r_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_extra_motor_measure_point(uint8_t i);
extern const can_power_meter_t *get_CAN_Power_Meter_Point(void);
extern const can_super_cap_t *get_CAN_Super_Cap_Point(void);
uint16_t *get_Enemy_Pos_x( void );
uint16_t *get_Enemy_Pos_y( void );

void Angle_Extend_Init( yaw_angle_exterd_t *yaw, uint8_t count_init, uint8_t count_min, uint8_t count_max, fp32 angle_min, fp32 angle_max );
void Angle_Extend( yaw_angle_exterd_t *yaw, fp32 input );

extern void CAN_SEND_MESSAGE( CAN_HandleTypeDef *CANx, uint8_t *sendbuf, uint16_t addr );

#endif
