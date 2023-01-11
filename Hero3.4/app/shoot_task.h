/**
  ****************************(C) COPYRIGHT 2022 Delitor****************************
  * @file       shoot_task.c/h
  * @brief      射击任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-8-2022      Delitor         1.shoot_task
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 Delitor****************************
  */
#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H


#include "struct_typedef.h"

#include "main.h"
#include "can_Receive.h"
#include "gimbal_Task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

/****************** 3508 PID值设置 ********************/
#define EXTRA_3508_SPEED_KP								    30.0f
#define EXTRA_3508_SPEED_KI								    1.0f
#define EXTRA_3508_SPEED_KD								    0.0f

#define EXTRA_3508_SPEED_PID_MAXOUT						16000.0f			// 3508最大能发送的电流
#define EXTRA_3508_SPEED_PID_MAXIOUT					20000.0f

/****************** 2006 PID值设置 ********************/
#define MOTOR_2006_SPEED_KP								20.0f
#define MOTOR_2006_SPEED_KI								1.0f
#define MOTOR_2006_SPEED_KD								0.0f

#define MOTOR_2006_SPEED_PID_MAXOUT						6000.0f				// 2006最大能发送的电流
#define MOTOR_2006_SPEED_PID_MAXIOUT					10000.0f

// 电机编码器最大值和最小值( 3508, 2006 )
#define MOTOR_USER_MAX_ENCODER							  8191
#define MOTOR_USER_MIN_ENCODER							  0
#define MOTOR_USER_HALF_ENCODER							  4096

// 3508电机减速比为3591：187，约为19：1，即内圈转动约19圈，外圈转动一圈
// 编码器记录的为内圈的编码值，故外圈转动一圈的编码计数值为 8192*3591/187 = 157313
// 然而程序有点问题，需要前面多转一点，故设置为160000
// 拨弹盘一圈5发弹丸，故一发需转动五分之一圈
#define MOTOR_TRIGGER_ENCODER_A_ROUND						160000
#define MOTOR_TRIGGER_ENCODER_HALF_ROUND			  (MOTOR_TRIGGER_ENCODER_A_ROUND/2)
#define MOTOR_TRIGGER_ENCODER_ONE_BULLET				(MOTOR_TRIGGER_ENCODER_A_ROUND/5)
#define MOTOR_TRIGGER_ENCODER_ONE_SHOOT					(MOTOR_TRIGGER_ENCODER_A_ROUND/5)
#define MOTOR_TRIGGER_ENCODER_ROUND_ERROR				500   //拨弹电机位置误差允许范围，单位为编码器的“1”
#define MOTOR_TRIGGER_ENCODER_COUNT_MAX					19

/****************** 摩擦轮电机的旋转速度 ********************/
#define EXTRA_3508_ROTATE_SPEED_16						  	9000.0f
#define EXTRA_3508_ROTATE_SPEED_10						  	3900.0f

/****************** 拨弹盘电机的旋转速度 ********************/
#define MOTOR_TRIGGER_SLOW_ROTATE_SPEED					400.0f
#define MOTOR_TRIGGER_QUICK_ROTATE_SPEED					1800.0f		// 拨弹盘速度对发射是有影响的，具体尚待探究

/****************** 电机的旋转方向 ********************/
#define EXTRA_3508_L_TURN_DIR							0
#define EXTRA_3508_R_TURN_DIR							0
#define MOTOR_TRIGGER_TURN_DIR					  1

/****************** 模式按键 ********************/
#define CONTROL_GUARD_TO_ATTACK			KEY_PRESSED_OFFSET_Q
#define CONTROL_GUARD_TO_FLEE			KEY_PRESSED_OFFSET_E

/****************** 电机堵转参数 ********************/
#define MOTOR_TRIGGER_LOCKED_ROTOR_ERROR			2
#define MOTOR_TRIGGER_LOCKED_ROTOR_SHORT			400
#define MOTOR_TRIGGER_LOCKED_ROTOR_REVERSAL		200
#define MOTOR_TRIGGER_LOCKED_ROTOR_MAX				5000
#define MOTOR_TRIGGER_LOCKED_ROTOR_MIN				0

#define	NORMAL_MODE 					      0x00			// 正常模式，正常工作
#define	MPU6050_ANGLE_INIT_MODE			0x01			// 陀螺仪初始化
#define	SERVO_ANGLE_SET_MODE			  0x02			// 舵机角度设置

#define MODE_CHANNEL 0  //小车运行模式通道，为遥控器上方右侧拨动开关
#define CTRL_CHANNEL 1  //小车控制方式通道，为遥控器上方左侧拨动开关
                        //拨在上侧键盘控制，拨在中间遥控器控制，拨在下侧时小车不移动只发射

typedef enum
{
	MOTOR_USER_STOP_MODE,	  // 停止
	MOTOR_PRE_MODE,         // 预处理，摩擦轮先转
	MOTOR_REVERSAL_MODE,    // 预处理二阶段，拨弹轮反转一小下
	MOTOR_GET_READY_MODE,	  // 准备射击
	MOTOR_SHOOT_MODE,		    // 射击子弹
	MOTOR_LOCKED_ROTOR,		  // 堵转
	CLEAR_BULLET_PRE_MODE,	// 准备清空所有子弹模块
	CLEAR_BULLET_STA_MODE,	// 开始清空所有子弹模块
}User_Mode_e;

typedef struct
{
	uint16_t given_current;
	fp32 speed;
	fp32 speed_set;
	const motor_measure_t	*extra_3508_measure;
	pid_type_def motor_speed_pid;					
}Extra_3508_Data_t;

typedef struct
{
	int16_t encoder_pos_count;
	int16_t encoder_pos_ecd;
	int16_t encoder_pos_ecd_last;
	int16_t given_current;
	int32_t encoder_pos_all;
	int32_t encoder_pos_all_set;
	fp32 speed;
	fp32 speed_set;
	const motor_measure_t	*motor_3508_measure;
	pid_type_def motor_speed_pid;						       
}TRIGGER_Motor_Data_t;

typedef struct
{
	const RC_ctrl_t *user_motor_RC;              	// 底盘使用的遥控器指针
	Extra_3508_Data_t motor_extra_3508_l;     	  // 左摩擦轮电机数据
	Extra_3508_Data_t motor_extra_3508_r;         // 右摩擦轮电机数据
	TRIGGER_Motor_Data_t motor_trigger;              // 拨弹盘电机，使用的是3508电机
	User_Mode_e motor_mode;                       // 发射模式
	int16_t locked_rotor_time;					          // 拨弹轮堵转时间检测
	int16_t reversal_time;						            // 反转时间计算
}User_Motor_t;

void shoot_task(void const * argument);
void Shoot_Init( User_Motor_t *User_Motor_InitType );
void Shoot_Set_Mode( User_Motor_t *User_Motor_ModeTypedef );
void Extra_3508_Data_Processing( User_Motor_t *User_Motor_DataType );
void Motor_2006_Data_Processing( User_Motor_t *User_Motor_DataType );
void Extra_3508_Speed_PID( User_Motor_t *User_Motor_PIDType );
void Motor_Trigger_PID_Set( User_Motor_t *User_Motor_PIDType );
void Shoot_Feedback_Update(User_Motor_t *User_Motor_PIDType);
void Shoot_Set_Contorl(User_Motor_t *User_Motor_PIDType);
void Shoot_PID_Calc(User_Motor_t *User_Motor_PIDType);

// 扩展板控制代码
void ExBoard_Angle_Init_Mode( CAN_HandleTypeDef* CANx, uint16_t addr );
void ExBoard_Servo_Control_Mode( CAN_HandleTypeDef* CANx, uint16_t addr, uint16_t angle1, uint16_t angle2 );
void CAR_FORCE_RESET( void );

#endif
