

#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

// 一些延时时间设置
#define GIMBAL_CONTROL_TIME							1
#define GIMBAL_TASK_INIT_TIME						500
// 电机PID参数设置
#define YAW_MOTOR_RELATIVE_BLOCK_BOUNDARY 100
#define YAW_MOTOR_RELATIVE_MAX_OUT			GM6020_MAX_OUTPUT_CURRENT
#define YAW_MOTOR_RELATIVE_MAX_IOUT			(GM6020_MAX_OUTPUT_CURRENT/6)
#define YAW_MOTOR_RELATIVE_KP						40		//10
#define YAW_MOTOR_RELATIVE_KI						0	//0.01
#define YAW_MOTOR_RELATIVE_KD						0.8

#define YAW_MOTOR_SPEED_MAX_OUT					(GM6020_MAX_OUTPUT_CURRENT)
#define YAW_MOTOR_SPEED_MAX_IOUT				(GM6020_MAX_OUTPUT_CURRENT/6)
#define YAW_MOTOR_SPEED_KP							10		//5
#define YAW_MOTOR_SPEED_KI							0	//0.001
#define YAW_MOTOR_SPEED_KD							0

#define YAW_MOTOR_ABSOLUTE_MAX_OUT			GM6020_MAX_OUTPUT_CURRENT
#define YAW_MOTOR_ABSOLUTE_MAX_IOUT			(GM6020_MAX_OUTPUT_CURRENT/3)
#define YAW_MOTOR_ABSOLUTE_KP						10
#define YAW_MOTOR_ABSOLUTE_KI						0.01
#define YAW_MOTOR_ABSOLUTE_KD						0

#define YAW_AUTO_AIMING_POS_KP						15
#define YAW_AUTO_AIMING_POS_KI						0.005
#define YAW_AUTO_AIMING_POS_KD						0
#define YAW_AUTO_AIMING_POS_MAX_IOUT				(GM6020_MAX_OUTPUT_CURRENT/3)
#define YAW_AUTO_AIMING_POS_MAX_OUT					(GM6020_MAX_OUTPUT_CURRENT)

#define YAW_AUTO_AIMING_SPEED_KP					1.5
#define YAW_AUTO_AIMING_SPEED_KI					0
#define YAW_AUTO_AIMING_SPEED_KD					10
#define YAW_AUTO_AIMING_SPEED_MAX_IOUT				(GM6020_MAX_OUTPUT_CURRENT/3)
#define YAW_AUTO_AIMING_SPEED_MAX_OUT				(GM6020_MAX_OUTPUT_CURRENT)

#define YAW_MOTOR_GYRO_ANGLE_KP						2
#define YAW_MOTOR_GYRO_ANGLE_KI						0
#define YAW_MOTOR_GYRO_ANGLE_KD						0
#define YAW_MOTOR_GYRO_ANGLE_MAX_IOUT			(GM6020_MAX_OUTPUT_CURRENT/3)
#define YAW_MOTOR_GYRO_ANGLE_MAX_OUT			(GM6020_MAX_OUTPUT_CURRENT)

#define YAW_MOTOR_GYRO_SPEED_KP					  250
#define YAW_MOTOR_GYRO_SPEED_KI						0
#define YAW_MOTOR_GYRO_SPEED_KD						0
#define YAW_MOTOR_GYRO_SPEED_MAX_IOUT			(GM6020_MAX_OUTPUT_CURRENT/3)
#define YAW_MOTOR_GYRO_SPEED_MAX_OUT			(GM6020_MAX_OUTPUT_CURRENT)

//---------- pitch电机的PID参数设置
#define PITCH_MOTOR_RELATIVE_MAX_OUT			GM6020_MAX_OUTPUT_CURRENT
#define PITCH_MOTOR_RELATIVE_MAX_IOUT			(GM6020_MAX_OUTPUT_CURRENT/3)
#define PITCH_MOTOR_RELATIVE_KP						20
#define PITCH_MOTOR_RELATIVE_KI						0.01
#define PITCH_MOTOR_RELATIVE_KD						0

#define PITCH_MOTOR_ABSOLUTE_MAX_OUT			GM6020_MAX_OUTPUT_CURRENT
#define PITCH_MOTOR_ABSOLUTE_MAX_IOUT			(GM6020_MAX_OUTPUT_CURRENT/3)
#define PITCH_MOTOR_ABSOLUTE_KP						10
#define PITCH_MOTOR_ABSOLUTE_KI						0
#define PITCH_MOTOR_ABSOLUTE_KD						0

#define PITCH_MOTOR_SPEED_MAX_OUT					(GM6020_MAX_OUTPUT_CURRENT)
#define PITCH_MOTOR_SPEED_MAX_IOUT				(GM6020_MAX_OUTPUT_CURRENT/3)
#define PITCH_MOTOR_SPEED_KP						  15
#define PITCH_MOTOR_SPEED_KI						  0.01
#define PITCH_MOTOR_SPEED_KD						  0

#define PITCH_AUTO_AIMING_POS_KP					10
#define PITCH_AUTO_AIMING_POS_KI					0.001
#define PITCH_AUTO_AIMING_POS_KD					0
#define PITCH_AUTO_AIMING_POS_MAX_IOUT				(GM6020_MAX_OUTPUT_CURRENT/3)
#define PITCH_AUTO_AIMING_POS_MAX_OUT				(GM6020_MAX_OUTPUT_CURRENT)

#define PITCH_AUTO_AIMING_SPEED_KP					2
#define PITCH_AUTO_AIMING_SPEED_KI					0
#define PITCH_AUTO_AIMING_SPEED_KD					0
#define PITCH_AUTO_AIMING_SPEED_MAX_IOUT			(GM6020_MAX_OUTPUT_CURRENT/3)
#define PITCH_AUTO_AIMING_SPEED_MAX_OUT				(GM6020_MAX_OUTPUT_CURRENT)

#define PITCH_MOTOR_GYRO_ANGLE_KP					30
#define PITCH_MOTOR_GYRO_ANGLE_KI					0
#define PITCH_MOTOR_GYRO_ANGLE_KD					0
#define PITCH_MOTOR_GYRO_ANGLE_MAX_IOUT		(GM6020_MAX_OUTPUT_CURRENT/3)
#define PITCH_MOTOR_GYRO_ANGLE_MAX_OUT		(GM6020_MAX_OUTPUT_CURRENT)

#define PITCH_MOTOR_GYRO_SPEED_KP					20
#define PITCH_MOTOR_GYRO_SPEED_KI					0
#define PITCH_MOTOR_GYRO_SPEED_KD					0
#define PITCH_MOTOR_GYRO_SPEED_MAX_IOUT		(GM6020_MAX_OUTPUT_CURRENT/3)
#define PITCH_MOTOR_GYRO_SPEED_MAX_OUT		(GM6020_MAX_OUTPUT_CURRENT)
// 旋转模式
#define ABSOLUTE_MODE_POS_KP					    20
#define ABSOLUTE_MODE_POS_KI					    0
#define ABSOLUTE_MODE_POS_KD					    0
#define ABSOLUTE_MODE_POS_MAX_IOUT				(GM6020_MAX_ENCODER/3)
#define ABSOLUTE_MODE_POS_MAX_OUT				  (GM6020_MAX_ENCODER/3)
//yaw,pitch控制通道以及状态开关通道
#define YawChannel 2
#define PitchChannel 3
#define MODE_CHANNEL 0  //小车运行模式通道，为遥控器上方右侧拨动开关
                       //拨在上侧为小陀螺模式，拨在中间为拨在下侧为无控制模式
#define CTRL_CHANNEL 1  //小车控制方式通道，为遥控器上方左侧拨动开关
                       //拨在上侧键盘控制，拨在中间遥控器控制，拨在下侧时无法控制小车
//云台控制相关按键
#define GIMBAL_TOP_MODE_START	KEY_PRESSED_OFFSET_Q
#define GIMBAL_TOP_MODE_CLOSE	KEY_PRESSED_OFFSET_E
#define CONTROL_BULLET_BIN_KEY	KEY_PRESSED_OFFSET_F
// 电机最大输出控制电流
#define GM6020_MAX_OUTPUT_CURRENT					30000
// 电机编码器最大值和最小值
#define GM6020_MAX_ENCODER							  8191
#define GM6020_HALF_ENCODER							  4096
#define GM6020_MIN_ENCODER							  0
// 数据扩展相关参数(小陀螺功能调试好了，这些参数不敢改动了，虽然理论上随便改问题不大)
#define EXTERND_DATA_COUNT_INIT							3		//1
#define EXTERND_DATA_COUNT_MIN							1
#define EXTERND_DATA_COUNT_MAX							6		//3
// 数据扩展相关参数2.0(小陀螺2.0功能调试好了，这些参数不敢改动了，虽然理论上随便改问题不大)
#define EXTERND_DATA2_COUNT_INIT						3		//1
#define EXTERND_DATA2_COUNT_MIN							1
#define EXTERND_DATA2_COUNT_MAX							6		//3
// 小陀螺模式下，云台限制移动距离
#define TOP_YAW_MIN_ECD									10000
#define TOP_YAW_MAX_ECD									49000
// 遥控控制电机旋转的速度
#define RC_CONTROL_YAW_ROTATE_SEPPD				0.003f
#define RC_CONTROL_PITCH_ROTATE_SEPPD			0.003f
#define MOUSE_CONTROL_YAW_ROTATE_SPEED		0.02f
#define MOUSE_CONTROL_PITCH_ROTATE_SPEED	0.05f
// 电机初始角度
#define YAW_INIT_ENCODER							    1550
#define PITCH_INIT_ENCODER							  3000
// 电机相对底盘旋转的角度范围设置
#define YAW_RELATIVE_INIT_ANGLE						2640
#define YAW_RELATIVE_MAX_ANGLE						5000
#define YAW_RELATIVE_MIN_ANGLE						1000
#define PITCH_RELATIVE_INIT_ANGLE					3000
#define PITCH_RELATIVE_MAX_ANGLE					3285
#define PITCH_RELATIVE_MIN_ANGLE					2290 
#define PITCH_RELATIVE_INIT_ANGLE_R				8192-PITCH_RELATIVE_INIT_ANGLE
#define PITCH_RELATIVE_MAX_ANGLE_R				8192-PITCH_RELATIVE_MAX_ANGLE
#define PITCH_RELATIVE_MIN_ANGLE_R				8192-PITCH_RELATIVE_MIN_ANGLE
// 电机控制旋转方向设置
#define YAW_REMOTE_ROTATE_DIR						  1
#define YAW_MOUSE_ROTATE_DIR						  1
#define PITCH_REMOTE_ROTATE_DIR						1
#define PITCH_MOUSE_ROTATE_DIR						0
// 自瞄设置
#define AIM_POS_MAX_X							700
#define AIM_POS_MAX_Y							500
#define AIM_POS_X								320
#define AIM_POS_Y								240
#define AUTO_AIMING_DELAY_TIME					100

//--------------------------------------------------------
/**********************鼠标右键减速***********************/
#define GIMBAL_YAW_SLOW_DOWN_START			8.0f
#define GIMBAL_PITCH_SLOW_DOWN_START		4.0f
#define GIMBAL_SLOW_DOWN_END				1.0f

typedef enum
{
  GIMBAL_MOTOR_NO_CONTROL = 0, 	// 电机没有任何控制
  GIMBAL_MOTOR_ABSOLUTE_MODE,		// 云台小陀螺模式，绝对地面静止
  GIMBAL_MOTOR_RELATIVE_MODE,		// 云台相对车身可移动

  GIMBAL_MOTOR_ENCODER_MODE,		// 云台编码器位置环
  GIMBAL_MOTOR_ENCODER_LOCK_MODE,	// 云台电机锁住编码器的位置不动
  GIMBAL_MOTOR_AUTO_AIMING,			// 云台自瞄模式
	
  GIMBAL_MOTOR_TEST_MODE,			// 测试模式

  GIMBAL_MOTOR_GYRO,    			//电机陀螺仪角度控制
  GIMBAL_MOTOR_ENCONDE, 			//电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
  fp32 kp;
  fp32 ki;
  fp32 kd;

  fp32 set;
  fp32 get;
  fp32 err;

  fp32 max_out;
  fp32 max_iout;

  fp32 Pout;
  fp32 Iout;
  fp32 Dout;

  fp32 out;
} Gimbal_PID_t;

typedef struct
{
	int16_t count;
	int16_t now;
	int16_t last;
	int16_t all;
}externd_encoder_t;

typedef struct
{
	fp32 count;
	fp32 now;
	fp32 last;
	fp32 all;
}externd_gyro_t;

typedef struct
{
	short int count;
	fp32 now;
	fp32 last;
	fp32 all;
	fp32 all_last;
}externd_angle_t;

typedef struct
{
  const motor_measure_t *gimbal_motor_measure;
  Gimbal_PID_t gimbal_motor_absolute_angle_pid;
  Gimbal_PID_t gimbal_motor_relative_angle_pid;
  Gimbal_PID_t gimbal_motor_gyro_mode_angle_pid;
  pid_type_def gimbal_motor_gyro_mode_speed_pid;
  pid_type_def auto_aiming_mode_pos_pid;
  pid_type_def auto_aiming_mode_speed_pid;
  pid_type_def gimbal_motor_gyro_pid;
  gimbal_motor_mode_e gimbal_motor_mode;
  gimbal_motor_mode_e last_gimbal_motor_mode;
  uint16_t offset_ecd;
  fp32 max_relative_angle;
  fp32 min_relative_angle;

  fp32 relative_angle;
  fp32 relative_angle_set;

  fp32 absolute_angle_stable; //rad
  fp32 absolute_ecd_stable; //ecd
  fp32 absolute_ecd;     //rad
  fp32 absolute_ecd_set; //rad
  
  fp32 enemy_pos;
  fp32 enemy_pos_set;
  
  fp32 motor_gyro;         //rad/s
  fp32 motor_gyro_set;
  fp32 motor_speed;
  fp32 raw_cmd_current;
  fp32 current_set;
  int16_t given_current;
} Gimbal_Motor_t;

typedef struct
{
  fp32 max_yaw;
  fp32 min_yaw;
  fp32 max_pitch;
  fp32 min_pitch;
  uint16_t max_yaw_ecd;
  uint16_t min_yaw_ecd;
  uint16_t max_pitch_ecd;
  uint16_t min_pitch_ecd;
  uint8_t step;
} Gimbal_Cali_t;

typedef struct
{
  const RC_ctrl_t *gimbal_rc_ctrl;
  const fp32 *gimbal_INT_angle_point;
  const fp32 *gimbal_INT_gyro_point;
  Gimbal_Motor_t gimbal_yaw_motor;
  Gimbal_Motor_t gimbal_pitch_motor;
  Gimbal_Cali_t gimbal_cali;
  pid_type_def gimbal_absolute_mode_pos_pid;

  fp32 *gimbal_yaw_gyro;		// 云台yaw角速度
  fp32 *gimbal_pitch_gyro;	    // 云台pitch角速度
  fp32 *gimbal_roll_gyro;		// 云台roll角速度
  fp32 *gimbal_yaw_angle;		// 云台yaw角度
  fp32 *gimbal_pitch_angle;		// 云台pitch角度
  fp32 *gimbal_roll_angle;		// 云台roll角度

  uint8_t bullet_bin_sta;		// 弹仓状态
  uint16_t bullet_bin_servo_angle;	// 弹仓舵机角度
  externd_encoder_t yaw_absolute_encoder_pos;	// yaw轴数据扩展
  externd_angle_t yaw_absolute_angle_pos;		// yaw轴数据扩展
 
  fp32 *chassis_yaw_angle;		// 底盘角度yaw
  fp32 *chassis_pitch_angle;	// 底盘角度pitch
  fp32 *chassis_roll_angle;		// 底盘角度roll
  fp32 *chassis_yaw_gyro;		// 底盘角速度yaw
  fp32 *chassis_pitch_gyro;	// 底盘角速度pitch
  fp32 *chassis_roll_gyro;	// 底盘角速度roll
	
	// 底盘陀螺仪
  fp32 chassis_yaw;				// C板自带yaw轴陀螺仪
  fp32 chassis_pitch;			// C板自带pitch轴陀螺仪
  fp32 chassis_roll;			// C板自带roll轴陀螺仪
  
  uint16_t *enemy_x;	// 敌人坐标x
  uint16_t *enemy_y;	// 敌人坐标y
  uint16_t enemy_xy_refresh_times;
  
} Gimbal_Control_t;


extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);
void GIMBAL_Init( Gimbal_Control_t *gimbal_type );
void gimbal_task(void const *pvParameters);
void AnyAngle_Spread( float *angle, float max, float min );


#endif

