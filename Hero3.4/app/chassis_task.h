

#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

//选择底盘状态 开关通道号
#define ModeChannel 0
#define CtrlChannel 1
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN (x_y_move_len)		// 10.0f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN (-x_y_move_len)		// 10.0f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN (-x_y_move_len)		// 10.0f

// 电脑控制旋转比例
#define CHASSIS_VX_KEYBOARD_SEN		(-x_y_move_len)
#define CHASSIS_VY_KEYBOARD_SEN		(-x_y_move_len)
#define CHASSIS_WZ_MOUSE_SEN 		90.0f
#define CHASSIS_WZ_ANGLE_MOUSE_SEN 	10//0.1//0.024f//0.008f
#define CHASSIS_WZ_ANGLE_ERROR		0.2f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

#define CHASSIS_RC_DEADLINE 10
#define CHASSIS_MOUSE_DEADLINE 1

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 1.0f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 1.0f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 1.0f

#define MOTOR_DISTANCE_TO_CENTER 1.0f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
//#define CHASSIS_ROTATE_L_KEY KEY_PRESSED_OFFSET_Q
//#define CHASSIS_ROTATE_R_KEY KEY_PRESSED_OFFSET_E
#define CHASSIS_TOP_MODE_START	KEY_PRESSED_OFFSET_Q
#define CHASSIS_TOP_MODE_CLOSE	KEY_PRESSED_OFFSET_E

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 1.0f			//0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//底盘电机最大速度
//#define MAX_WHEEL_SPEED 4.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED	1000.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 1000.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1000.0f
//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 20

// 遥控参数
#define REMOTE_MAX_VALUE		160

// 低电压保护
#define CHASSIS_MIN_VOLTAGE		17.0f

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 10.0f
#define M3505_MOTOR_SPEED_PID_KI 0.5f
#define M3505_MOTOR_SPEED_PID_KD -1.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT MAX_MOTOR_CAN_CURRENT

//底盘鼠标控制旋转PID
#define CHASSIS_MOUSE_PID_KP 20.0f
#define CHASSIS_MOUSE_PID_KI 0.0f
#define CHASSIS_MOUSE_PID_KD 0.0f
#define CHASSIS_MOUSE_PID_MAX_OUT REMOTE_MAX_VALUE
#define CHASSIS_MOUSE_PID_MAX_IOUT REMOTE_MAX_VALUE

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 70.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.0f

//--------------------------------------------------------
/**********************功率限制参数***********************/
//最大限制功率 
#define SUPER_CUP_INPUT_VOLTAGE_MIN		18.0f				// 超级电容模块最小的输入电压
#define SUPER_CUP_INPUT_VOLTAGE_WARNING	22.0f				// 超级电容模块警告电压
#define SUPER_CAP_INPUT_VOLTAGE_ERROR	1.5f				// (SUPER_CUP_INPUT_VOLTAGE_WARNING - SUPER_CUP_INPUT_VOLTAGE_MIN)

#define CHASSIS_SPEED_BUFFER_PRA1	0.01f
#define CHASSIS_SPEED_BUFFER_PRA2	0.99f
#define CHASSIS_MIN_BUFFER_VALUE	5.0f
#define CHASSIS_BUFFER_INCREMENT	3

//--------------------------------------------------------
/**********************鼠标右键减速***********************/
#define CHASSIS_VX_MAX						5000
#define CHASSIS_VY_MAX						5000
#define CHASSIS_WZ_MAX						5000

//--------------------------------------------------------
/**********************鼠标右键减速***********************/
#define CHASSIS_SLOW_DOWN_START				2.0f
//#define CHASSIS_PITCH_SLOW_DOWN_START		4.0f
#define CHASSIS_SLOW_DOWN_END				1.0f

//--------------------------------------------------------
/**********************底盘yaw扩展***********************/
#define CHASSIS_YAW_EXTEND_COUNT_MIN		1
#define CHASSIS_YAW_EXTEND_COUNT_MAX		6
#define CHASSIS_YAW_EXTEND_COUNT_INIT		3

//--------------------------------------------------------
/**********************其他参数***********************/
#define CHASSIS_ANGLE_RETURN_ERROR	0.3
#define CHASSIS_ANGLE_ROTOAT_ERROR	0.01
#define CHASSIS_TOP_ROTATE_SPEED	700
#define CHASSIS_VECTOR_TOP_CLOSE_ERROR 5 //小陀螺结束回正时，判定回正到位的允许误差值


typedef enum
{	
  CHASSIS_VECTOR_NO_CONTROL,
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
  CHASSIS_VECTOR_NO_FOLLOW_YAW,
  CHASSIS_VECTOR_NO_FOLLOW_YAW_END,
  CHASSIS_VECTOR_TOP_START_MODE,
  CHASSIS_VECTOR_TOP_CLOSE_MODE,

  CHASSIS_VECTOR_RAW,
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,
} chassis_mode_e;

typedef enum
{
	//SUPER_CAP_NO_CONNECT,	// 超级电容充电模式
	SUPER_CAP_OPEN,			// 超级电容开启模式
	SUPER_CAP_CLOSE,		// 超级电容关闭模式
	SUPER_CAP_OVER,			// 超级电容过放
	SUPER_CAP_CHARGE,		// 超级电容充电模式
	SUPER_CAP_BEGIN,
} super_cap_sta_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  fp32 give_current;
} Chassis_Motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
  const Gimbal_Motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
  const Gimbal_Motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
  const fp32 *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
  chassis_mode_e chassis_mode;               //底盘控制状态机
  chassis_mode_e last_chassis_mode;          //底盘上次控制状态机
  Chassis_Motor_t motor_chassis[4];          //底盘电机数据
  pid_type_def motor_speed_pid[4];             //底盘电机速度pid
  pid_type_def chassis_mouse_control_pid;        //底盘鼠标控制pid
  pid_type_def chassis_angle_pid;              //底盘跟随角度pid

  const can_power_meter_t *chassis_power_meter;	// 功率测量
  const can_super_cap_t *chassis_super_cap;	// 超级电容

	uint16_t super_cap_setting;		// 超级电容功率设置
	super_cap_sta_e super_cap_sta;	// 超级电容状态设置
	fp32 super_cap_voltage_min;     //超级电容最小电压值
	
	/***
	****/
	fp32 super_cap_soc;				// 超级电容SOC，这个是剩余的能量
	fp32 super_cap_per;				// 超级电容的百分比，这个是相对于满电剩余的能量
//  fp32 super_cap_voltage_warning;	// 超级电容警告电压值
//	fp32 super_cap_voltage_diff;	// 超级电容电压差

  fp32 vx;                         //底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                         //底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
  fp32 wz_angle_set;               //鼠标控制时要滑动多少角度

  fp32 top_mode_dir;				// 小陀螺模式时的方向，角度值

  fp32 *gimbal_mpu6050_yaw_gyro;		// 扩展板云台陀螺仪yaw
  fp32 *gimbal_mpu6050_pitch_gyro;	// 扩展板云台陀螺仪pitch
  fp32 *gimbal_mpu6050_roll_gyro;		// 扩展板云台陀螺仪roll
  fp32 gimbal_mpu6050_yaw;		// 扩展板云台陀螺仪yaw
  fp32 *gimbal_mpu6050_pitch;		// 扩展板云台陀螺仪pitch
  fp32 *gimbal_mpu6050_roll;		// 扩展板云台陀螺仪roll

	// 以下为扩展
	//externd_angle_t
//  fp32 chassis_yaw;				// C板自带yaw轴陀螺仪
//  fp32 chassis_pitch;			// C板自带pitch轴陀螺仪
//  fp32 chassis_roll;			// C板自带roll轴陀螺仪

  fp32 chassis_mpu6050_yaw;		// 扩展板底盘陀螺仪yaw
  fp32 *chassis_mpu6050_pitch;	// 扩展板底盘陀螺仪pitch
  fp32 *chassis_mpu6050_roll;		// 扩展板底盘陀螺仪roll
  fp32 *chassis_mpu6050_yaw_gyro;		// 扩展板底盘陀螺仪yaw
  fp32 *chassis_mpu6050_pitch_gyro;	// 扩展板底盘陀螺仪pitch
  fp32 *chassis_mpu6050_roll_gyro;	// 扩展板底盘陀螺仪roll
} chassis_move_t;

extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);


#endif
