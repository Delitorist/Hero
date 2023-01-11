#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "struct_typedef.h"
#include "math.h"
#include "vision_protocol.h"

#include "main.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "INS_task.h"
#include "pid.h"
#include "shoot_task.h"
#include "usb_task.h"
#include "can.h"
#include "referee.h"
#define rc_deadline_limit(input, output, dealine)  \
{                                                  \
	if ((input) > (dealine) || (input) < -(dealine)) \
	{                                                \
			(output) = (input);                          \
	}                                                \
	else                                             \
	{                                                \
			(output) = 0;                                \
	}                                                \
}

//这个是6020的电机编码值规整8191
#define ECD_Format(ecd)    \
{                          \
  if ((ecd) > ecd_range)   \
    (ecd) -= ecd_range;    \
  else if ((ecd) < 0)      \
    (ecd) += ecd_range;    \
}

#define gimbal_total_pid_clear(gimbal_clear)                                             \
{                                                                                        \
	Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
	Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
	PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
																																												 \
	Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
	Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
	PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
}

gimbal_motor_mode_e gimbal_yaw_mode;
Gimbal_Control_t gimbal_control;//云台控制结构体
int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0, Shoot_Can_Set_Current = 0;
extern float gimbal_gyro[3], chassis_gyro[3], gimbal_angle[3], chassis_angle[3];
extern fp32 INS_angle[3];
extern fp32 temp_gimbal_yaw;
extern uint8_t enemy_refresh_flag;
// 给云台提示，底盘回归时云台为相关模式
volatile extern uint8_t chassis_return_mode;

//会用到的函数
static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);
static void GIMBAL_Mode_Set( Gimbal_Control_t *gimbal_mode_set );
static void GIMBAL_Data_Updata( Gimbal_Control_t *gimbal_updata );
static void GIMBAL_Data_Processing( Gimbal_Control_t *gimbal_data );
static void GIMBAL_Set_Control( Gimbal_Control_t *gimbal_set_control );
static void GIMBAL_PID_Cal(Gimbal_Control_t *gimbal_control_loop);
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
static void Gimbal_Death( Gimbal_Control_t *gimbal_death );
static void Gimbal_Tim( Gimbal_Control_t *gimbal_tim );

extern uint8_t exboard_data_feedback;
yaw_angle_exterd_t yaw_gyro_pos;		// yaw轴数据扩展

// 裁判系统数据
extern ext_game_robot_state_t robot_state;
extern fp32 temp_chassis_yaw;

// 鼠标右键按下以后，所有东西都要减速运行
fp32 gimbal_yaw_slow_down_rate = GIMBAL_SLOW_DOWN_END, gimbal_pitch_slow_down_rate = GIMBAL_SLOW_DOWN_END;

//fp32 data_buf1[1000] = { 0 };
//uint16_t data_i = 0;
	
void gimbal_task(void const *pvParameters)
{
  vTaskDelay(GIMBAL_TASK_INIT_TIME);
  GIMBAL_Init(&gimbal_control);
  while (1)
  {
	  if (robot_state.remain_HP)	// 还活着 robot_state.remain_HP
	  {
			GIMBAL_Mode_Set(&gimbal_control);				    // 云台模式设置
			GIMBAL_Data_Updata(&gimbal_control);				// 云台数据更新
			GIMBAL_Set_Control(&gimbal_control);				// 将鼠标和遥控的值赋值进入
			GIMBAL_PID_Cal(&gimbal_control);					  // 云台控制PID计算

			Yaw_Can_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
			Pitch_Can_Set_Current = gimbal_control.gimbal_pitch_motor.given_current;

			//yaw轴电机为hcan2 1号，Pitch轴电机为hcan2 2号
			CAN_cmd_gimbal(Yaw_Can_Set_Current, Pitch_Can_Set_Current, 0, 0);
	  }
	  else	// 不好意思您已经壮烈牺牲
	  {
		  Gimbal_Death(&gimbal_control);
	  }
    vTaskDelay(1);
  }
}

void GIMBAL_Init( Gimbal_Control_t *gimbal_init )
{
  float Yaw_speed_pid[3] = { YAW_MOTOR_SPEED_KP, YAW_MOTOR_SPEED_KI, YAW_MOTOR_SPEED_KD };
  float yaw_gyro_speed_pid[3] = { YAW_MOTOR_GYRO_SPEED_KP, YAW_MOTOR_GYRO_SPEED_KI, YAW_MOTOR_GYRO_SPEED_KD };
  float Pitch_speed_pid[3] = { PITCH_MOTOR_SPEED_KP, PITCH_MOTOR_SPEED_KI, PITCH_MOTOR_SPEED_KD };
  float Absolute_mode_pos_pid[3] = { ABSOLUTE_MODE_POS_KP, ABSOLUTE_MODE_POS_KI, ABSOLUTE_MODE_POS_KD };
  float yaw_auto_aiming_pos_pid[3] = { YAW_AUTO_AIMING_POS_KP, YAW_AUTO_AIMING_POS_KI, YAW_AUTO_AIMING_POS_KD };
  float yaw_auto_aiming_speed_pid[3] = { YAW_AUTO_AIMING_SPEED_KP, YAW_AUTO_AIMING_SPEED_KI, YAW_AUTO_AIMING_SPEED_KD };
  float pitch_auto_aiming_pos_pid[3] = { PITCH_AUTO_AIMING_POS_KP, PITCH_AUTO_AIMING_POS_KI, PITCH_AUTO_AIMING_POS_KD };
  float pitch_auto_aiming_speed_pid[3] = { PITCH_AUTO_AIMING_SPEED_KP, PITCH_AUTO_AIMING_SPEED_KI, PITCH_AUTO_AIMING_SPEED_KD };
  
  if( gimbal_init == NULL )						return;

  //电机数据指针获取
  gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
  gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
	//陀螺仪数据指针获取
  //	gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
  //	gimbal_init->gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();
  //遥控器数据指针获取
  gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
  // 弹仓状态变量初始化
  gimbal_init->bullet_bin_sta = 0;
  gimbal_init->bullet_bin_servo_angle = 1500;
  // 获取敌人坐标
  gimbal_init->enemy_x = get_Enemy_Pos_x();
  *gimbal_init->enemy_x = AIM_POS_X;
  gimbal_init->enemy_y = get_Enemy_Pos_y();
  *gimbal_init->enemy_y = AIM_POS_Y;
  gimbal_init->gimbal_yaw_motor.enemy_pos_set = AIM_POS_X;
  gimbal_init->gimbal_pitch_motor.enemy_pos_set = AIM_POS_Y;
  // 电机模式初始化
  gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = gimbal_init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_NO_CONTROL;
  gimbal_init->gimbal_pitch_motor.gimbal_motor_mode = gimbal_init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_NO_CONTROL;
	// yaw 电机PID初始化
  GIMBAL_PID_Init( &gimbal_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_MOTOR_RELATIVE_MAX_OUT, YAW_MOTOR_RELATIVE_MAX_IOUT,
                   YAW_MOTOR_RELATIVE_KP, YAW_MOTOR_RELATIVE_KI, YAW_MOTOR_RELATIVE_KD );
  GIMBAL_PID_Init( &gimbal_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_MOTOR_ABSOLUTE_MAX_OUT, YAW_MOTOR_ABSOLUTE_MAX_IOUT,
                   YAW_MOTOR_ABSOLUTE_KP, YAW_MOTOR_ABSOLUTE_KI, YAW_MOTOR_ABSOLUTE_KD );
  PID_init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_MOTOR_SPEED_MAX_OUT, YAW_MOTOR_SPEED_MAX_IOUT);
  PID_init(&gimbal_init->gimbal_yaw_motor.auto_aiming_mode_pos_pid, PID_POSITION, yaw_auto_aiming_pos_pid, YAW_AUTO_AIMING_POS_MAX_OUT, 
			YAW_AUTO_AIMING_POS_MAX_IOUT);
  PID_init(&gimbal_init->gimbal_yaw_motor.auto_aiming_mode_speed_pid, PID_POSITION, yaw_auto_aiming_speed_pid, YAW_AUTO_AIMING_SPEED_MAX_OUT, 
			YAW_AUTO_AIMING_SPEED_MAX_IOUT);
  GIMBAL_PID_Init( &gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_mode_angle_pid, YAW_MOTOR_GYRO_ANGLE_MAX_OUT, YAW_MOTOR_GYRO_ANGLE_MAX_IOUT,
                   YAW_MOTOR_GYRO_ANGLE_KP, YAW_MOTOR_GYRO_ANGLE_KI, YAW_MOTOR_GYRO_ANGLE_KD);
  PID_init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_mode_speed_pid, PID_POSITION, yaw_gyro_speed_pid, YAW_MOTOR_GYRO_SPEED_MAX_OUT, YAW_MOTOR_GYRO_SPEED_MAX_IOUT);
  // pitch电机PID初始化设置
  GIMBAL_PID_Init( &gimbal_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_MOTOR_RELATIVE_MAX_OUT, PITCH_MOTOR_RELATIVE_MAX_IOUT,
                   PITCH_MOTOR_RELATIVE_KP, PITCH_MOTOR_RELATIVE_KI, PITCH_MOTOR_RELATIVE_KD );
  GIMBAL_PID_Init( &gimbal_init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_MOTOR_ABSOLUTE_MAX_OUT, PITCH_MOTOR_ABSOLUTE_MAX_IOUT,
                   PITCH_MOTOR_ABSOLUTE_KP, PITCH_MOTOR_ABSOLUTE_KI, PITCH_MOTOR_ABSOLUTE_KD );
  PID_init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_MOTOR_SPEED_MAX_OUT, PITCH_MOTOR_SPEED_MAX_IOUT);
  PID_init(&gimbal_init->gimbal_pitch_motor.auto_aiming_mode_pos_pid, PID_POSITION, pitch_auto_aiming_pos_pid, PITCH_AUTO_AIMING_POS_MAX_OUT, 
			PITCH_AUTO_AIMING_POS_MAX_IOUT);
  PID_init(&gimbal_init->gimbal_pitch_motor.auto_aiming_mode_speed_pid, PID_POSITION, pitch_auto_aiming_speed_pid, PITCH_AUTO_AIMING_SPEED_MAX_OUT, 
			PITCH_AUTO_AIMING_SPEED_MAX_IOUT);
//  GIMBAL_PID_Init( &gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_mode_angle_pid, PITCH_MOTOR_GYRO_ANGLE_MAX_OUT, PITCH_MOTOR_GYRO_ANGLE_MAX_IOUT,
//                   PITCH_MOTOR_GYRO_ANGLE_KP, PITCH_MOTOR_GYRO_ANGLE_KI, PITCH_MOTOR_GYRO_ANGLE_KD );
//  PID_init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_mode_speed_pid, PID_POSITION, Pitch_gyro_speed_pid, PITCH_MOTOR_GYRO_SPEED_MAX_OUT, PITCH_MOTOR_GYRO_SPEED_MAX_IOUT);

  // 旋转模式PID
  PID_init(&gimbal_init->gimbal_absolute_mode_pos_pid, PID_POSITION, Absolute_mode_pos_pid, ABSOLUTE_MODE_POS_MAX_IOUT, ABSOLUTE_MODE_POS_MAX_OUT);
  // 将所有PID参数归零
  gimbal_total_pid_clear(gimbal_init);

  // 将YAW电机角度初始化
  gimbal_init->gimbal_yaw_motor.absolute_ecd = gimbal_init->gimbal_yaw_motor.gimbal_motor_measure->ecd;
  gimbal_init->gimbal_yaw_motor.absolute_ecd_set = YAW_RELATIVE_INIT_ANGLE;
  gimbal_init->gimbal_yaw_motor.relative_angle = gimbal_init->gimbal_yaw_motor.gimbal_motor_measure->ecd;
  gimbal_init->gimbal_yaw_motor.relative_angle_set = YAW_RELATIVE_INIT_ANGLE;
	
  // 将PITCH电机角度初始化
  gimbal_init->gimbal_pitch_motor.absolute_ecd = gimbal_init->gimbal_pitch_motor.gimbal_motor_measure->ecd;
  gimbal_init->gimbal_pitch_motor.absolute_ecd_set = PITCH_RELATIVE_INIT_ANGLE;
  gimbal_init->gimbal_pitch_motor.relative_angle = gimbal_init->gimbal_pitch_motor.gimbal_motor_measure->ecd;
  gimbal_init->gimbal_pitch_motor.relative_angle_set = PITCH_RELATIVE_INIT_ANGLE;

  // 电机旋转角度限位
  gimbal_init->gimbal_yaw_motor.max_relative_angle = gimbal_init->gimbal_yaw_motor.gimbal_motor_measure->ecd + YAW_RELATIVE_MAX_ANGLE;
  gimbal_init->gimbal_yaw_motor.min_relative_angle = gimbal_init->gimbal_yaw_motor.gimbal_motor_measure->ecd - YAW_RELATIVE_MIN_ANGLE;
  gimbal_init->gimbal_pitch_motor.max_relative_angle = PITCH_RELATIVE_MAX_ANGLE;
  gimbal_init->gimbal_pitch_motor.min_relative_angle = PITCH_RELATIVE_MIN_ANGLE;

  // 云台使用扩展板的陀螺仪MPU6050
  gimbal_init->gimbal_pitch_angle = &gimbal_angle[0];
  gimbal_init->gimbal_roll_angle = &gimbal_angle[1];
  //gimbal_init->gimbal_yaw_angle = &gimbal_angle[2];
  gimbal_init->gimbal_pitch_gyro = &gimbal_gyro[0];
  gimbal_init->gimbal_roll_gyro = &gimbal_gyro[1];
  gimbal_init->gimbal_yaw_gyro = &gimbal_gyro[2];

  // 底盘使用C板自带陀螺仪，由于底盘自带陀螺仪是弧度制，后面有算式将其转化为角度制
  
  gimbal_init->chassis_yaw = INS_angle[0];
  gimbal_init->chassis_pitch = INS_angle[2];
  gimbal_init->chassis_roll = INS_angle[1];
//  gimbal_init->chassis_pitch_angle = &chassis_angle[0];
//  gimbal_init->chassis_roll_angle = &chassis_angle[1];
//  gimbal_init->chassis_yaw_angle = &temp_chassis_yaw;
//  gimbal_init->chassis_pitch_gyro = &chassis_gyro[0];
//  gimbal_init->chassis_roll_gyro = &chassis_gyro[1];
//  gimbal_init->chassis_yaw_gyro = &chassis_gyro[2];
  
  // 陀螺仪角度初始化
	
  //ExBoard_Angle_Init_Mode( &hcan2, 0x520 );
	gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
	gimbal_init->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
	gimbal_init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
	gimbal_init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
	gimbal_init->gimbal_yaw_motor.relative_angle_set = YAW_RELATIVE_INIT_ANGLE;
	gimbal_init->gimbal_pitch_motor.relative_angle_set = PITCH_RELATIVE_INIT_ANGLE;
  //............................	/****** 其他代码稍后再加 *****/
}

static void GIMBAL_Mode_Set( Gimbal_Control_t *gimbal_mode_set )
{
  if( gimbal_mode_set == NULL )  return;

	static uint16_t chassis_no_flow_yaw_mode_count = 0;
  /************* 遥控修改云台模式 **************/
	if( switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CTRL_CHANNEL]) )
	{
		if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODE_CHANNEL]))
	  {
			gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_NO_CONTROL;
			gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_NO_CONTROL;
	  }
	  else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODE_CHANNEL]))
	  {
			gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
			gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
	  }
	  else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODE_CHANNEL]))
	  {
			//gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_LOCK_MODE;
			gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
			gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
	  }
	}
	else if( switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CTRL_CHANNEL]) )
	{
//		if( gimbal_mode_set->gimbal_rc_ctrl->key.v & GIMBAL_TOP_MODE_START )	// 小陀螺模式开启
//		{
//			gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
//			gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
//		}
		if( gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E )
		{
			chassis_no_flow_yaw_mode_count++;
			if (chassis_no_flow_yaw_mode_count == 20)
			{
				if (gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode != GIMBAL_MOTOR_ENCODER_MODE) 
				{
					gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
					gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
				}
				else 
				{
					gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_LOCK_MODE;
					gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
				}
			}
		}
		else 
		{
			chassis_no_flow_yaw_mode_count = 0;
		}
		
		if( gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r )	// 减速
		{
			gimbal_yaw_slow_down_rate = GIMBAL_YAW_SLOW_DOWN_START;
			gimbal_pitch_slow_down_rate = GIMBAL_PITCH_SLOW_DOWN_START;
		}
		else
		{
			gimbal_yaw_slow_down_rate = GIMBAL_SLOW_DOWN_END;
			gimbal_pitch_slow_down_rate = GIMBAL_SLOW_DOWN_END;
		}
	}
	else if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CTRL_CHANNEL]))
	{
		if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODE_CHANNEL]))
		{
			gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_LOCK_MODE;
			gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_LOCK_MODE;
		}
		else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODE_CHANNEL]))
		{
			gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
			gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
		}
	}

  if( chassis_return_mode == 1 )
  {
	  gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
  }
  else if( chassis_return_mode == 2 )
  {
	  gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_LOCK_MODE;
	  chassis_return_mode = 0;
  }

	gimbal_yaw_mode = gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode;
  /************* 当模式修改时，做一些数据的过度 **************/
  if( gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode != gimbal_mode_set->gimbal_yaw_motor.last_gimbal_motor_mode )
  {
    if( gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_NO_CONTROL )
    {
      gimbal_mode_set->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_NO_CONTROL;
    }
    else if( gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_LOCK_MODE )
    {
			gimbal_mode_set->gimbal_yaw_motor.relative_angle_set = YAW_RELATIVE_INIT_ANGLE;
//      gimbal_mode_set->gimbal_yaw_motor.relative_angle_set = gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_measure->ecd;
      gimbal_mode_set->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_LOCK_MODE;
    }
    else if( gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_MODE )
    {
      gimbal_mode_set->gimbal_yaw_motor.relative_angle_set = gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_measure->ecd;
      gimbal_mode_set->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
    }
	else if( gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO )	// 开启小陀螺模式2.0
	{
		Angle_Extend_Init( &yaw_gyro_pos, EXTERND_DATA_COUNT_INIT, EXTERND_DATA2_COUNT_MIN, EXTERND_DATA2_COUNT_MAX, -180.0f, 180.0f );
		gimbal_mode_set->gimbal_yaw_motor.motor_gyro_set = temp_gimbal_yaw + yaw_gyro_pos.count*( yaw_gyro_pos.angle_max - yaw_gyro_pos.angle_min );
		gimbal_mode_set->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
	}
    /*else if( gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_MODE )
    {
		gimbal_mode_set->yaw_absolute_angle_pos.count = EXTERND_DATA_COUNT_INIT;
		gimbal_mode_set->yaw_absolute_angle_pos.now = *gimbal_mode_set->chassis_yaw_angle+180;
		gimbal_mode_set->yaw_absolute_angle_pos.last = gimbal_mode_set->yaw_absolute_angle_pos.now;
		gimbal_mode_set->gimbal_yaw_motor.absolute_angle_stable = *gimbal_mode_set->chassis_yaw_angle+180 + gimbal_mode_set->yaw_absolute_angle_pos.count*360;
		gimbal_mode_set->yaw_absolute_angle_pos.all = gimbal_mode_set->gimbal_yaw_motor.absolute_angle_stable + gimbal_mode_set->yaw_absolute_angle_pos.count*360;
//		gimbal_mode_set->yaw_absolute_angle_pos.all_last = gimbal_mode_set->yaw_absolute_angle_pos.all;
		
		gimbal_mode_set->yaw_absolute_encoder_pos.count = EXTERND_DATA_COUNT_INIT;
		gimbal_mode_set->yaw_absolute_encoder_pos.now = gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_measure->ecd;
		gimbal_mode_set->yaw_absolute_encoder_pos.last = gimbal_mode_set->yaw_absolute_encoder_pos.now;
		gimbal_mode_set->gimbal_yaw_motor.absolute_ecd_stable = gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_measure->ecd + gimbal_mode_set->yaw_absolute_encoder_pos.count*GM6020_MAX_ENCODER;
		gimbal_mode_set->yaw_absolute_encoder_pos.all = gimbal_mode_set->gimbal_yaw_motor.absolute_ecd_stable + gimbal_mode_set->yaw_absolute_encoder_pos.count*GM6020_MAX_ENCODER;
//		gimbal_mode_set->yaw_absolute_encoder_pos.all_last = gimbal_mode_set->yaw_absolute_encoder_pos.all;
		gimbal_mode_set->gimbal_yaw_motor.absolute_ecd = gimbal_mode_set->yaw_absolute_encoder_pos.all;
		gimbal_mode_set->gimbal_yaw_motor.absolute_ecd_set = gimbal_mode_set->yaw_absolute_encoder_pos.all;
		
      gimbal_mode_set->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ABSOLUTE_MODE;
    }*/
	else if( gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_AIMING )	// 开启自瞄模式
    {
//		gimbal_mode_set->enemy_xy_refresh_times = 0;
	}
    else
    {
      gimbal_mode_set->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode;
    }
  }
	
	//pitch轴电机数据过渡
  if( gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode != gimbal_mode_set->gimbal_pitch_motor.last_gimbal_motor_mode )
  {
    if( gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_NO_CONTROL )
    {
      gimbal_mode_set->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_NO_CONTROL;
    }
    else if( gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_LOCK_MODE )
    {
      gimbal_mode_set->gimbal_pitch_motor.relative_angle_set = gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_measure->ecd;
      gimbal_mode_set->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_LOCK_MODE;
    }
    else if( gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_MODE )
    {
			gimbal_mode_set->gimbal_pitch_motor.relative_angle_set = gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_measure->ecd;
			gimbal_mode_set->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;
    }
    else if( gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_MODE )
    {
//      gimbal_mode_set->gimbal_pitch_motor.absolute_angle_stable = *gimbal_mode_set->gimbal_mpu6050_pitch;
//      gimbal_mode_set->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_measure->ecd;
//      gimbal_mode_set->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ABSOLUTE_MODE;
    }
    else
    {
      gimbal_mode_set->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode;
    }
  }
}

static void GIMBAL_Data_Updata( Gimbal_Control_t *gimbal_updata )
{
  if( gimbal_updata == NULL )	return;
  
  if( gimbal_updata->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_NO_CONTROL )
  {
  }
  else if( gimbal_updata->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_LOCK_MODE )
  {
    gimbal_updata->gimbal_yaw_motor.relative_angle = gimbal_updata->gimbal_yaw_motor.gimbal_motor_measure->ecd;
  }
  else if( gimbal_updata->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_MODE )
  {
    gimbal_updata->gimbal_yaw_motor.relative_angle = gimbal_updata->gimbal_yaw_motor.gimbal_motor_measure->ecd;
  }
  else if( gimbal_updata->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_AIMING )
  {
    gimbal_updata->gimbal_yaw_motor.enemy_pos = *gimbal_updata->enemy_x;
  }
  else if( gimbal_updata->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO )
  {
	  // 数据处理，将将0~360度数据转化为 0~无穷 的范围
	  Angle_Extend( &yaw_gyro_pos, temp_gimbal_yaw );
	  gimbal_updata->gimbal_yaw_motor.motor_gyro = yaw_gyro_pos.all;
  }
  else if( gimbal_updata->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_MODE )	// 小陀螺模式
  {
	  /*数据处理，将0~360度、0~8191的数据转化为 0~无穷 的范围*/
	  // 先处理陀螺仪值
	  gimbal_updata->yaw_absolute_angle_pos.now = *gimbal_updata->chassis_yaw_angle+180.0f;
	  if( gimbal_updata->yaw_absolute_angle_pos.now - gimbal_updata->yaw_absolute_angle_pos.last > 180.0f )
	  {
		  gimbal_updata->yaw_absolute_angle_pos.count--;
	  }
	  else if( gimbal_updata->yaw_absolute_angle_pos.now - gimbal_updata->yaw_absolute_angle_pos.last < -180.0f )
	  {
		  gimbal_updata->yaw_absolute_angle_pos.count++;
	  }
	  if( gimbal_updata->yaw_absolute_angle_pos.count < EXTERND_DATA_COUNT_MIN )
	  {
		  gimbal_updata->yaw_absolute_angle_pos.count += ( EXTERND_DATA_COUNT_MAX - EXTERND_DATA_COUNT_MIN );
		  gimbal_updata->yaw_absolute_encoder_pos.count -= ( EXTERND_DATA_COUNT_MAX - EXTERND_DATA_COUNT_MIN );
	  }
	  else if( gimbal_updata->yaw_absolute_angle_pos.count > EXTERND_DATA_COUNT_MAX )
	  {
		  gimbal_updata->yaw_absolute_angle_pos.count -= ( EXTERND_DATA_COUNT_MAX - EXTERND_DATA_COUNT_MIN );
		  gimbal_updata->yaw_absolute_encoder_pos.count += ( EXTERND_DATA_COUNT_MAX - EXTERND_DATA_COUNT_MIN );
	  }
	  gimbal_updata->yaw_absolute_angle_pos.last = gimbal_updata->yaw_absolute_angle_pos.now;
	  
	  // 再处理编码器的值
	  gimbal_updata->yaw_absolute_encoder_pos.now = gimbal_updata->gimbal_yaw_motor.gimbal_motor_measure->ecd;
	  if( gimbal_updata->yaw_absolute_encoder_pos.now - gimbal_updata->yaw_absolute_encoder_pos.last > GM6020_HALF_ENCODER )
	  {
		  gimbal_updata->yaw_absolute_encoder_pos.count--;
	  }
	  else if( gimbal_updata->yaw_absolute_encoder_pos.now - gimbal_updata->yaw_absolute_encoder_pos.last < -GM6020_HALF_ENCODER )
	  {
		  gimbal_updata->yaw_absolute_encoder_pos.count++;
	  }
	  gimbal_updata->yaw_absolute_encoder_pos.last = gimbal_updata->yaw_absolute_encoder_pos.now;
  }
  
  if( gimbal_updata->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_NO_CONTROL )
  {
  }
  else if( gimbal_updata->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_LOCK_MODE )
  {
    gimbal_updata->gimbal_pitch_motor.relative_angle = gimbal_updata->gimbal_pitch_motor.gimbal_motor_measure->ecd;
  }
  else if( gimbal_updata->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_MODE )
  {
    gimbal_updata->gimbal_pitch_motor.relative_angle = gimbal_updata->gimbal_pitch_motor.gimbal_motor_measure->ecd;
  }
  else if( gimbal_updata->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_AIMING )
  {
    gimbal_updata->gimbal_pitch_motor.enemy_pos = *gimbal_updata->enemy_y;
  }
  else if( gimbal_updata->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_MODE )
  {
    gimbal_updata->gimbal_pitch_motor.absolute_ecd = gimbal_updata->gimbal_pitch_motor.gimbal_motor_measure->ecd;
  }
  
  // C板自带陀螺仪的弧度制与角度值的换算
  gimbal_updata->chassis_yaw = INS_angle[0]*180/PI;
  gimbal_updata->chassis_pitch = INS_angle[1]*180/PI;
  gimbal_updata->chassis_roll = INS_angle[2]*180/PI;
}

static void GIMBAL_Set_Control( Gimbal_Control_t *gimbal_set_control )
{
  int16_t yaw_remote_channel, pitch_remote_channel;
  int16_t yaw_mouse_channel, pitch_mouse_channel;
  float yaw_set_channel = 0, pitch_set_channel = 0;

  if( gimbal_set_control == NULL )  return;

  // 将遥控和键盘的值赋值进来
  if( switch_is_mid(gimbal_set_control->gimbal_rc_ctrl->rc.s[CTRL_CHANNEL]) )	// 如果想被遥控控制，就要拨到中档
  {
    rc_deadline_limit( gimbal_set_control->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_remote_channel, 10 );
    rc_deadline_limit( gimbal_set_control->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_remote_channel, 10 );

#if YAW_REMOTE_ROTATE_DIR
    yaw_remote_channel = -yaw_remote_channel;
#endif
#if PITCH_REMOTE_ROTATE_DIR
    pitch_remote_channel = -pitch_remote_channel;
#endif

    yaw_set_channel = yaw_remote_channel * RC_CONTROL_YAW_ROTATE_SEPPD;
    pitch_set_channel = pitch_remote_channel * RC_CONTROL_PITCH_ROTATE_SEPPD;
  }
  else if( switch_is_up(gimbal_set_control->gimbal_rc_ctrl->rc.s[CTRL_CHANNEL]) )	// 如果想被键盘控制，就要拨到上档
  {
	  rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->mouse.x, yaw_mouse_channel, 3);
	  rc_deadline_limit(-gimbal_set_control->gimbal_rc_ctrl->mouse.y, pitch_mouse_channel, 3);

	  // 鼠标右键减速设置
	  yaw_mouse_channel /= gimbal_yaw_slow_down_rate;
	  pitch_mouse_channel /= gimbal_pitch_slow_down_rate;
	  
#if YAW_MOUSE_ROTATE_DIR
    yaw_mouse_channel = -yaw_mouse_channel;
#endif
#if PITCH_MOUSE_ROTATE_DIR
    pitch_mouse_channel = -pitch_mouse_channel;
#endif

    yaw_set_channel = yaw_mouse_channel * MOUSE_CONTROL_YAW_ROTATE_SPEED;
    pitch_set_channel = pitch_mouse_channel * MOUSE_CONTROL_PITCH_ROTATE_SPEED;
  }

  if( gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_NO_CONTROL )
  {
    //该模式不移动电机
    yaw_set_channel = 0;
		gimbal_set_control->gimbal_yaw_motor.relative_angle_set += pitch_set_channel;
    // 限制最大旋转的角度
    if(gimbal_set_control->gimbal_yaw_motor.relative_angle_set > gimbal_set_control->gimbal_yaw_motor.max_relative_angle)
      gimbal_set_control->gimbal_yaw_motor.relative_angle_set = gimbal_set_control->gimbal_yaw_motor.max_relative_angle;
    if(gimbal_set_control->gimbal_yaw_motor.relative_angle_set < gimbal_set_control->gimbal_yaw_motor.min_relative_angle)
      gimbal_set_control->gimbal_yaw_motor.relative_angle_set = gimbal_set_control->gimbal_yaw_motor.min_relative_angle;
  }
  else if( gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_LOCK_MODE )
  {
    //不增加电机要移动的角度
    yaw_set_channel = 0;			//yaw轴的电机不移动
	  
		fp32 yaw_error = gimbal_set_control->gimbal_yaw_motor.relative_angle_set - gimbal_set_control->gimbal_yaw_motor.relative_angle;
		if( yaw_error < YAW_MOTOR_RELATIVE_BLOCK_BOUNDARY && yaw_error > -YAW_MOTOR_RELATIVE_BLOCK_BOUNDARY ) 
		{
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.kp = 30;
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.ki = 0.003;
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.kd = 0.5;
			
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kp = 5;
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.Ki = 0;
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kd = 0.1;
			
//			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.kp = 1;
//			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.ki = 0.001;
//			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.kd = 0;
//			
//			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kp = 10;
//			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.Ki = 0;
//			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kd = 0;
		}
		else
		{
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.kp = YAW_MOTOR_RELATIVE_KP;
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.ki = YAW_MOTOR_RELATIVE_KI;
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.kd = YAW_MOTOR_RELATIVE_KD;
			
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kp = YAW_MOTOR_SPEED_KP;
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.Ki = YAW_MOTOR_SPEED_KI;
			gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kd = YAW_MOTOR_SPEED_KD;
		}
  }
  else if( gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_MODE )
  {
    // 改变电机要转的角度
    gimbal_set_control->gimbal_yaw_motor.relative_angle_set += yaw_set_channel;
//    // 限制最大旋转的角度
//    if( gimbal_set_control->gimbal_yaw_motor.relative_angle_set > gimbal_set_control->gimbal_yaw_motor.max_relative_angle )
//      gimbal_set_control->gimbal_yaw_motor.relative_angle_set = gimbal_set_control->gimbal_yaw_motor.max_relative_angle;
//    if( gimbal_set_control->gimbal_yaw_motor.relative_angle_set < gimbal_set_control->gimbal_yaw_motor.min_relative_angle )
//      gimbal_set_control->gimbal_yaw_motor.relative_angle_set = gimbal_set_control->gimbal_yaw_motor.min_relative_angle;
  }
  else if( gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO )
  {
	  if( chassis_return_mode != 1 )
	  {
		  yaw_set_channel /= 6;
		  gimbal_set_control->gimbal_yaw_motor.motor_gyro_set += yaw_set_channel;
	  }
	  
	  if( gimbal_set_control->gimbal_yaw_motor.motor_gyro_set - gimbal_set_control->gimbal_yaw_motor.motor_gyro > 360 )
	  {
		  gimbal_set_control->gimbal_yaw_motor.motor_gyro_set -= (yaw_gyro_pos.count_max-yaw_gyro_pos.count_min)*(yaw_gyro_pos.angle_max-yaw_gyro_pos.angle_min);
	  }
	  else if( gimbal_set_control->gimbal_yaw_motor.motor_gyro_set - gimbal_set_control->gimbal_yaw_motor.motor_gyro < -360 )
	  {
		  gimbal_set_control->gimbal_yaw_motor.motor_gyro_set += (yaw_gyro_pos.count_max-yaw_gyro_pos.count_min)*(yaw_gyro_pos.angle_max-yaw_gyro_pos.angle_min);
	  }
	  
	  // 模糊PID，当yaw轴移动范围过大时，修改PID参数
//	  if( fabs(gimbal_set_control->gimbal_yaw_motor.motor_gyro_set-gimbal_set_control->gimbal_yaw_motor.motor_gyro) < 2 )
//	  {
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_angle_pid.kp = 20;
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_angle_pid.ki = 0;
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_angle_pid.kd = 0.05;
//		  
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_speed_pid.Kp = 70;
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_speed_pid.Ki = 0;
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_speed_pid.Kd = 0;
//	  }
//	  else
//	  {
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_angle_pid.kp = YAW_MOTOR_GYRO_ANGLE_KP;
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_angle_pid.ki = YAW_MOTOR_GYRO_ANGLE_KI;
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_angle_pid.kd = YAW_MOTOR_GYRO_ANGLE_KD;
//		  
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_speed_pid.Kp = YAW_MOTOR_GYRO_SPEED_KP;
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_speed_pid.Ki = YAW_MOTOR_GYRO_SPEED_KI;
//		  gimbal_set_control->gimbal_yaw_motor.gimbal_motor_gyro_mode_speed_pid.Kd = YAW_MOTOR_GYRO_SPEED_KD;
//	  }
  }
  else if( gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_MODE )
  {
	  // 云台的角度根据鼠标的移动而增加，但移动的范围有限制（左右两圈的距离）
	  gimbal_set_control->gimbal_yaw_motor.absolute_ecd_stable += yaw_set_channel;
	  if( gimbal_set_control->gimbal_yaw_motor.absolute_ecd_stable <= TOP_YAW_MIN_ECD )
	  {
		  gimbal_set_control->gimbal_yaw_motor.absolute_ecd_stable = TOP_YAW_MIN_ECD;
	  }
	  else if( gimbal_set_control->gimbal_yaw_motor.absolute_ecd_stable >= TOP_YAW_MAX_ECD )
	  {
		  gimbal_set_control->gimbal_yaw_motor.absolute_ecd_stable = TOP_YAW_MAX_ECD;
	  }
	  
	  gimbal_set_control->gimbal_yaw_motor.absolute_ecd = gimbal_set_control->gimbal_yaw_motor.gimbal_motor_measure->ecd + 
														  gimbal_set_control->yaw_absolute_encoder_pos.count*GM6020_MAX_ENCODER;
	  gimbal_set_control->gimbal_yaw_motor.absolute_ecd_set = gimbal_set_control->yaw_absolute_angle_pos.now + 
															  gimbal_set_control->yaw_absolute_angle_pos.count*360 - 
															  gimbal_set_control->gimbal_yaw_motor.absolute_angle_stable;
	  gimbal_set_control->gimbal_yaw_motor.absolute_ecd_set = gimbal_set_control->gimbal_yaw_motor.absolute_ecd_set*
															  GM6020_MAX_ENCODER/360.0f;
	  gimbal_set_control->gimbal_yaw_motor.absolute_ecd_set = -gimbal_set_control->gimbal_yaw_motor.absolute_ecd_set + 
															  gimbal_set_control->gimbal_yaw_motor.absolute_ecd_stable;
//	  gimbal_set_control->gimbal_yaw_motor.absolute_angle_set = gimbal_set_control->chassis_yaw - gimbal_set_control->gimbal_yaw_motor.absolute_angle_stable;
//	  gimbal_set_control->gimbal_yaw_motor.absolute_angle_set = gimbal_set_control->gimbal_yaw_motor.absolute_angle_set*GM6020_MAX_ENCODER/360.0f;
//	  gimbal_set_control->gimbal_yaw_motor.absolute_angle_set -= gimbal_set_control->gimbal_yaw_motor.absolute_ecd_stable;
    //		gimbal_set_control->gimbal_yaw_motor.absolute_angle_stable += yaw_set_channel*0.1;
    //PID_calc( &gimbal_set_control->gimbal_absolute_mode_pos_pidaO, gimbal_set_control->gimbal_yaw_motor.absolute_angle, gimbal_set_control->gimbal_yaw_motor.absolute_angle_set );
    //gimbal_set_control->gimbal_yaw_motor.absolute_angle_set = gimbal_set_control->gimbal_yaw_motor.absolute_ecd_stable + gimbal_set_control->gimbal_absolute_mode_pos_pid.out;
  }

	//pitch轴电机控制量设置
  if( gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_NO_CONTROL )
  {
    // 该模式不移动电机
    pitch_set_channel = 0;
  }
  else if( gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_LOCK_MODE )
  {
    // pitch电机限制角度:
    pitch_set_channel = 0;
  }
  else if( gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_MODE )
  {
	  // 自瞄2.0测试代码
	  //gimbal_set_control->gimbal_pitch_motor.relative_angle_set += ((*gimbal_set_control->enemy_y-AIM_POS_Y)*temp_auto_aming_pitch);
    // 改变电机要转的角度
    gimbal_set_control->gimbal_pitch_motor.relative_angle_set += pitch_set_channel;
    // 限制最大旋转的角度
    if(gimbal_set_control->gimbal_pitch_motor.relative_angle_set > gimbal_set_control->gimbal_pitch_motor.max_relative_angle)
      gimbal_set_control->gimbal_pitch_motor.relative_angle_set = gimbal_set_control->gimbal_pitch_motor.max_relative_angle;
    if(gimbal_set_control->gimbal_pitch_motor.relative_angle_set < gimbal_set_control->gimbal_pitch_motor.min_relative_angle)
      gimbal_set_control->gimbal_pitch_motor.relative_angle_set = gimbal_set_control->gimbal_pitch_motor.min_relative_angle;
  }
  else if(gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_AIMING)
  {
	  
  }
  else if(gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_MODE)
  {
//    gimbal_set_control->gimbal_pitch_motor.absolute_angle_stable += yaw_set_channel * 0.1;
//    gimbal_set_control->gimbal_pitch_motor.absolute_angle_set = gimbal_set_control->gimbal_pitch_motor.absolute_angle_stable + gimbal_set_control->chassis_pitch * GM6020_MAX_ENCODER / 360;
  }
  
  
//  if( data_i > 999 )	data_i = 999;
//  data_buf1[data_i++] = yaw_set_channel;
}
int16_t speed_rpm_temp;
fp32 current_set_temp;
static void GIMBAL_PID_Cal(Gimbal_Control_t *gimbal_pid_cal)
{
  if (gimbal_pid_cal == NULL)				return;
	
  //yaw不同模式对于不同的控制函数
  if(gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_NO_CONTROL)
  {
    // 不给电机任何控制信号
    gimbal_pid_cal->gimbal_yaw_motor.current_set = 0;
    gimbal_pid_cal->gimbal_yaw_motor.given_current = 0;
  }
  else if(gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_LOCK_MODE)
  {
    // 位置没有增量
    gimbal_pid_cal->gimbal_yaw_motor.current_set = GIMBAL_PID_Calc(&gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_relative_angle_pid,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.relative_angle,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.relative_angle_set,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm);
    
		//gimbal_pid_cal->gimbal_yaw_motor.current_set = 40;
		//current_set_temp = gimbal_pid_cal->gimbal_yaw_motor.current_set;
		gimbal_pid_cal->gimbal_yaw_motor.current_set = PID_calc(&gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_gyro_pid,
                                                             gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm,
                                                             gimbal_pid_cal->gimbal_yaw_motor.current_set);
    gimbal_pid_cal->gimbal_yaw_motor.given_current = gimbal_pid_cal->gimbal_yaw_motor.current_set;
  }
  else if( gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_MODE )
  {
    // 编码器控制
    gimbal_pid_cal->gimbal_yaw_motor.current_set = GIMBAL_PID_Calc( &gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_relative_angle_pid,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.relative_angle,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.relative_angle_set,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm );
    gimbal_pid_cal->gimbal_yaw_motor.current_set = PID_calc( &gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_gyro_pid,
                                                             gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm,
                                                             gimbal_pid_cal->gimbal_yaw_motor.current_set );
    gimbal_pid_cal->gimbal_yaw_motor.given_current = gimbal_pid_cal->gimbal_yaw_motor.current_set;
  }
  else if( gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO )
  {
		gimbal_pid_cal->gimbal_yaw_motor.current_set = GIMBAL_PID_Calc( &gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_gyro_mode_angle_pid,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.motor_gyro,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.motor_gyro_set,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm );
    speed_rpm_temp = gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
		gimbal_pid_cal->gimbal_yaw_motor.current_set = PID_calc( &gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_gyro_mode_speed_pid,
                                                             gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm,
                                                             gimbal_pid_cal->gimbal_yaw_motor.current_set );
    gimbal_pid_cal->gimbal_yaw_motor.given_current = gimbal_pid_cal->gimbal_yaw_motor.current_set;
  }
  else if( gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_MODE )
  {
    gimbal_pid_cal->gimbal_yaw_motor.current_set = GIMBAL_PID_Calc( &gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.absolute_ecd,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.absolute_ecd_set,
                                                                    0 );
    gimbal_pid_cal->gimbal_yaw_motor.current_set = PID_calc( &gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_gyro_pid,
                                                             gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm,
                                                             gimbal_pid_cal->gimbal_yaw_motor.current_set );
    gimbal_pid_cal->gimbal_yaw_motor.given_current = gimbal_pid_cal->gimbal_yaw_motor.current_set;
	  
//	  gimbal_pid_cal->gimbal_yaw_motor.current_set = GIMBAL_PID_Calc( &gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid,
//                                                                    gimbal_pid_cal->gimbal_yaw_motor.absolute_ecd,
//                                                                    gimbal_pid_cal->gimbal_yaw_motor.absolute_ecd_set,
//                                                                    0 );
//    gimbal_pid_cal->gimbal_yaw_motor.current_set = PID_calc( &gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_gyro_pid,
//                                                             *gimbal_pid_cal->gimbal_yaw_gyro,
//                                                             gimbal_pid_cal->gimbal_yaw_motor.current_set );
//    gimbal_pid_cal->gimbal_yaw_motor.given_current = gimbal_pid_cal->gimbal_yaw_motor.current_set;
  }
  else if( gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_AIMING )
  {
    gimbal_pid_cal->gimbal_yaw_motor.current_set = PID_calc( &gimbal_pid_cal->gimbal_yaw_motor.auto_aiming_mode_pos_pid,
                                                                    *gimbal_pid_cal->enemy_x,
                                                                    gimbal_pid_cal->gimbal_yaw_motor.enemy_pos_set );
    gimbal_pid_cal->gimbal_yaw_motor.current_set = PID_calc( &gimbal_pid_cal->gimbal_yaw_motor.auto_aiming_mode_speed_pid,
                                                             gimbal_pid_cal->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm,
                                                             -gimbal_pid_cal->gimbal_yaw_motor.current_set );
    gimbal_pid_cal->gimbal_yaw_motor.given_current = gimbal_pid_cal->gimbal_yaw_motor.current_set;
  }

  //pitch不同模式对于不同的控制函数
  if( gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_NO_CONTROL )
  {
    // 不给电机任何控制信号
    gimbal_pid_cal->gimbal_pitch_motor.current_set = 0;
    gimbal_pid_cal->gimbal_pitch_motor.given_current = 0;
		gimbal_pid_cal->gimbal_pitch_motor.current_set = 0;
    gimbal_pid_cal->gimbal_pitch_motor.given_current = 0;
  }
  else if( gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_LOCK_MODE )
  {
    // 位置没有增量
    gimbal_pid_cal->gimbal_pitch_motor.current_set = GIMBAL_PID_Calc( &gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_relative_angle_pid,
                                                                      gimbal_pid_cal->gimbal_pitch_motor.relative_angle,
                                                                      gimbal_pid_cal->gimbal_pitch_motor.relative_angle_set,
                                                                      *gimbal_pid_cal->gimbal_pitch_gyro );
    gimbal_pid_cal->gimbal_pitch_motor.current_set = PID_calc( &gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_gyro_pid,
                                                               *gimbal_pid_cal->gimbal_pitch_gyro,
                                                               gimbal_pid_cal->gimbal_pitch_motor.current_set );
    gimbal_pid_cal->gimbal_pitch_motor.given_current = gimbal_pid_cal->gimbal_pitch_motor.current_set;
  }
  else if( gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER_MODE )
  {
    // 编码器控制
    gimbal_pid_cal->gimbal_pitch_motor.current_set = GIMBAL_PID_Calc( &gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_relative_angle_pid,
                                                                      gimbal_pid_cal->gimbal_pitch_motor.relative_angle,
                                                                      gimbal_pid_cal->gimbal_pitch_motor.relative_angle_set,
                                                                      gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm );
    gimbal_pid_cal->gimbal_pitch_motor.current_set = PID_calc( &gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_gyro_pid,
                                                               gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm,
                                                               gimbal_pid_cal->gimbal_pitch_motor.current_set );
    gimbal_pid_cal->gimbal_pitch_motor.given_current = gimbal_pid_cal->gimbal_pitch_motor.current_set;
  }
  else if( gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_AIMING )
  {
    gimbal_pid_cal->gimbal_pitch_motor.current_set = PID_calc( &gimbal_pid_cal->gimbal_pitch_motor.auto_aiming_mode_pos_pid,
                                                                    *gimbal_pid_cal->enemy_y,
                                                                    gimbal_pid_cal->gimbal_pitch_motor.enemy_pos_set );
    gimbal_pid_cal->gimbal_pitch_motor.current_set = PID_calc( &gimbal_pid_cal->gimbal_pitch_motor.auto_aiming_mode_speed_pid,
                                                             gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm,
                                                             -gimbal_pid_cal->gimbal_pitch_motor.current_set );
    gimbal_pid_cal->gimbal_pitch_motor.given_current = gimbal_pid_cal->gimbal_pitch_motor.current_set;
  }
  else if( gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_MODE )
  {
    // 没啥意义
    //		gimbal_pid_cal->gimbal_pitch_motor.current_set = GIMBAL_PID_Calc( &gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid,
    //																		gimbal_pid_cal->gimbal_pitch_motor.absolute_angle,
    //																		gimbal_pid_cal->gimbal_pitch_motor.absolute_angle_set,
    //																		*gimbal_pid_cal->gimbal_pitch_gyro );
    //		gimbal_pid_cal->gimbal_pitch_motor.current_set = PID_Calc( &gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_gyro_pid,
    //																*gimbal_pid_cal->gimbal_pitch_gyro,
    //																gimbal_pid_cal->gimbal_pitch_motor.current_set );
    //		gimbal_pid_cal->gimbal_pitch_motor.given_current = gimbal_pid_cal->gimbal_pitch_motor.current_set;
  }
//  usb_printf("%6d,%6.0f,%6.0f,%6.0f.", gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_pid_cal->gimbal_pitch_motor.relative_angle_set, *gimbal_pid_cal->gimbal_pitch_gyro, gimbal_pid_cal->gimbal_pitch_motor.gimbal_motor_relative_angle_pid.out );
  //.....................
}

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
  if (pid == NULL)
  {
    return;
  }
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->err = 0.0f;
  pid->get = 0.0f;

  pid->max_iout = max_iout;
  pid->max_out = maxout;
}

/**
***********************************************************************************************************
*	@Function				: AnyAngle_Spread
*	@Description		:	角度扩展函数，受是上面函数的影响，做一个可以扩展任何范围的
*	@Input					:
*						angle	:	输入想要转变的角度值
*						max		:	这个角度本来的范围最大值
*						min		: 这个角度本来范围的最小值
*	@Return					: none
*	@author					：卢佳威
***********************************************************************************************************
*/
void AnyAngle_Spread( float *angle, float max, float min )
{
  static short int change_times = 0, change_flag = 0;
  float len = max - min;

  if( len <= 0 )					return;					// 如果输入范围有问题，返回0

  if( change_flag != 1 )
    if( (*angle >= (max - len / 4)) && (*angle <= max) )
    {
      if( change_flag == 2 )		change_times--;
      change_flag = 1;
    }
  if( change_flag != 2 )
    if( (*angle >= min) && (*angle <= (min + len / 4)) )
    {
      if( change_flag == 1 )		change_times++;
      change_flag = 2;
    }
  if( (*angle > (min + len / 4)) && (*angle < (max - len / 4)) )				change_flag = 0;

  *angle = *angle + len * change_times;
}

const Gimbal_Motor_t *get_yaw_motor_point(void)
{
  return &gimbal_control.gimbal_yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
  return &gimbal_control.gimbal_pitch_motor;
}

static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
  fp32 err;
  if (pid == NULL)
  {
    return 0.0f;
  }
  pid->get = get;
  pid->set = set;

  err = set - get;
  pid->Dout = pid->kd * error_delta;
  pid->err = err;		//原rad_format(err);
  pid->Pout = pid->kp * pid->err;
  pid->Iout += pid->ki * pid->err;
  abs_limit(&pid->Iout, pid->max_iout);
  pid->out = pid->Pout + pid->Iout + pid->Dout;
  abs_limit(&pid->out, pid->max_out);
  return pid->out;
}
//pid数据清理
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
  if (gimbal_pid_clear == NULL)
  {
    return;
  }
  gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
  gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

static void Gimbal_Tim( Gimbal_Control_t *gimbal_tim )
{
	static uint16_t count_time = 0;
	
	count_time++;
	
	if( count_time%10 == 0 )	// 每过10ms周期设置一次舵机的值，以免CAN2总线太过繁忙
	{
		ExBoard_Servo_Control_Mode( &hcan2, 0x520, gimbal_tim->bullet_bin_servo_angle, gimbal_tim->bullet_bin_servo_angle );
	}
	
	if( count_time >= 999 )		count_time = 0;
}

static void Gimbal_Death( Gimbal_Control_t *gimbal_death )
{
	// 云台初始角度设置
	// 将YAW电机角度初始化
	gimbal_death->gimbal_yaw_motor.absolute_ecd 	 = gimbal_death->gimbal_yaw_motor.gimbal_motor_measure->ecd;
	gimbal_death->gimbal_yaw_motor.absolute_ecd_set = YAW_RELATIVE_INIT_ANGLE;
	gimbal_death->gimbal_yaw_motor.relative_angle 	 = gimbal_death->gimbal_yaw_motor.gimbal_motor_measure->ecd;
	gimbal_death->gimbal_yaw_motor.relative_angle_set = YAW_RELATIVE_INIT_ANGLE;
	// 将PITCH电机角度初始化
	gimbal_death->gimbal_pitch_motor.absolute_ecd 	   = gimbal_death->gimbal_pitch_motor.gimbal_motor_measure->ecd;
	gimbal_death->gimbal_pitch_motor.absolute_ecd_set = PITCH_RELATIVE_INIT_ANGLE;
	gimbal_death->gimbal_pitch_motor.relative_angle   = gimbal_death->gimbal_pitch_motor.gimbal_motor_measure->ecd;
	gimbal_death->gimbal_pitch_motor.relative_angle_set = PITCH_RELATIVE_INIT_ANGLE;
	// PID算法中I清零
	
	
	// 电机参数清零？
	
	
	// 模式设置
	gimbal_death->gimbal_yaw_motor.gimbal_motor_mode = gimbal_death->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_LOCK_MODE;
	gimbal_death->gimbal_pitch_motor.gimbal_motor_mode = gimbal_death->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ENCODER_MODE;

	// 计数器清零
	
}
