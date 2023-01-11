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
#include "shoot_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "main.h"
#include "INS_Task.h"
#include "gimbal_task.h"
#include "usb_task.h"
#include "referee.h"
#include "math.h"
#include "cmsis_armcc.h"

User_Motor_t user_motor;   //射击相关电机的结构体
extern ext_power_heat_data_t power_heat_data_t;
extern ext_game_robot_state_t robot_state;   // 裁判系统数据
static fp32 Extra_3508_SetCurrent_L = 0, Extra_3508_SetCurrent_R = 0; //左右摩擦轮电机电流设置值
static fp32 Motor_Trigger_SetCurrent = 0;   //拨弹电机电流设置值
int temp;   //一个临时变量，设为全局变量方便查看变量值
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};

/**
  * @brief          shoot任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void shoot_task(void const *argument)
{
	vTaskDelay(50);
	Shoot_Init(&user_motor);

  while (1)
  {
	  if ( robot_state.remain_HP )	// 还活着 robot_state.remain_HP
	  {
			//遥控器设置状态
			Shoot_Set_Mode(&user_motor);
			Shoot_Feedback_Update(&user_motor);
			Shoot_Set_Contorl(&user_motor);
			Shoot_PID_Calc(&user_motor);

			Extra_3508_SetCurrent_L = user_motor.motor_extra_3508_l.given_current;
			Extra_3508_SetCurrent_R = user_motor.motor_extra_3508_r.given_current;
			Motor_Trigger_SetCurrent = user_motor.motor_trigger.given_current;

			CAN_CMD_Extra3508(Extra_3508_SetCurrent_L, Extra_3508_SetCurrent_R, 0, 0);   //摩擦轮电机为 hcan2 1，2号
			CAN_cmd_temporary(Motor_Trigger_SetCurrent, 0, 0, 0);   //拨弹盘电机为 hcan1 5号
	  }
	  else	// 不好意思您已经壮烈牺牲
	  {
		  
	  }
		//usb_printf("%6d,%6d.", user_motor.motor_extra_3508_l.extra_3508_measure->speed_rpm, user_motor.motor_extra_3508_r.extra_3508_measure->speed_rpm );
		vTaskDelay(2);	
	}
}
// 额外的3508电机控制代码
void Shoot_Init(User_Motor_t *User_Motor_InitType)
{
	const static fp32 extra_3508_speed_pid[3] = {EXTRA_3508_SPEED_KP, EXTRA_3508_SPEED_KI, EXTRA_3508_SPEED_KD};
	const static fp32 motor_trigger_speed_pid[3] = {MOTOR_2006_SPEED_KP, MOTOR_2006_SPEED_KI, MOTOR_2006_SPEED_KD};
	
	// 模式初始化
	User_Motor_InitType->motor_mode = MOTOR_PRE_MODE;
	
	// 遥控指针初始化
	User_Motor_InitType->user_motor_RC = get_remote_control_point();
	
	// 拨弹盘3508电机参数初始化
	User_Motor_InitType->motor_trigger.encoder_pos_count = 1;
	User_Motor_InitType->motor_trigger.encoder_pos_all = User_Motor_InitType->motor_trigger.motor_3508_measure->ecd;
	User_Motor_InitType->motor_trigger.encoder_pos_all_set = User_Motor_InitType->motor_trigger.motor_3508_measure->ecd;
	User_Motor_InitType->motor_trigger.encoder_pos_ecd = User_Motor_InitType->motor_trigger.motor_3508_measure->ecd;
	User_Motor_InitType->motor_trigger.encoder_pos_ecd_last = User_Motor_InitType->motor_trigger.motor_3508_measure->ecd;
	
	// 拨弹盘3508 PID 初始化
	User_Motor_InitType->motor_trigger.motor_3508_measure = get_trigger_motor_measure_point();
	PID_init(&User_Motor_InitType->motor_trigger.motor_speed_pid, PID_POSITION, motor_trigger_speed_pid, MOTOR_2006_SPEED_PID_MAXOUT, MOTOR_2006_SPEED_PID_MAXIOUT);
	
	// 防堵转参数初始化设置
	User_Motor_InitType->locked_rotor_time = 0;
	User_Motor_InitType->reversal_time = 0;
	
	// 摩擦轮3508电机参数初始化
	User_Motor_InitType->motor_extra_3508_l.given_current = 0;
	User_Motor_InitType->motor_extra_3508_l.speed_set = 0;
	User_Motor_InitType->motor_extra_3508_l.speed = 0;
	User_Motor_InitType->motor_extra_3508_r.given_current = 0;
	User_Motor_InitType->motor_extra_3508_r.speed_set = 0;
	User_Motor_InitType->motor_extra_3508_r.speed = 0;
	
	// 摩擦轮3508 PID 初始化
	User_Motor_InitType->motor_extra_3508_l.extra_3508_measure = get_extra_motor_measure_point(0);
	PID_init(&User_Motor_InitType->motor_extra_3508_l.motor_speed_pid, PID_POSITION, extra_3508_speed_pid, EXTRA_3508_SPEED_PID_MAXOUT, EXTRA_3508_SPEED_PID_MAXIOUT);
	User_Motor_InitType->motor_extra_3508_r.extra_3508_measure = get_extra_motor_measure_point(1);
	PID_init(&User_Motor_InitType->motor_extra_3508_r.motor_speed_pid, PID_POSITION, extra_3508_speed_pid, EXTRA_3508_SPEED_PID_MAXOUT, EXTRA_3508_SPEED_PID_MAXIOUT);
}
void Shoot_Set_Mode(User_Motor_t *User_Motor_ModeTypedef)
{
	// 遥控设置电机模式
	static uint16_t count = 0, fric_count = 0, shoot_count = 0;   //count用于防抖
	if (switch_is_down(User_Motor_ModeTypedef->user_motor_RC->rc.s[CTRL_CHANNEL]))	
	{
		count++;
		if (count == 1)
			User_Motor_ModeTypedef->motor_mode = MOTOR_PRE_MODE;
		if (count == 500)
			User_Motor_ModeTypedef->motor_mode = MOTOR_GET_READY_MODE;
		if (count >= 10000) 
			count = 501;
	}
	else if (switch_is_mid(User_Motor_ModeTypedef->user_motor_RC->rc.s[CTRL_CHANNEL]))
	{
		count = 0;
		User_Motor_ModeTypedef->motor_mode = MOTOR_USER_STOP_MODE;
	}
	else if (switch_is_up(User_Motor_ModeTypedef->user_motor_RC->rc.s[CTRL_CHANNEL]))	// 使用键盘控制模式
	{
		// 键盘设置电机模式
//		if ((User_Motor_ModeTypedef->user_motor_RC->key.v & KEY_PRESSED_OFFSET_CTRL) == KEY_PRESSED_OFFSET_CTRL)	// ctrl都按下
//		{
//			fric_count++;
//			if (fric_count == 20)
//			{
//				if (User_Motor_ModeTypedef->motor_mode != MOTOR_PRE_MODE) 
//				{
//					User_Motor_ModeTypedef->motor_mode = MOTOR_PRE_MODE;
//				}
//				else 
//				{
//					User_Motor_ModeTypedef->motor_mode = MOTOR_USER_STOP_MODE;
//				}
//			}
//		}
//		else
//		{
//			fric_count = 0;
//		}
		
		if( User_Motor_ModeTypedef->motor_mode == MOTOR_USER_STOP_MODE )
			User_Motor_ModeTypedef->motor_mode = MOTOR_PRE_MODE;
		
		// 鼠标检测
		if (User_Motor_ModeTypedef->motor_mode == MOTOR_PRE_MODE && (User_Motor_ModeTypedef->user_motor_RC->mouse.press_l))
		{
			shoot_count++;
			if(shoot_count == 20)
			{
				User_Motor_ModeTypedef->motor_mode = MOTOR_GET_READY_MODE;
			}
		}
		else
		{
			shoot_count = 0;
		}
	}
	
	//if( power_heat_data_t.shooter_id1_42mm_cooling_heat < 100 )
	//	User_Motor_ModeTypedef->motor_mode = MOTOR_PRE_MODE;
	
	if (switch_is_down(User_Motor_ModeTypedef->user_motor_RC->rc.s[MODE_CHANNEL]))	// 所有电机停转，除拨弹轮
	{
		User_Motor_ModeTypedef->motor_mode = MOTOR_USER_STOP_MODE;
	}
}
void Shoot_Feedback_Update(User_Motor_t *User_Motor_DataType) 
{
	static uint8_t count = 0;
	
	//摩擦轮3508电机数据反馈
	User_Motor_DataType->motor_extra_3508_l.speed = User_Motor_DataType->motor_extra_3508_l.extra_3508_measure->speed_rpm;
	User_Motor_DataType->motor_extra_3508_r.speed = User_Motor_DataType->motor_extra_3508_r.extra_3508_measure->speed_rpm;
	
	//拨弹盘电机机械角度读取
	User_Motor_DataType->motor_trigger.encoder_pos_ecd_last = User_Motor_DataType->motor_trigger.encoder_pos_ecd;
	User_Motor_DataType->motor_trigger.encoder_pos_ecd = User_Motor_DataType->motor_trigger.motor_3508_measure->ecd;

	//拨弹盘电机速度读取
	User_Motor_DataType->motor_trigger.speed = User_Motor_DataType->motor_trigger.motor_3508_measure->speed_rpm;

	//计算拨弹电机转了几圈,电机有内外圈,编码器记录的为内圈的编码值，所以要记录转了几圈来计算外圈位置
	//理解方式可以参考官方例程shoot.c，官方例程里注释多一点，多看几遍慢慢就懂了
	if (User_Motor_DataType->motor_trigger.encoder_pos_ecd - User_Motor_DataType->motor_trigger.encoder_pos_ecd_last >= MOTOR_USER_HALF_ENCODER)
	{
		User_Motor_DataType->motor_trigger.encoder_pos_count--;
	}
	else if (User_Motor_DataType->motor_trigger.encoder_pos_ecd - User_Motor_DataType->motor_trigger.encoder_pos_ecd_last <= -MOTOR_USER_HALF_ENCODER)
	{
		User_Motor_DataType->motor_trigger.encoder_pos_count++;
	}
	if (User_Motor_DataType->motor_trigger.encoder_pos_count >= MOTOR_TRIGGER_ENCODER_COUNT_MAX)
	{
		User_Motor_DataType->motor_trigger.encoder_pos_count = 0;
	}
	else if (User_Motor_DataType->motor_trigger.encoder_pos_count < 0)
	{
		User_Motor_DataType->motor_trigger.encoder_pos_count = MOTOR_TRIGGER_ENCODER_COUNT_MAX - 1;
	}		
	
	User_Motor_DataType->motor_trigger.encoder_pos_all = User_Motor_DataType->motor_trigger.encoder_pos_count*(MOTOR_USER_MAX_ENCODER + 1) 
																												+ User_Motor_DataType->motor_trigger.encoder_pos_ecd;
	if(count == 0) 
	{
		count = 1;
		User_Motor_DataType->motor_trigger.encoder_pos_all_set = User_Motor_DataType->motor_trigger.encoder_pos_all;
	}
}
void Shoot_Set_Contorl(User_Motor_t *User_Motor_DataType) 
{
	if (User_Motor_DataType->motor_mode == MOTOR_PRE_MODE)  //PRE MODE模式下让摩擦轮先转起来
	{
		//根据裁判系统设置射速
		if (robot_state.shooter_id1_42mm_speed_limit == 10) //
		{
			//如果最大射速是10m/s的话
			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_10;
			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_10;
			User_Motor_DataType->motor_trigger.speed_set = 0;
		}
		else if (robot_state.shooter_id1_42mm_speed_limit == 16)
		{
			//如果最大射速是16m/s的话
			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_16;
			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_16;
			User_Motor_DataType->motor_trigger.speed_set = 0;
		}
		else
		{
			//如果都不是的话，那就速度设置为0来报错
			User_Motor_DataType->motor_extra_3508_l.speed_set = 0;
			User_Motor_DataType->motor_extra_3508_r.speed_set = 0;
		}
		#if EXTRA_3508_L_TURN_DIR
			User_Motor_DataType->motor_extra_3508_l.speed_set = User_Motor_DataType->motor_extra_3508_l.speed_set;
		#else
			User_Motor_DataType->motor_extra_3508_l.speed_set = -User_Motor_DataType->motor_extra_3508_l.speed_set;
		#endif
		#if EXTRA_3508_R_TURN_DIR
			User_Motor_DataType->motor_extra_3508_r.speed_set = User_Motor_DataType->motor_extra_3508_r.speed_set;
		#else
			User_Motor_DataType->motor_extra_3508_r.speed_set = -User_Motor_DataType->motor_extra_3508_r.speed_set;
		#endif
	}
	else if (User_Motor_DataType->motor_mode == MOTOR_GET_READY_MODE)
	{
		/*MOTOR_GET_READY_MODE的任务本来是把子弹送到微动开关处，
		但2022年的英雄还未实装微动开关，以后加上了可以在此处修改(huaji.jpg)*/
		User_Motor_DataType->motor_trigger.speed_set = 0;
		#if MOTOR_TRIGGER_TURN_DIR
			User_Motor_DataType->motor_trigger.encoder_pos_all_set += MOTOR_TRIGGER_ENCODER_ONE_SHOOT;
			if (User_Motor_DataType->motor_trigger.encoder_pos_all_set >= MOTOR_TRIGGER_ENCODER_A_ROUND)
			{
				User_Motor_DataType->motor_trigger.encoder_pos_all_set -= MOTOR_TRIGGER_ENCODER_A_ROUND;
			}
		#else
			User_Motor_DataType->motor_trigger.encoder_pos_all_set -= MOTOR_TRIGGER_ENCODER_ONE_SHOOT;
			if (User_Motor_DataType->motor_trigger.encoder_pos_all_set < 0)
			{
				User_Motor_DataType->motor_trigger.encoder_pos_all_set += MOTOR_TRIGGER_ENCODER_A_ROUND;
			}
		#endif
		if (User_Motor_DataType->motor_mode == MOTOR_GET_READY_MODE)		
			//User_Motor_DataType->motor_mode = MOTOR_REVERSAL_MODE;
			User_Motor_DataType->motor_mode = MOTOR_SHOOT_MODE;
	}
	else if (User_Motor_DataType->motor_mode == MOTOR_REVERSAL_MODE)
	{
		if (robot_state.shooter_id1_42mm_speed_limit == 10) //
		{
			//如果最大射速是10m/s的话
			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_10;
			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_10;
			User_Motor_DataType->motor_trigger.speed_set = 0;
		}
		else if (robot_state.shooter_id1_42mm_speed_limit == 16)
		{
			//如果最大射速是16m/s的话
			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_16;
			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_16;
			User_Motor_DataType->motor_trigger.speed_set = 0;
		}
		else
		{
			//如果都不是的话，那就速度设置为0来报错
			User_Motor_DataType->motor_extra_3508_l.speed_set = 0;
			User_Motor_DataType->motor_extra_3508_r.speed_set = 0;
		}
		#if EXTRA_3508_L_TURN_DIR
			User_Motor_DataType->motor_extra_3508_l.speed_set = User_Motor_DataType->motor_extra_3508_l.speed_set;
		#else
			User_Motor_DataType->motor_extra_3508_l.speed_set = -User_Motor_DataType->motor_extra_3508_l.speed_set;
		#endif
		#if EXTRA_3508_R_TURN_DIR
			User_Motor_DataType->motor_extra_3508_r.speed_set = User_Motor_DataType->motor_extra_3508_r.speed_set;
		#else
			User_Motor_DataType->motor_extra_3508_r.speed_set = -User_Motor_DataType->motor_extra_3508_r.speed_set;
		#endif
		
		#if MOTOR_TRIGGER_TURN_DIR
			User_Motor_DataType->motor_trigger.speed_set = -MOTOR_TRIGGER_QUICK_ROTATE_SPEED;
		#else
			User_Motor_DataType->motor_trigger.speed_set = MOTOR_TRIGGER_QUICK_ROTATE_SPEED;
		#endif
		
		User_Motor_DataType->reversal_time++;	
		if (User_Motor_DataType->reversal_time > MOTOR_TRIGGER_LOCKED_ROTOR_REVERSAL)
		{
			// 认为反转时间达到，重新开始正转
			User_Motor_DataType->motor_mode = MOTOR_SHOOT_MODE;
			User_Motor_DataType->reversal_time = 0;
			User_Motor_DataType->locked_rotor_time = 0;
			return;
		}
	}
	else if (User_Motor_DataType->motor_mode == MOTOR_SHOOT_MODE)	// 发射子弹时
	{
		//根据裁判系统设置射速
		if (robot_state.shooter_id1_42mm_speed_limit == 10) //
		{
			//如果最大射速是10m/s的话
			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_10;
			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_10;
		}
		else if (robot_state.shooter_id1_42mm_speed_limit == 16)
		{
			//如果最大射速是16m/s的话
			User_Motor_DataType->motor_extra_3508_l.speed_set = EXTRA_3508_ROTATE_SPEED_16;
			User_Motor_DataType->motor_extra_3508_r.speed_set = EXTRA_3508_ROTATE_SPEED_16;
		}
		else
		{
			//如果都不是的话，那就速度设置为0来报错
			User_Motor_DataType->motor_extra_3508_l.speed_set = 0;
			User_Motor_DataType->motor_extra_3508_r.speed_set = 0;
		}
		#if EXTRA_3508_L_TURN_DIR
			User_Motor_DataType->motor_extra_3508_l.speed_set = User_Motor_DataType->motor_extra_3508_l.speed_set;
		#else
			User_Motor_DataType->motor_extra_3508_l.speed_set = -User_Motor_DataType->motor_extra_3508_l.speed_set;
		#endif
		#if EXTRA_3508_R_TURN_DIR
			User_Motor_DataType->motor_extra_3508_r.speed_set = User_Motor_DataType->motor_extra_3508_r.speed_set;
		#else
			User_Motor_DataType->motor_extra_3508_r.speed_set = -User_Motor_DataType->motor_extra_3508_r.speed_set;
		#endif
		
		//temp为拨弹电机设定位置到实际位置的差值，带符号
		temp = User_Motor_DataType->motor_trigger.encoder_pos_all_set - User_Motor_DataType->motor_trigger.encoder_pos_all;
		#if MOTOR_TRIGGER_TURN_DIR
			if (temp <= -MOTOR_TRIGGER_ENCODER_HALF_ROUND)	temp = temp + MOTOR_TRIGGER_ENCODER_A_ROUND;
		#else
			if (temp >= MOTOR_TRIGGER_ENCODER_HALF_ROUND)	temp = temp - MOTOR_TRIGGER_ENCODER_A_ROUND;
		#endif
		
		if (temp < -MOTOR_TRIGGER_ENCODER_ROUND_ERROR) User_Motor_DataType->motor_trigger.speed_set = MOTOR_TRIGGER_QUICK_ROTATE_SPEED;
		else if (temp > MOTOR_TRIGGER_ENCODER_ROUND_ERROR) User_Motor_DataType->motor_trigger.speed_set = -MOTOR_TRIGGER_QUICK_ROTATE_SPEED;
		//if( temp > 0 ) User_Motor_DataType->motor_trigger.speed_set = -MOTOR_TRIGGER_QUICK_ROTATE_SPEED;
		else
		{
			if (switch_is_up(User_Motor_DataType->user_motor_RC->rc.s[CTRL_CHANNEL]))
				User_Motor_DataType->motor_mode = MOTOR_PRE_MODE;
			else 
				User_Motor_DataType->motor_mode = MOTOR_USER_STOP_MODE;
		}
		#if MOTOR_TRIGGER_TURN_DIR
			User_Motor_DataType->motor_trigger.speed_set = -User_Motor_DataType->motor_trigger.speed_set;
		#endif
	}
	else if (User_Motor_DataType->motor_mode == MOTOR_USER_STOP_MODE)
	{
		User_Motor_DataType->motor_extra_3508_l.speed_set = 0;
		User_Motor_DataType->motor_extra_3508_r.speed_set = 0;
		User_Motor_DataType->motor_trigger.speed_set = 0;
	}
	else if (User_Motor_DataType->motor_mode == MOTOR_LOCKED_ROTOR)
	{
		// 堵转，开始反转
		#if MOTOR_TRIGGER_TURN_DIR
			User_Motor_DataType->motor_trigger.speed_set = -MOTOR_TRIGGER_QUICK_ROTATE_SPEED;
		#else
			User_Motor_DataType->motor_trigger.speed_set = MOTOR_TRIGGER_QUICK_ROTATE_SPEED;
		#endif
		
		temp = User_Motor_DataType->motor_trigger.encoder_pos_all_set - User_Motor_DataType->motor_trigger.encoder_pos_all;
		#if MOTOR_TRIGGER_TURN_DIR
			if (temp <= -MOTOR_TRIGGER_ENCODER_HALF_ROUND)	temp = temp + MOTOR_TRIGGER_ENCODER_A_ROUND;
		#else
			if (temp >= MOTOR_TRIGGER_ENCODER_HALF_ROUND)	temp = temp - MOTOR_TRIGGER_ENCODER_A_ROUND;
		#endif
		User_Motor_DataType->reversal_time++;	
		if (User_Motor_DataType->reversal_time > MOTOR_TRIGGER_LOCKED_ROTOR_REVERSAL 
			|| temp > 1.5 * MOTOR_TRIGGER_ENCODER_ONE_SHOOT )
		{
			// 认为反转条件达到，重新开始正转
			User_Motor_DataType->motor_mode = MOTOR_SHOOT_MODE;
			User_Motor_DataType->reversal_time = 0;
			User_Motor_DataType->locked_rotor_time = 0;
			return;
		}
	}
	
	// 堵转检测
	if (User_Motor_DataType->motor_trigger.speed_set != 0)		// 当电机旋转的时候才开始检测堵转
	{
		if (((User_Motor_DataType->motor_trigger.encoder_pos_ecd - User_Motor_DataType->motor_trigger.encoder_pos_ecd_last) < MOTOR_TRIGGER_LOCKED_ROTOR_ERROR) 
			&& ((User_Motor_DataType->motor_trigger.encoder_pos_ecd - User_Motor_DataType->motor_trigger.encoder_pos_ecd_last) > -MOTOR_TRIGGER_LOCKED_ROTOR_ERROR))
		{
			User_Motor_DataType->locked_rotor_time++;	// 认为是堵转了，开始计数
		}
		else
		{
			User_Motor_DataType->locked_rotor_time--;	// 防抖
		}

		//限制locked_rotor_time的取值范围，防止溢出
		if (User_Motor_DataType->locked_rotor_time > MOTOR_TRIGGER_LOCKED_ROTOR_MAX)
			User_Motor_DataType->locked_rotor_time = MOTOR_TRIGGER_LOCKED_ROTOR_MAX;
		else if (User_Motor_DataType->locked_rotor_time < MOTOR_TRIGGER_LOCKED_ROTOR_MIN)
			User_Motor_DataType->locked_rotor_time = MOTOR_TRIGGER_LOCKED_ROTOR_MIN;
		
		if (User_Motor_DataType->locked_rotor_time >= MOTOR_TRIGGER_LOCKED_ROTOR_SHORT)
		{
			// 确认为堵转了
			User_Motor_DataType->motor_mode = MOTOR_LOCKED_ROTOR;
		}
	}
}
void Shoot_PID_Calc(User_Motor_t *User_Motor_PIDType)
{
	//摩擦轮3508电机PID计算
	PID_calc(&User_Motor_PIDType->motor_extra_3508_l.motor_speed_pid, User_Motor_PIDType->motor_extra_3508_l.speed, User_Motor_PIDType->motor_extra_3508_l.speed_set);
	User_Motor_PIDType->motor_extra_3508_l.given_current = (int16_t)User_Motor_PIDType->motor_extra_3508_l.motor_speed_pid.out;
	PID_calc(&User_Motor_PIDType->motor_extra_3508_r.motor_speed_pid, User_Motor_PIDType->motor_extra_3508_r.speed, -User_Motor_PIDType->motor_extra_3508_r.speed_set);
	User_Motor_PIDType->motor_extra_3508_r.given_current = (int16_t)User_Motor_PIDType->motor_extra_3508_r.motor_speed_pid.out;

	//拨弹盘电机PID计算
	PID_calc(&User_Motor_PIDType->motor_trigger.motor_speed_pid, User_Motor_PIDType->motor_trigger.speed, User_Motor_PIDType->motor_trigger.speed_set);
	User_Motor_PIDType->motor_trigger.given_current = (int16_t)User_Motor_PIDType->motor_trigger.motor_speed_pid.out;
}
void ExBoard_Angle_Init_Mode(CAN_HandleTypeDef *hcan, uint16_t addr)
{
	uint8_t buf[8] = {0};
	buf[7] = MPU6050_ANGLE_INIT_MODE;
	CAN_SEND_MESSAGE(hcan, buf, addr);
}
//这个是弹仓门的舵机
void ExBoard_Servo_Control_Mode(CAN_HandleTypeDef *hcan, uint16_t addr, uint16_t angle1, uint16_t angle2)
{
	uint8_t buf[8] = {0};
	buf[7] = SERVO_ANGLE_SET_MODE;
	buf[0] = angle1 >> 8;
	buf[1] = angle1;
	buf[2] = angle2 >> 8;
	buf[3] = angle2;
	CAN_SEND_MESSAGE(hcan, buf, addr);
}

// 强制复位函数
void CAR_FORCE_RESET(void)
{
	__set_FAULTMASK(1);//关闭所有中断
	NVIC_SystemReset();//复位函数
}
