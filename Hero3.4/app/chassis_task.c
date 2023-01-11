#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "remote_control.h"
#include "can_receive.h"
#include "shoot_task.h"
#include "referee.h"
#include "usb_task.h"

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

		
extern gimbal_motor_mode_e gimbal_yaw_mode;
		
chassis_move_t chassis_move;
extern Gimbal_Control_t gimbal_control;
extern fp32 INS_angle[3];
fp32 temp_gimbal_yaw = 0;
		
//底盘初始化，主要是pid初始化
static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_loop_init( chassis_move_t *chassis_loop_init );
//超级电容初始化输入功率
static void chassis_set_super_cap_value( uint16_t set_power );
//底盘状态机选择，通过遥控器的开关
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//底盘状态改变后处理控制量的改变static
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//底盘设置根据遥控器控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//底盘PID计算以及运动分解
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop, FunctionalState power_limit_sta, uint8_t mode );
//底盘时钟
static void chassis_tim( chassis_move_t *chassis_tim );
//阵亡后数据清零
static void chassis_death( chassis_move_t *chassis_death );
//陀螺仪数据
extern float gimbal_gyro[3], chassis_gyro[3], gimbal_angle[3], chassis_angle[3];
		
// 临时变量
volatile extern int16_t Yaw_Can_Set_Current, Pitch_Can_Set_Current;
// 电压检测
extern fp32 battery_voltage;
// 循环初始化计数变量
uint16_t loop_init_count = 0;
uint16_t time_count = 0;
// 鼠标右键按下以后，所有东西都要减速运行
fp32 chassis_slow_down_rate = CHASSIS_SLOW_DOWN_END;
// 给云台提示，底盘回归时云台为相关模式
volatile uint8_t chassis_return_mode = 0;
// 裁判系统数据
extern ext_power_heat_data_t power_heat_data_t;
extern ext_game_robot_state_t robot_state;
yaw_angle_exterd_t chassis_yaw;
// 复位计时
uint16_t reset_count = 0;
// 暂时用的yaw轴陀螺仪数据
fp32 temp_chassis_yaw = 0;
// 遥控与电脑的控制比例
fp32 x_y_move_len = 10;//, wz_move_len = 0;

void chassis_task(void const *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_move);
	
	while(1)
	{
		if (  robot_state.remain_HP )	// 还活着 robot_state.remain_HP
		{
			chassis_set_mode(&chassis_move);
			chassis_feedback_update(&chassis_move);
			chassis_set_contorl(&chassis_move);
			chassis_tim(&chassis_move);
			if(chassis_move.chassis_power_meter->voltage>12)
			{
			if( chassis_move.super_cap_sta == SUPER_CAP_OPEN )	// 超级电容开启
			{
				chassis_control_loop(&chassis_move, DISABLE, 1);	// 关闭功率限制
			}
			else if( chassis_move.super_cap_sta == SUPER_CAP_CHARGE )	// 通过实时功率来限制功率到某个值
			{
				chassis_control_loop(&chassis_move, ENABLE, 0);
			}
			else	// 开启裁判系统功率限制
			{
				chassis_control_loop(&chassis_move, ENABLE, 1);	
			}
			
			if( chassis_move.chassis_mode == CHASSIS_VECTOR_NO_CONTROL )
			{
				loop_init_count++;
				if( loop_init_count >= 200 )	// 设置超级电容与功率控制参数
				{
					loop_init_count = 0;
					chassis_loop_init( &chassis_move );
				}
				CAN_cmd_chassis( 0, 0, 0, 0 );
			}
			else
			{
				CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
												chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
			}
//			chassis_control_loop(&chassis_move, ENABLE, 0); //麦轮速度分解
//			CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
//												chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
			}
			else
			{
				// PID算法中I清零
				chassis_move.chassis_mouse_control_pid.Iout = 0;
				chassis_move.chassis_angle_pid.Iout = 0;
				int i;
				for( i = 0; i < 4; i++ )
				{
					chassis_move.motor_speed_pid[i].Iout = 0;
				}
				
				// 模式设置
				chassis_move.chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
				chassis_move.super_cap_sta=SUPER_CAP_CLOSE;
				CAN_cmd_chassis(chassis_move.motor_speed_pid[0].Iout, chassis_move.motor_speed_pid[1].Iout,
															chassis_move.motor_speed_pid[2].Iout, chassis_move.motor_speed_pid[3].Iout);
				// 计数器清零
				loop_init_count = 0;
				time_count = 0;
			}
		}
		else	// 不好意思您已经壮烈牺牲
		{
			chassis_death(&chassis_move);
		}
		vTaskDelay(2);
	}
}

static void chassis_loop_init( chassis_move_t *chassis_loop_init )
{
	// 设定超级电容无连接模式
	chassis_loop_init->super_cap_sta = SUPER_CAP_CHARGE;
	
	// 陀螺仪角度初始化
  ExBoard_Angle_Init_Mode( &hcan1, 0x520 );
	ExBoard_Angle_Init_Mode( &hcan2, 0x520 );
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
  if (chassis_move_init == NULL)
  {
    return;
  }

  //底盘速度环pid值
  const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
  //底盘旋转环pid值
  const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
  const static fp32 chassis_mouse_pid[3] = {CHASSIS_MOUSE_PID_KP, CHASSIS_MOUSE_PID_KI, CHASSIS_MOUSE_PID_KD};


  // 陀螺仪角度初始化
  //ExBoard_Angle_Init_Mode( &hcan1, 0x520 );
  Angle_Extend_Init( &chassis_yaw, CHASSIS_YAW_EXTEND_COUNT_INIT, CHASSIS_YAW_EXTEND_COUNT_MIN, CHASSIS_YAW_EXTEND_COUNT_MAX, -180, 180 );
  chassis_move_init->wz_angle_set = CHASSIS_YAW_EXTEND_COUNT_INIT*360;
//  Angle_Extend_Init( &gimbal_yaw, 3, 1, 6, -180, 180 );
	
  //底盘开机状态为停止
  chassis_move_init->chassis_mode = CHASSIS_VECTOR_NO_CONTROL;
  //获取遥控器指针
  chassis_move_init->chassis_RC = get_remote_control_point();
  //获取陀螺仪姿态角指针
  chassis_move_init->chassis_INS_angle = get_INS_angle_point();
  //获取云台电机数据指针
  chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
  chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
  // 获取功率计指针
  chassis_move_init->chassis_power_meter = get_CAN_Power_Meter_Point();
  // 获取超级电容指针
  chassis_move_init->chassis_super_cap = get_CAN_Super_Cap_Point();
  chassis_move_init->super_cap_voltage_min = SUPER_CUP_INPUT_VOLTAGE_MIN;
//  chassis_move_init->super_cap_voltage_warning_ref = chassis_move_init->chassis_power_meter->voltage - SUPER_CAP_INPUT_VOLTAGE_ERROR;
//  chassis_move_init->super_cap_voltage_warning = chassis_move_init->super_cap_voltage_warning_ref;
  //初始化PID 运动
  for (uint8_t i = 0; i < 4; i++)
  {
    chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
  }
  // 初始化旋转PID
  PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
  // 底盘鼠标控制旋转PID
  PID_init(&chassis_move_init->chassis_mouse_control_pid, PID_POSITION, chassis_mouse_pid, CHASSIS_MOUSE_PID_MAX_OUT, CHASSIS_MOUSE_PID_MAX_IOUT);
  // 获取底盘和云台陀螺仪数据

  // 云台使用扩展板的陀螺仪MPU6050
  chassis_move_init->gimbal_mpu6050_pitch = &gimbal_angle[0];
  chassis_move_init->gimbal_mpu6050_roll = &gimbal_angle[1];
  chassis_move_init->gimbal_mpu6050_yaw = gimbal_angle[2];
  chassis_move_init->gimbal_mpu6050_pitch_gyro = &gimbal_gyro[0];
  chassis_move_init->gimbal_mpu6050_roll_gyro = &gimbal_gyro[1];
  chassis_move_init->gimbal_mpu6050_yaw_gyro = &gimbal_gyro[2];
  
  // 底盘使用C板自带陀螺仪，由于底盘自带陀螺仪是弧度制，后面有算式将其转化为角度制
  
  chassis_move_init->chassis_mpu6050_pitch = &chassis_angle[0];
  chassis_move_init->chassis_mpu6050_roll = &chassis_angle[1];
  //chassis_move_init->chassis_mpu6050_yaw = &chassis_angle[2];
  chassis_move_init->chassis_mpu6050_yaw = temp_chassis_yaw;
  chassis_move_init->chassis_mpu6050_pitch_gyro = &chassis_gyro[0];
  chassis_move_init->chassis_mpu6050_roll_gyro = &chassis_gyro[1];
  chassis_move_init->chassis_mpu6050_yaw_gyro = &chassis_gyro[2];
  
  // 设定初始模式
	chassis_move_init->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
  chassis_move_init->super_cap_sta = SUPER_CAP_CHARGE;

  //更新一下数据
  chassis_feedback_update(chassis_move_init);
}

static void chassis_set_super_cap_value( uint16_t set_power )
{
  uint8_t sendbuf[8];
  sendbuf[0] = set_power >> 8;
  sendbuf[1] = set_power;
  CAN_SEND_MESSAGE( &hcan1, sendbuf, 0x210 );
}

fp32 angle_error_whole = 0;
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	static uint16_t top_mode_count = 0;
  if (chassis_move_mode == NULL)
  {
    return;
  }
  else if( switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CtrlChannel]) )	// 使用遥控控制模式
  {
    if( switch_is_mid(chassis_move_mode->chassis_RC->rc.s[ModeChannel]) )	// 普通模式
    {
			if( chassis_move_mode->chassis_mode == CHASSIS_VECTOR_TOP_START_MODE )
			{
				chassis_move_mode->chassis_mode = CHASSIS_VECTOR_TOP_CLOSE_MODE;
				chassis_return_mode = 1;
			}
			else if( chassis_return_mode == 0 )
			{
				chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
				chassis_move_mode->super_cap_sta = SUPER_CAP_OPEN;
			}
			else
			{
				//return_mode != 0 的时候下面有模式变换的逻辑，这里不需要改变
			}
    }
    else if( switch_is_down(chassis_move_mode->chassis_RC->rc.s[ModeChannel]) )	// 所有电机不使能
    {
      chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_CONTROL;
			chassis_move_mode->super_cap_sta = SUPER_CAP_CHARGE;
    }
		else if( switch_is_up(chassis_move_mode->chassis_RC->rc.s[ModeChannel]) )
    {
      chassis_move_mode->chassis_mode = CHASSIS_VECTOR_TOP_START_MODE;
			chassis_move_mode->super_cap_sta = SUPER_CAP_CHARGE;
    }
		
		if(chassis_return_mode == 1)
		{
			fp32 angle_error = chassis_move_mode->gimbal_mpu6050_yaw - chassis_move_mode->chassis_mpu6050_yaw;
			angle_error_whole = angle_error;
			if(angle_error < 0) angle_error = -angle_error;
			if(angle_error < CHASSIS_VECTOR_TOP_CLOSE_ERROR)  //认为回正到位
			{
				chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;	
				chassis_return_mode = 2;
			}
		}
  }
  else if( switch_is_up(chassis_move_mode->chassis_RC->rc.s[CtrlChannel]) )	// 使用键盘控制模式
  {
		if( switch_is_mid(chassis_move_mode->chassis_RC->rc.s[ModeChannel]) )	// 普通模式
    {
			chassis_move_mode->super_cap_sta = SUPER_CAP_OPEN;
    }
		
	  if( chassis_move_mode->chassis_RC->key.v & CHASSIS_TOP_MODE_START )	// 小陀螺模式开启与关闭
	  {
			top_mode_count++;
			if (top_mode_count == 20)
			{
				if (chassis_move_mode->chassis_mode != CHASSIS_VECTOR_TOP_START_MODE) 
				{
					chassis_move_mode->chassis_mode = CHASSIS_VECTOR_TOP_START_MODE;
				}
				else 
				{
					chassis_move_mode->chassis_mode = CHASSIS_VECTOR_TOP_CLOSE_MODE;
				}
			}
	  }
		else 
		{
			top_mode_count = 0;
		}			
		
		if (gimbal_yaw_mode == GIMBAL_MOTOR_ENCODER_MODE)
			chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
		else if( gimbal_yaw_mode == GIMBAL_MOTOR_ENCODER_LOCK_MODE )
			chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; 
		
//	  else if( chassis_move_mode->chassis_RC->key.v & CHASSIS_TOP_MODE_CLOSE )	// 关闭小陀螺模式
//	  {
//			chassis_move_mode->chassis_mode = CHASSIS_VECTOR_TOP_CLOSE_MODE;
//		  chassis_return_mode = 1;
//	  }
	
		if(chassis_return_mode == 1)
		{
			fp32 angle_error = chassis_move_mode->gimbal_mpu6050_yaw - chassis_move_mode->chassis_mpu6050_yaw;
			angle_error_whole = angle_error;
			if(angle_error < 0) angle_error = -angle_error;
			if(angle_error < CHASSIS_VECTOR_TOP_CLOSE_ERROR)  //认为回正到位
			{
				chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;	
				chassis_return_mode = 2;
			}
		}
	  
	  if( reset_count >= 5000 )
	  {
		  if( chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL )	// ctrl按下，强行复位按钮
		  {
			  if( chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT )	// shift按下，强行复位按钮
			  {
				  if( chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_A )	// A按下，强行复位按钮
				  {
					  if( chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_D )	// D按下，强行复位按钮
					  {
						  CAR_FORCE_RESET();
					  }
				  }
			  }
		  }
	  }
	
	  // 鼠标检测
		if( chassis_move_mode->chassis_RC->mouse.press_r )	// 减速
		{
			chassis_slow_down_rate = CHASSIS_SLOW_DOWN_START;
		}
		else
		{
			chassis_slow_down_rate = CHASSIS_SLOW_DOWN_END;
		}
  }
  else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CtrlChannel]))
  {
		if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[ModeChannel]))	// 普通模式
    {
      chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[ModeChannel]))	// 所有电机不使能
    {
      chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_CONTROL;
    }
  }
	
  // last_chassis_mode设置
  if( chassis_move_mode->chassis_mode != chassis_move_mode->last_chassis_mode )
  {
    if( chassis_move_mode->chassis_mode == CHASSIS_VECTOR_NO_CONTROL )
    {
      chassis_move_mode->last_chassis_mode = CHASSIS_VECTOR_NO_CONTROL;
    }
    else if( chassis_move_mode->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW )
    {
			chassis_move_mode->top_mode_dir = chassis_move_mode->gimbal_mpu6050_yaw;
      chassis_move_mode->last_chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else if( chassis_move_mode->chassis_mode == CHASSIS_VECTOR_TOP_START_MODE )
    {

      chassis_move_mode->top_mode_dir = chassis_move_mode->gimbal_mpu6050_yaw;
      /**************************************/
      chassis_move_mode->last_chassis_mode = CHASSIS_VECTOR_TOP_START_MODE;
    }
		else if( chassis_move_mode->last_chassis_mode == CHASSIS_VECTOR_TOP_START_MODE )
    {
			// 当从小陀螺模式变回来时
      //chassis_move_mode->last_chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else
    {
      chassis_move_mode->last_chassis_mode = chassis_move_mode->chassis_mode;
    }
  }
}
//该函数未使用
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
  if (chassis_move_transit == NULL)
  {
    return;
  }

  if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
  {
    return;
  }

  chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
  if (chassis_move_update == NULL)
  {
    return;
  }

  uint8_t i = 0;
  for (i = 0; i < 4; i++)
  {
    //更新电机速度，加速度是速度的PID微分
    chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
  }

  //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
  chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
  chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
  chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

	// C板自带陀螺仪的弧度制与角度值换算
	chassis_move_update->chassis_mpu6050_yaw = INS_angle[0]*180/PI;
}
//遥控器的数据处理成底盘的前进vx速度，vy速度
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
 
}
// 速度缓冲函数，该函数同样未使用
static void chassis_speed_buffer( fp32 *vx, fp32 *vy, fp32 *vz )
{
  static fp32 last_vx = 0, last_vy = 0, last_vz = 0;

  if( *vx * last_vx < 0 )	last_vx = *vx / 2;
  if( *vy * last_vy < 0 )	last_vy = *vy / 2;
  if( *vz * last_vz < 0 )	last_vz = *vz / 2;

  // 一阶滤波
  *vx = CHASSIS_SPEED_BUFFER_PRA1 **vx + CHASSIS_SPEED_BUFFER_PRA2 * last_vx;
  *vy = CHASSIS_SPEED_BUFFER_PRA1 **vy + CHASSIS_SPEED_BUFFER_PRA2 * last_vy;
  *vz = CHASSIS_SPEED_BUFFER_PRA1 **vz + CHASSIS_SPEED_BUFFER_PRA2 * last_vz;

  last_vx = *vx;
  last_vy = *vy;
  last_vz = *vz;
}
static int16_t my_abs16( int16_t num )
{
  if( num < 0 )	return( -num );
  return( num );
}

static fp32 my_abs_float( fp32 num )
{
  if( num < 0 )	return( -num );
  return( num );
}
/**
  * @brief  速度缓冲函数
  * @param  1、最小缓冲值2、目标速度3、当下速度4、缓冲增量5、缓冲模式
  * @retval 无
 **/
fp32 Ramp_function(fp32 delta_Min, fp32 target, fp32 now, fp32 ramp_Coeff, uint8_t mode)
{
  fp32 Lenght = target - now;
  if(fabs(Lenght) < delta_Min)
    now = target;
  else
  {
    if(mode == 0)
    {
      if(fabs(Lenght) > REMOTE_MAX_VALUE)
      {
        return target / 3;
      }
    }
    if(Lenght > 0)
      now += ramp_Coeff;
    else if(Lenght < 0)
      now -= ramp_Coeff;
  }
  return now;
}
//设置遥控器输入控制量
fp32 angle_error_temp = 0, angle_temp = 0;
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
  int16_t wz_temp = 0, vx_temp = 0, vy_temp = 0;
	
  if (chassis_move_control == NULL) return;
  if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
  {
//		if( switch_is_mid(chassis_move_control->chassis_RC->rc.s[CtrlChannel]) )	// 如果想被遥控控制，就要拨到中档
//		{
//      //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
//      rc_deadline_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_temp, CHASSIS_RC_DEADLINE);
//      rc_deadline_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_temp, CHASSIS_RC_DEADLINE);
//      rc_deadline_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_temp, CHASSIS_RC_DEADLINE);

//      vx_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vx_temp, (fp32)chassis_move_control->vx_set/CHASSIS_VX_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
//      vy_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vy_temp, (fp32)chassis_move_control->vy_set/CHASSIS_VY_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
//      wz_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, wz_temp, (fp32)chassis_move_control->wz_set/CHASSIS_WZ_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
//		
//			chassis_move_control->vx_set = vx_temp * CHASSIS_VX_RC_SEN;
//			chassis_move_control->vy_set = vy_temp * CHASSIS_VY_RC_SEN;
//			chassis_move_control->wz_set = wz_temp * CHASSIS_WZ_RC_SEN;
//    }
//    else if( switch_is_up(chassis_move_control->chassis_RC->rc.s[CtrlChannel]) )	// 如果想被键盘控制，就要拨到上档
//    {	
//      int16_t temp;
//			rc_deadline_limit(-chassis_move_control->chassis_RC->mouse.x, wz_temp, CHASSIS_MOUSE_DEADLINE);
//      if( chassis_move_control->chassis_RC->key.v & CHASSIS_FRONT_KEY )	vx_temp -= REMOTE_MAX_VALUE;
//      if( chassis_move_control->chassis_RC->key.v & CHASSIS_BACK_KEY )	vx_temp += REMOTE_MAX_VALUE;
//      if( chassis_move_control->chassis_RC->key.v & CHASSIS_LEFT_KEY )	vy_temp -= REMOTE_MAX_VALUE;
//      if( chassis_move_control->chassis_RC->key.v & CHASSIS_RIGHT_KEY )	vy_temp += REMOTE_MAX_VALUE;

//      temp = ( my_abs16( vx_temp ) + my_abs16( vy_temp ) ) / REMOTE_MAX_VALUE;
//      if( temp != 0 )
//      {
//        vx_temp /= temp;
//        vy_temp /= temp;
//      }

//      vx_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vx_temp, chassis_move_control->vx_set / CHASSIS_VX_KEYBOARD_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
//      vy_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vy_temp, chassis_move_control->vy_set / CHASSIS_VY_KEYBOARD_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
//      //wz_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, wz_temp, chassis_move_control->wz_set/CHASSIS_WZ_MOUSE_SEN, CHASSIS_BUFFER_INCREMENT, 0 );

//			/*旋转为鼠标控制，前后左右平移为键盘控制*/
//      chassis_move_control->vx_set = vx_temp * CHASSIS_VX_KEYBOARD_SEN;
//      chassis_move_control->vy_set = vy_temp * CHASSIS_VY_KEYBOARD_SEN;
//			chassis_move_control->wz_set = wz_temp * CHASSIS_WZ_ANGLE_MOUSE_SEN;
//			
//      //chassis_speed_buffer( &chassis_move_control->vx_set, &chassis_move_control->vy_set, &chassis_move_control->wz_set );
//    }
		fp32 VX_temp = 0, VY_temp = 0, WZ_temp = 0;
    fp32 angle;
		
		int16_t temp = 0;
	  fp32 angle_error = chassis_move_control->gimbal_mpu6050_yaw - chassis_move_control->chassis_mpu6050_yaw;
		if( angle_error > 180 ) angle_error -= 360;
		else if( angle_error < -180 ) angle_error += 360;
		angle_error_temp = angle_error;
		
		if( (-180<angle_error) && (angle_error<= 0) )	// 底盘向右转
	  {
		  chassis_move_control->wz_set = angle_error*90;
		  if( chassis_move_control->wz_set < -5000 )	chassis_move_control->wz_set = -5000;
		  else if( chassis_move_control->wz_set > -1000 )	chassis_move_control->wz_set = -1000;
	  }
	  else if( (0 < angle_error) && (angle_error < 180) )	// 底盘向右转
	  {
		  chassis_move_control->wz_set = angle_error*90;
		  if( chassis_move_control->wz_set >5000 )	chassis_move_control->wz_set = 5000;
		  else if( chassis_move_control->wz_set < 1000 )	chassis_move_control->wz_set = 1000;
	  }
		
		if( -3 < angle_error && angle_error < 3 )
			chassis_move_control->wz_set = 0;
		
    if( switch_is_mid(chassis_move_control->chassis_RC->rc.s[CtrlChannel]) )	// 如果想被遥控控制，就要拨到中档
    {
      rc_deadline_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], VX_temp, CHASSIS_RC_DEADLINE);
      rc_deadline_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], VY_temp, CHASSIS_RC_DEADLINE);

			VX_temp /= 3;
			VY_temp /= 3;
		
      angle = (-angle_error) * PI / 180;
      vx_temp = VX_temp * cos(angle) - VY_temp * sin(angle);
      vy_temp = VX_temp * sin(angle) + VY_temp * cos(angle);
		
			vx_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vx_temp, (fp32)chassis_move_control->vx_set/CHASSIS_VX_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      vy_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vy_temp, (fp32)chassis_move_control->vy_set/CHASSIS_VY_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      //wz_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, wz_temp, (fp32)chassis_move_control->wz_set/CHASSIS_WZ_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
		
      chassis_move_control->vx_set = vx_temp * CHASSIS_VX_RC_SEN;
      chassis_move_control->vy_set = vy_temp * CHASSIS_VY_RC_SEN;
      //chassis_move_control->wz_set = wz_temp * CHASSIS_WZ_RC_SEN;
    }
    else if( switch_is_up(chassis_move_control->chassis_RC->rc.s[CtrlChannel]) )	// 如果想被键盘控制，就要拨到上档
    {
      int16_t temp = 0, speed_par = REMOTE_MAX_VALUE/3;

      if( chassis_move_control->chassis_RC->key.v & CHASSIS_FRONT_KEY )	VX_temp -= speed_par;
      if( chassis_move_control->chassis_RC->key.v & CHASSIS_BACK_KEY )	VX_temp += speed_par;
      if( chassis_move_control->chassis_RC->key.v & CHASSIS_LEFT_KEY )	VY_temp -= speed_par;
      if( chassis_move_control->chassis_RC->key.v & CHASSIS_RIGHT_KEY )	VY_temp += speed_par;

			temp = ( fabs( VX_temp ) + fabs( VY_temp ) ) / speed_par;
      if( temp != 0 )
      {
        VX_temp /= temp;
        VY_temp /= temp;
      }
      angle = (chassis_move_control->chassis_mpu6050_yaw - chassis_move_control->gimbal_mpu6050_yaw) * PI / 180;
      vx_temp = VX_temp * cos(angle) + VY_temp * sin(angle);
      vy_temp = -VX_temp * sin(angle) + VY_temp * cos(angle);

			vx_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vx_temp, (fp32)chassis_move_control->vx_set/CHASSIS_VX_KEYBOARD_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      vy_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vy_temp, (fp32)chassis_move_control->vy_set/CHASSIS_VY_KEYBOARD_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      //wz_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, wz_temp, (fp32)chassis_move_control->wz_set/CHASSIS_WZ_ANGLE_MOUSE_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
	  
			chassis_move_control->vx_set = vx_temp * CHASSIS_VX_KEYBOARD_SEN;
      chassis_move_control->vy_set = vy_temp * CHASSIS_VY_KEYBOARD_SEN;
      //chassis_move_control->wz_set = wz_temp * CHASSIS_WZ_ANGLE_MOUSE_SEN;
    }
  }
  else if( chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW )
  {
    if( switch_is_mid(chassis_move_control->chassis_RC->rc.s[CtrlChannel]) )	// 如果想被遥控控制，就要拨到中档
    {
      rc_deadline_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_temp, CHASSIS_RC_DEADLINE);
      rc_deadline_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_temp, CHASSIS_RC_DEADLINE);

      vx_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vx_temp, chassis_move_control->vx_set / CHASSIS_VX_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      vy_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vy_temp, chassis_move_control->vx_set / CHASSIS_VX_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      wz_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, wz_temp, chassis_move_control->vx_set / CHASSIS_VX_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );

      chassis_move_control->vx_set = vx_temp * CHASSIS_VX_RC_SEN;
      chassis_move_control->vy_set = vy_temp * CHASSIS_VY_RC_SEN;
      chassis_move_control->wz_set = 0;
    }
    else if( switch_is_up(chassis_move_control->chassis_RC->rc.s[CtrlChannel]) )	// 如果想被键盘控制，就要拨到上档
    {
      int16_t temp;

      //rc_deadline_limit(chassis_move_control->chassis_RC->mouse.x, wz_temp, CHASSIS_MOUSE_DEADLINE);
      wz_temp = 0;
      if( chassis_move_control->chassis_RC->key.v & CHASSIS_FRONT_KEY )	vx_temp -= REMOTE_MAX_VALUE;
      if( chassis_move_control->chassis_RC->key.v & CHASSIS_BACK_KEY )	vx_temp += REMOTE_MAX_VALUE;
      if( chassis_move_control->chassis_RC->key.v & CHASSIS_LEFT_KEY )	vy_temp -= REMOTE_MAX_VALUE;
      if( chassis_move_control->chassis_RC->key.v & CHASSIS_RIGHT_KEY )	vy_temp += REMOTE_MAX_VALUE;

		// 鼠标右键减速控制
			wz_temp /= chassis_slow_down_rate;
			vy_temp /= chassis_slow_down_rate;
			vx_temp /= chassis_slow_down_rate;
		
      temp = ( my_abs16( vx_temp ) + my_abs16( vy_temp ) ) / REMOTE_MAX_VALUE;
      if( temp != 0 )
      {
        vx_temp /= temp;
        vy_temp /= temp;
      }

      vx_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vx_temp, chassis_move_control->vx_set / CHASSIS_VX_KEYBOARD_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      vy_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vy_temp, chassis_move_control->vy_set / CHASSIS_VY_KEYBOARD_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      //			wz_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, wz_temp, chassis_move_control->vx_set/CHASSIS_VX_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );

      chassis_move_control->vx_set = vx_temp * CHASSIS_VX_KEYBOARD_SEN;
      chassis_move_control->vy_set = vy_temp * CHASSIS_VY_KEYBOARD_SEN;
      chassis_move_control->wz_set = wz_temp;
    }
  }
  else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_TOP_START_MODE)	// 小陀螺旋转模式
  {
    fp32 VX_temp = 0, VY_temp = 0, WZ_temp = 0;
    fp32 angle;
    if( switch_is_mid(chassis_move_control->chassis_RC->rc.s[CtrlChannel]) )	// 如果想被遥控控制，就要拨到中档
    {
      rc_deadline_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], VX_temp, CHASSIS_RC_DEADLINE);
      rc_deadline_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], VY_temp, CHASSIS_RC_DEADLINE);

			VX_temp /= 3;
			VY_temp /= 3;
		
      angle = (temp_chassis_yaw - chassis_move_control->gimbal_mpu6050_yaw) * PI / 180;
      vx_temp = VX_temp * cos(angle) + VY_temp * sin(angle);
      vy_temp = -VX_temp * sin(angle) + VY_temp * cos(angle);
			wz_temp = CHASSIS_TOP_ROTATE_SPEED;
		
			vx_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vx_temp, (fp32)chassis_move_control->vx_set/CHASSIS_VX_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      vy_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vy_temp, (fp32)chassis_move_control->vy_set/CHASSIS_VY_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      wz_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, wz_temp, (fp32)chassis_move_control->wz_set/CHASSIS_WZ_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
		
      chassis_move_control->vx_set = vx_temp * CHASSIS_VX_RC_SEN;
      chassis_move_control->vy_set = vy_temp * CHASSIS_VY_RC_SEN;
      chassis_move_control->wz_set = wz_temp * CHASSIS_WZ_RC_SEN;
    }
    else if( switch_is_up(chassis_move_control->chassis_RC->rc.s[CtrlChannel]) )	// 如果想被键盘控制，就要拨到上档
    {
      int16_t temp = 0, speed_par = REMOTE_MAX_VALUE/3;

      if( chassis_move_control->chassis_RC->key.v & CHASSIS_FRONT_KEY )	VX_temp -= speed_par;
      if( chassis_move_control->chassis_RC->key.v & CHASSIS_BACK_KEY )	VX_temp += speed_par;
      if( chassis_move_control->chassis_RC->key.v & CHASSIS_LEFT_KEY )	VY_temp -= speed_par;
      if( chassis_move_control->chassis_RC->key.v & CHASSIS_RIGHT_KEY )	VY_temp += speed_par;

			temp = ( fabs( VX_temp ) + fabs( VY_temp ) ) / speed_par;
      if( temp != 0 )
      {
        VX_temp /= temp;
        VY_temp /= temp;
      }
      angle = (chassis_move_control->chassis_mpu6050_yaw - chassis_move_control->gimbal_mpu6050_yaw) * PI / 180;
      vx_temp = VX_temp * cos(angle) + VY_temp * sin(angle);
      vy_temp = -VX_temp * sin(angle) + VY_temp * cos(angle);
      wz_temp = CHASSIS_TOP_ROTATE_SPEED;

			vx_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vx_temp, (fp32)chassis_move_control->vx_set/CHASSIS_VX_KEYBOARD_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      vy_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vy_temp, (fp32)chassis_move_control->vy_set/CHASSIS_VY_KEYBOARD_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
      wz_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, wz_temp, (fp32)chassis_move_control->wz_set/CHASSIS_WZ_ANGLE_MOUSE_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
	  
			chassis_move_control->vx_set = vx_temp * CHASSIS_VX_KEYBOARD_SEN;
      chassis_move_control->vy_set = vy_temp * CHASSIS_VY_KEYBOARD_SEN;
      chassis_move_control->wz_set = wz_temp * CHASSIS_WZ_ANGLE_MOUSE_SEN;
    }
  }
  else if( chassis_move_control->chassis_mode == CHASSIS_VECTOR_TOP_CLOSE_MODE )	// 小陀螺结束，底盘强行回归
  {
	  #if 1		// 通过底盘与云台的陀螺仪控制其强制回归，但底盘陀螺仪有问题
	  int16_t temp = 0;
	  fp32 angle_error = chassis_move_control->gimbal_mpu6050_yaw - chassis_move_control->chassis_mpu6050_yaw;
	  fp32 VX_temp = 0, VY_temp = 0;
	  fp32 angle;
	  
	  #if 0		// 这个是自由视角结束以后，根据所在角度用不同的方向回正
	  if( (-180<angle_error) && (angle_error<= 0) )	// 底盘向左转
	  {
		  chassis_move_control->wz_set = angle_error*60;
		  if( chassis_move_control->wz_set < -5000 )	chassis_move_control->wz_set = -5000;
	  }
	  else if( angle_error > 180 )	// 底盘向左转
	  {
		  chassis_move_control->wz_set = -angle_error*60;
		  if( chassis_move_control->wz_set < -5000 )	chassis_move_control->wz_set = -5000;
	  }
	  else if( (0<angle_error) && (angle_error<= 180) )	// 底盘向右转
	  {
		  chassis_move_control->wz_set = angle_error*60;
		  if( chassis_move_control->wz_set > 5000 )	chassis_move_control->wz_set = 5000;
	  }
	  else if( angle_error <= -180 )	// 底盘向右转
	  {
		  chassis_move_control->wz_set = -angle_error*60;
		  if( chassis_move_control->wz_set > 5000 )	chassis_move_control->wz_set = 5000;
	  }
	  #endif
	  #if 1	// 这才是小陀螺结束以后，用同一个方向回正
	  if( (-180<angle_error) && (angle_error<= 0) )	// 底盘向右转
	  {
		  chassis_move_control->wz_set = angle_error*90;
		  if( chassis_move_control->wz_set < -5000 )	chassis_move_control->wz_set = -5000;
		  else if( chassis_move_control->wz_set > -1000 )	chassis_move_control->wz_set = -1000;
	  }
	  else if( angle_error > 180 )	// 底盘向右转
	  {
		  chassis_move_control->wz_set = -angle_error*90;
		  if( chassis_move_control->wz_set < -5000 )	chassis_move_control->wz_set = -5000;
		  else if( chassis_move_control->wz_set > -1000 )	chassis_move_control->wz_set = -1000;
	  }
	  else if( (0<angle_error) && (angle_error<= 30) )	// 底盘向左转
	  {
		  chassis_move_control->wz_set = angle_error*90;
		  if( chassis_move_control->wz_set > 5000 )	chassis_move_control->wz_set = 5000;
		  else if( chassis_move_control->wz_set < 1000 )	chassis_move_control->wz_set = 1000;
	  }
	  else if( (30<angle_error) && (angle_error<= 180) )	// 底盘向右转
	  {
		  chassis_move_control->wz_set = -angle_error*90;
		  if( chassis_move_control->wz_set < -5000 )	chassis_move_control->wz_set = -5000;
		  else if( chassis_move_control->wz_set > -1000 )	chassis_move_control->wz_set = -1000;
	  }
	  else if( (-360<angle_error) && (angle_error<= -330) )	// 底盘向左转
	  {
		  chassis_move_control->wz_set = -angle_error*90;
		  if( chassis_move_control->wz_set > 5000 )	chassis_move_control->wz_set = 5000;
		  else if( chassis_move_control->wz_set < 1000 )	chassis_move_control->wz_set = 1000;
	  }
	  else if( (-330<angle_error) && (angle_error<= -180) )	// 底盘向右转
	  {
		  chassis_move_control->wz_set = angle_error*90;
		  if( chassis_move_control->wz_set < -5000 )	chassis_move_control->wz_set = -5000;
		  else if( chassis_move_control->wz_set > -1000 )	chassis_move_control->wz_set = -1000;
	  }
	  #endif
	  
	  if( chassis_move_control->chassis_RC->key.v & CHASSIS_FRONT_KEY )	VX_temp -= REMOTE_MAX_VALUE;
    if( chassis_move_control->chassis_RC->key.v & CHASSIS_BACK_KEY )	VX_temp += REMOTE_MAX_VALUE;
    if( chassis_move_control->chassis_RC->key.v & CHASSIS_LEFT_KEY )	VY_temp -= REMOTE_MAX_VALUE;
    if( chassis_move_control->chassis_RC->key.v & CHASSIS_RIGHT_KEY )	VY_temp += REMOTE_MAX_VALUE;

	  angle = (chassis_move_control->chassis_mpu6050_yaw - chassis_move_control->gimbal_mpu6050_yaw) * PI / 180;
    vx_temp = VX_temp * cos(angle) + VY_temp * sin(angle);
    vy_temp = -VX_temp * sin(angle) + VY_temp * cos(angle);

	  vx_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vx_temp, (fp32)chassis_move_control->vx_set/CHASSIS_VX_KEYBOARD_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
    vy_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vy_temp, (fp32)chassis_move_control->vy_set/CHASSIS_VY_KEYBOARD_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
	  
	  chassis_move_control->vx_set = vx_temp * CHASSIS_VX_KEYBOARD_SEN;
    chassis_move_control->vy_set = vy_temp * CHASSIS_VY_KEYBOARD_SEN;
	  #endif
	  #if 0
		int16_t temp = 0;
		fp32 VX_temp = 0, VY_temp = 0;
		fp32 angle;
	  
		if( chassis_move_control->chassis_RC->key.v & CHASSIS_FRONT_KEY )	VY_temp += REMOTE_MAX_VALUE;
		if( chassis_move_control->chassis_RC->key.v & CHASSIS_BACK_KEY )	VY_temp -= REMOTE_MAX_VALUE;
		if( chassis_move_control->chassis_RC->key.v & CHASSIS_LEFT_KEY )	VX_temp -= REMOTE_MAX_VALUE;
		if( chassis_move_control->chassis_RC->key.v & CHASSIS_RIGHT_KEY )	VX_temp += REMOTE_MAX_VALUE;

		//temp = ( fabs( VX_temp ) + fabs( VY_temp ) + fabs( WZ_temp ) ) / REMOTE_MAX_VALUE;
		temp = ( fabs( VX_temp ) + fabs( VY_temp ) ) / REMOTE_MAX_VALUE;
		if( temp != 0 )
		{
			VX_temp /= temp;
			VY_temp /= temp;
			//WZ_temp /= temp;
		}
		  
		angle = (-*chassis_move_control->chassis_mpu6050_yaw + *chassis_move_control->gimbal_mpu6050_yaw) * PI / 180;
		vx_temp = VX_temp * cos(angle) + VY_temp * sin(angle);
		vy_temp = -VX_temp * sin(angle) + VY_temp * cos(angle);

		vx_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vx_temp, (fp32)chassis_move_control->vx_set/CHASSIS_VX_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
		vy_temp = Ramp_function( CHASSIS_MIN_BUFFER_VALUE, vy_temp, (fp32)chassis_move_control->vy_set/CHASSIS_VY_RC_SEN, CHASSIS_BUFFER_INCREMENT, 0 );
		  
		chassis_move_control->vx_set = vx_temp * CHASSIS_VX_RC_SEN;
		chassis_move_control->vy_set = vy_temp * CHASSIS_VY_RC_SEN;
	  
	  #endif
  }
}
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
  //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
  wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}
fp32 abs(fp32 a)
{
	if(a<0)  a=-a;
	return a;
}
fp32 power_limit_kp = 50.0f;
fp32 actual_vx, actual_vy, actual_wz;

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop, FunctionalState power_limit_sta, uint8_t mode )
{
  // fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
//  fp32 actual_vx, actual_vy, actual_wz;
  uint8_t i = 0;
  
  actual_vx = chassis_move_control_loop->vx_set;
  actual_vy = chassis_move_control_loop->vy_set;
  actual_wz = chassis_move_control_loop->wz_set;


  #if 1		// 这是根据裁判系统设计的功率控制
	if( power_limit_sta == ENABLE )	// 是否开启功率限制？
	{
		if( mode == 1 )		// 开启裁判系统功率控制
		{
			if( power_heat_data_t.chassis_power_buffer < 20 )	// 当缓冲功率小于20J时，强制减速
			{
				// (应该不会进入这里)
			  // 强制将速度设置在很小值，所有速度值除以4
			  actual_vx /= 4;
			  actual_vy /= 4;
			  actual_wz /= 4;
			}
			else if( power_heat_data_t.chassis_power_buffer < 60 )	// 当缓冲功率小于60J时，开始减速
			{

				#if 1	// 另一种功率限制方式
				fp32 temp = (fp32)power_heat_data_t.chassis_power_buffer/60;
				temp *= temp;
				
				actual_vx *= temp;
				actual_vy *= temp;
				actual_wz *= temp;
				#endif
			}
		}
		else	// 开启电机电流功率控制，功率控制在比限制功率要低一些
		{
			#if 1	// 一种功率限制方式
			fp32 temp;
			temp = chassis_move_control_loop->chassis_power_meter->power - (robot_state.chassis_power_limit - 20);	// 留一点能量给超级电容充电
			if( temp > 0 )	// 如果超功率了
			{
				temp *= power_limit_kp;
//				if( chassis_move_control_loop->vx_set > 0 )		chassis_move_control_loop->vx_set -= temp;
//				else if( chassis_move_control_loop->vx_set < 0 )	chassis_move_control_loop->vx_set += temp;
//				if( chassis_move_control_loop->vy_set > 0 )		chassis_move_control_loop->vy_set -= temp;
//				else if( chassis_move_control_loop->vy_set < 0 )	chassis_move_control_loop->vy_set += temp;
//				if( chassis_move_control_loop->wz_set > 0 )		chassis_move_control_loop->wz_set -= temp;
//				else if( chassis_move_control_loop->wz_set < 0 )	chassis_move_control_loop->wz_set += temp;
							  
				if( chassis_move_control_loop->vx_set > 0 )		abs(chassis_move_control_loop->vx_set = chassis_move_control_loop->vx_set-temp);
				else if( chassis_move_control_loop->vx_set < 0 )	-abs(chassis_move_control_loop->vx_set += temp);
				if( chassis_move_control_loop->vy_set > 0 )		abs(chassis_move_control_loop->vy_set -= temp);
				else if( chassis_move_control_loop->vy_set < 0 )	-abs(chassis_move_control_loop->vy_set += temp);
				if( chassis_move_control_loop->wz_set > 0 )		abs(chassis_move_control_loop->wz_set -= temp);
				else if( chassis_move_control_loop->wz_set < 0 )	-abs(chassis_move_control_loop->wz_set += temp);
			  
				actual_vx = chassis_move_control_loop->vx_set;
				actual_vy = chassis_move_control_loop->vy_set;
				actual_wz = chassis_move_control_loop->wz_set;
				
				//usb_printf("%8.0f,%8.0f,%8.0f\n", actual_vx, actual_vy, actual_wz);
			}
			#endif
		}
	}
  #endif
	
  /*普通的3508电机电流计算*/
  //麦轮运动分解
  chassis_vector_to_mecanum_wheel_speed(actual_vx, actual_vy, actual_wz, wheel_speed);
  // 速度设置
  for( i = 0; i < 4; i++ )
  {
		chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
  }  
  //计算速度环pid
  for (i = 0; i < 4; i++)
  {
		PID_calc(&chassis_move_control_loop->motor_speed_pid[i],
		chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
  }
  
  /*赋值电流值*/
  for (i = 0; i < 4; i++)
  {
    chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
  }
}

static void chassis_tim( chassis_move_t *chassis_tim )
{
	//robot_state.chassis_power_limit=80;
	fp32 temp = robot_state.chassis_power_limit;
	time_count++;
	reset_count++;	// 复位计时
	
	if( chassis_tim->chassis_super_cap->cap_vot < SUPER_CUP_INPUT_VOLTAGE_MIN )
	{
		chassis_tim->super_cap_sta = SUPER_CAP_CHARGE;
	}
	else
	{
		// 相对于满电状态下的SOC
		chassis_tim->super_cap_per = (chassis_tim->chassis_super_cap->cap_vot-SUPER_CUP_INPUT_VOLTAGE_MIN)/(25-SUPER_CUP_INPUT_VOLTAGE_MIN);
		// 实际状态下的SOC
		chassis_tim->super_cap_soc = (chassis_tim->chassis_super_cap->cap_vot-SUPER_CUP_INPUT_VOLTAGE_MIN)/(chassis_tim->chassis_super_cap->input_vot-SUPER_CUP_INPUT_VOLTAGE_MIN - 2);
		
		if( chassis_tim->super_cap_sta == SUPER_CAP_CHARGE )
		{
			if( chassis_tim->super_cap_soc > 0.97 )
			{
				chassis_tim->super_cap_sta = SUPER_CAP_CLOSE;
			}
		}
		
	}
	
	if( time_count%50 == 0 )	// 给超级电容设置参数，方便开启超级电容
	{
//		wz_move_len = robot_state.chassis_power_limit/10;
		switch( chassis_tim->super_cap_sta )
		{
			case SUPER_CAP_BEGIN:
			{
				x_y_move_len = temp/9;
				chassis_tim->super_cap_setting = 6500;
			}
			case SUPER_CAP_CLOSE:	// 超级电容关闭模式，直接开启超级电容最大充电功率
			{
				x_y_move_len = temp/9;
				chassis_tim->super_cap_setting = 13000;
				break;
			}
			case SUPER_CAP_OPEN:	// 超级电容开启模式，超级电容充电功率设置为裁判系统的功率限制
			{
				x_y_move_len = temp/5;
				chassis_tim->super_cap_setting = robot_state.chassis_power_limit*100;
				break;
			}
			case SUPER_CAP_CHARGE:	// 超级电容充电模式
			{
				x_y_move_len = temp/9;
				chassis_tim->super_cap_setting = robot_state.chassis_power_limit*100;
				break;
			}
			case SUPER_CAP_OVER:	// 当超级电容没能量了，超级电容充电功率设置为裁判系统的功率限制
			{
				x_y_move_len = temp/9;
				chassis_tim->super_cap_setting = robot_state.chassis_power_limit*100;
				break;
			}
//			case SUPER_CAP_NO_CONNECT:	// 超级电容还在充电的时候，为了不超功率，设置了这个模式
//			{
//				x_y_move_len = temp/9;
//				chassis_tim->super_cap_setting = robot_state.chassis_power_limit*100;
//				break;
//			}
			default:
			{
				x_y_move_len = temp/9;
				chassis_tim->super_cap_setting = robot_state.chassis_power_limit*100;
				break;
			}
		}
		chassis_set_super_cap_value( chassis_move.super_cap_setting );

	}
//	else if( time_count%224 == 0 )
//	{
//		if( get_robot_id() < 10 )	// 是红方机器人，告诉视觉我们是红方
//		{
//		}
//		else		// 是蓝方机器人，告诉视觉我们是蓝方
//		{
//		}
//		
//		// 可以将超级电容功率，剩余弹丸数量发送给射手
//		
//	}
	
	if( time_count >= 999 )	time_count = 0;
	if( reset_count >= 5000 )	reset_count = 5000;
	
	int16_t ecd_error = chassis_tim->chassis_yaw_motor->gimbal_motor_measure->ecd - YAW_RELATIVE_INIT_ANGLE;
	fp32 angle_error = ecd_error*360;
	angle_error /= GM6020_MAX_ENCODER;
	
	chassis_tim->gimbal_mpu6050_yaw = chassis_tim->chassis_mpu6050_yaw + angle_error;
	if( chassis_tim->gimbal_mpu6050_yaw > 180 )		chassis_tim->gimbal_mpu6050_yaw -= 360;
	else if( chassis_tim->gimbal_mpu6050_yaw < -180 )	chassis_tim->gimbal_mpu6050_yaw += 360;
	temp_gimbal_yaw = chassis_tim->gimbal_mpu6050_yaw;
}

static void chassis_death( chassis_move_t *chassis_death )
{
	uint8_t i = 0;
	
	// PID算法中I清零
	chassis_death->chassis_mouse_control_pid.Iout = 0;
	chassis_death->chassis_angle_pid.Iout = 0;
	for( i = 0; i < 4; i++ )
	{
		chassis_death->motor_speed_pid[i].Iout = 0;
	}
	
	// 模式设置
	chassis_death->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
	chassis_death->super_cap_sta=SUPER_CAP_CLOSE;
	CAN_cmd_chassis(chassis_death->motor_speed_pid[0].Iout, chassis_death->motor_speed_pid[1].Iout,
												chassis_death->motor_speed_pid[2].Iout, chassis_death->motor_speed_pid[3].Iout);
	// 计数器清零
	loop_init_count = 0;
	time_count = 0;
}
