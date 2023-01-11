

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

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

//ѡ�����״̬ ����ͨ����
#define ModeChannel 0
#define CtrlChannel 1
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN (x_y_move_len)		// 10.0f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN (-x_y_move_len)		// 10.0f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN (-x_y_move_len)		// 10.0f

// ���Կ�����ת����
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

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//����ҡ�ڰ���
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
//#define CHASSIS_ROTATE_L_KEY KEY_PRESSED_OFFSET_Q
//#define CHASSIS_ROTATE_R_KEY KEY_PRESSED_OFFSET_E
#define CHASSIS_TOP_MODE_START	KEY_PRESSED_OFFSET_Q
#define CHASSIS_TOP_MODE_CLOSE	KEY_PRESSED_OFFSET_E

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 1.0f			//0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//���̵������ٶ�
//#define MAX_WHEEL_SPEED 4.0f
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED	1000.0f
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 1000.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 1000.0f
//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0

//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 20

// ң�ز���
#define REMOTE_MAX_VALUE		160

// �͵�ѹ����
#define CHASSIS_MIN_VOLTAGE		17.0f

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 10.0f
#define M3505_MOTOR_SPEED_PID_KI 0.5f
#define M3505_MOTOR_SPEED_PID_KD -1.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT MAX_MOTOR_CAN_CURRENT

//������������תPID
#define CHASSIS_MOUSE_PID_KP 20.0f
#define CHASSIS_MOUSE_PID_KI 0.0f
#define CHASSIS_MOUSE_PID_KD 0.0f
#define CHASSIS_MOUSE_PID_MAX_OUT REMOTE_MAX_VALUE
#define CHASSIS_MOUSE_PID_MAX_IOUT REMOTE_MAX_VALUE

//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 70.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.0f

//--------------------------------------------------------
/**********************�������Ʋ���***********************/
//������ƹ��� 
#define SUPER_CUP_INPUT_VOLTAGE_MIN		18.0f				// ��������ģ����С�������ѹ
#define SUPER_CUP_INPUT_VOLTAGE_WARNING	22.0f				// ��������ģ�龯���ѹ
#define SUPER_CAP_INPUT_VOLTAGE_ERROR	1.5f				// (SUPER_CUP_INPUT_VOLTAGE_WARNING - SUPER_CUP_INPUT_VOLTAGE_MIN)

#define CHASSIS_SPEED_BUFFER_PRA1	0.01f
#define CHASSIS_SPEED_BUFFER_PRA2	0.99f
#define CHASSIS_MIN_BUFFER_VALUE	5.0f
#define CHASSIS_BUFFER_INCREMENT	3

//--------------------------------------------------------
/**********************����Ҽ�����***********************/
#define CHASSIS_VX_MAX						5000
#define CHASSIS_VY_MAX						5000
#define CHASSIS_WZ_MAX						5000

//--------------------------------------------------------
/**********************����Ҽ�����***********************/
#define CHASSIS_SLOW_DOWN_START				2.0f
//#define CHASSIS_PITCH_SLOW_DOWN_START		4.0f
#define CHASSIS_SLOW_DOWN_END				1.0f

//--------------------------------------------------------
/**********************����yaw��չ***********************/
#define CHASSIS_YAW_EXTEND_COUNT_MIN		1
#define CHASSIS_YAW_EXTEND_COUNT_MAX		6
#define CHASSIS_YAW_EXTEND_COUNT_INIT		3

//--------------------------------------------------------
/**********************��������***********************/
#define CHASSIS_ANGLE_RETURN_ERROR	0.3
#define CHASSIS_ANGLE_ROTOAT_ERROR	0.01
#define CHASSIS_TOP_ROTATE_SPEED	700
#define CHASSIS_VECTOR_TOP_CLOSE_ERROR 5 //С���ݽ�������ʱ���ж�������λ���������ֵ


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
	//SUPER_CAP_NO_CONNECT,	// �������ݳ��ģʽ
	SUPER_CAP_OPEN,			// �������ݿ���ģʽ
	SUPER_CAP_CLOSE,		// �������ݹر�ģʽ
	SUPER_CAP_OVER,			// �������ݹ���
	SUPER_CAP_CHARGE,		// �������ݳ��ģʽ
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
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
  const Gimbal_Motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
  const Gimbal_Motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
  const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //���̿���״̬��
  chassis_mode_e last_chassis_mode;          //�����ϴο���״̬��
  Chassis_Motor_t motor_chassis[4];          //���̵������
  pid_type_def motor_speed_pid[4];             //���̵���ٶ�pid
  pid_type_def chassis_mouse_control_pid;        //����������pid
  pid_type_def chassis_angle_pid;              //���̸���Ƕ�pid

  const can_power_meter_t *chassis_power_meter;	// ���ʲ���
  const can_super_cap_t *chassis_super_cap;	// ��������

	uint16_t super_cap_setting;		// �������ݹ�������
	super_cap_sta_e super_cap_sta;	// ��������״̬����
	fp32 super_cap_voltage_min;     //����������С��ѹֵ
	
	/***
	****/
	fp32 super_cap_soc;				// ��������SOC�������ʣ�������
	fp32 super_cap_per;				// �������ݵİٷֱȣ���������������ʣ�������
//  fp32 super_cap_voltage_warning;	// �������ݾ����ѹֵ
//	fp32 super_cap_voltage_diff;	// �������ݵ�ѹ��

  fp32 vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 wz_angle_set;               //������ʱҪ�������ٽǶ�

  fp32 top_mode_dir;				// С����ģʽʱ�ķ��򣬽Ƕ�ֵ

  fp32 *gimbal_mpu6050_yaw_gyro;		// ��չ����̨������yaw
  fp32 *gimbal_mpu6050_pitch_gyro;	// ��չ����̨������pitch
  fp32 *gimbal_mpu6050_roll_gyro;		// ��չ����̨������roll
  fp32 gimbal_mpu6050_yaw;		// ��չ����̨������yaw
  fp32 *gimbal_mpu6050_pitch;		// ��չ����̨������pitch
  fp32 *gimbal_mpu6050_roll;		// ��չ����̨������roll

	// ����Ϊ��չ
	//externd_angle_t
//  fp32 chassis_yaw;				// C���Դ�yaw��������
//  fp32 chassis_pitch;			// C���Դ�pitch��������
//  fp32 chassis_roll;			// C���Դ�roll��������

  fp32 chassis_mpu6050_yaw;		// ��չ�����������yaw
  fp32 *chassis_mpu6050_pitch;	// ��չ�����������pitch
  fp32 *chassis_mpu6050_roll;		// ��չ�����������roll
  fp32 *chassis_mpu6050_yaw_gyro;		// ��չ�����������yaw
  fp32 *chassis_mpu6050_pitch_gyro;	// ��չ�����������pitch
  fp32 *chassis_mpu6050_roll_gyro;	// ��չ�����������roll
} chassis_move_t;

extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);


#endif
