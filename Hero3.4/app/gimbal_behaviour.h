#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "main.h"

#include "Gimbal_Task.h"
typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //云台无力
  GIMBAL_INIT,           //云台初始化
  GIMBAL_CALI,           //云台校准
  GIMBAL_ABSOLUTE_ANGLE, //云台陀螺仪绝对角度控制
  GIMBAL_RELATIVE_ANGLE, //云台电机编码值相对角度控制
  GIMBAL_MOTIONLESS,     //云台在遥控器无输入一段时间后保持不动，避免陀螺仪漂移
} gimbal_behaviour_e;




#endif
