#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "main.h"

#include "Gimbal_Task.h"
typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //��̨����
  GIMBAL_INIT,           //��̨��ʼ��
  GIMBAL_CALI,           //��̨У׼
  GIMBAL_ABSOLUTE_ANGLE, //��̨�����Ǿ��ԽǶȿ���
  GIMBAL_RELATIVE_ANGLE, //��̨�������ֵ��ԽǶȿ���
  GIMBAL_MOTIONLESS,     //��̨��ң����������һ��ʱ��󱣳ֲ���������������Ư��
} gimbal_behaviour_e;




#endif
