/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb输出错误信息
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"
#include "referee.h"
#include "CAN_receive.h"
#include "chassis_task.h"

//#include "gimbal_task.h"

//static void usb_printf(const char *fmt,...);
//extern fp32 data_buf1[1000];
//extern uint16_t data_i;
extern float ANGLE_CHANGE;
int i;
extern chassis_move_t chassis_move;
extern ext_game_robot_state_t robot_state;
extern volatile uint8_t chassis_return_mode;
#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920
uint32_t Itemp=0;
ext_student_interactive_header_data_t custom_grapic_draw;			//自定义图像绘制
ext_client_custom_graphic_seven_t custom_graphic;	//自定义图像

ext_student_interactive_header_data_t_char custom_character_draw;//char1
ext_client_custom_character_t custom_character;

ext_student_interactive_header_data_t_char custom_character_draw_2;//char2
ext_client_custom_character_t custom_character_2;

ext_student_interactive_header_data_t_char custom_character_draw_3;//char3

ext_student_interactive_header_data_t_char custom_character_draw_4;//char4

ext_student_interactive_header_data_t_num custom_number_draw;//自定义数字数据
ext_client_custom_number_seven_t custom_number;
/*
ext_student_interactive_header_data_t_float custom_float_draw;
ext_client_custom_float_t custom_float;
*/
int h=20;
static void U32_To_Str( uint32_t num, char* str );
char tempch[30];
char tempch2[30];
char tempch3[30];
char tempch4[30];
void usb_task(void const * argument)
{
	vTaskDelay(500);	// 等待一段时间，等获取到裁判系统的数据以后再工作
	//初始化图形数据变量
	//自定义图形绘制
	
	uint8_t sender_id;
	uint16_t receiver_id;
	
	sender_id = (uint8_t)robot_state.robot_id;	
	if( sender_id < 10 )	// 为红方
	{
		receiver_id = sender_id + 0x0100;
	}
	else	// 为蓝方
	{
		receiver_id = sender_id + 256;
	}
	
//	receiver_id = 0x167;
//	sender_id = 103;
	
		custom_grapic_draw.data_cmd_id=0x0104;//绘制七个图形（内容ID，查询裁判系统手册）
		
			custom_grapic_draw.sender_ID=sender_id;//发送者ID，机器人对应ID，此处为蓝方英雄
			custom_grapic_draw.receiver_ID=receiver_id;//接收者ID，操作手客户端ID，此处为蓝方英雄操作手客户端
		//自定义图像数据
		{
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 97;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 97;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;//图形名
		//上面三个字节代表的是图形名，用于图形索引，可自行定义
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=1;//图形类型，0为直线，其他的查看用户手册
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=1;//图层数
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=0;//颜色
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=2;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=947;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=401;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x=973;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y=426;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius=0;
	}
				//自定义图像数据
		{
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 96;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[1] = 96;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[2] = 0;//图形名
		//上面三个字节代表的是图形名，用于图形索引，可自行定义
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=1;//图形类型，0为直线，其他的查看用户手册
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=1;//图层数
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=1;//颜色
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].width=2;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_x=1341;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_y=472;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_x=1476;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_y=607;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].radius=0;
	}
		{
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 95;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[1] = 95;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[2] = 0;//图形名
		//上面三个字节代表的是图形名，用于图形索引，可自行定义
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=1;//图形类型，0为直线，其他的查看用户手册
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=1;//图层数
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=2;//颜色
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=2;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=878;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=414;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_x=1043;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_y=414;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].radius=0;
	}
		{
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[0] = 94;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[1] = 94;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[2] = 0;//图形名
		//上面三个字节代表的是图形名，用于图形索引，可自行定义
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_tpye=0;//图形类型，0为直线，其他的查看用户手册
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].layer=2;//图层数
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].color=3;//颜色
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].width=2;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_x=947;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_y=358;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_x=973;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_y=358;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].radius=0;
	}
		{
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[0] = 88;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[1] = 88;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[2] = 0;//图形名
		//上面三个字节代表的是图形名，用于图形索引，可自行定义
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_tpye=0;//图形类型，0为直线，其他的查看用户手册
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].layer=1;//图层数
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].color=3;//颜色
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].width=2;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_x=947;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_y=230;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_x=973;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_y=230;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].radius=0;
	}
		//自定义字符
	custom_character_draw.data_cmd_id=0x0110;
	custom_character_draw.sender_ID=sender_id;
		custom_character_draw.receiver_ID=receiver_id;
	{
		custom_character_draw.character_custom.grapic_data_struct.graphic_name[0]=93;
		custom_character_draw.character_custom.grapic_data_struct.graphic_name[1]=93;
		custom_character_draw.character_custom.grapic_data_struct.graphic_name[2]=0;
		
		custom_character_draw.character_custom.grapic_data_struct.operate_tpye=1;
		custom_character_draw.character_custom.grapic_data_struct.graphic_tpye=7;
		custom_character_draw.character_custom.grapic_data_struct.layer=2;
		custom_character_draw.character_custom.grapic_data_struct.color=6;
		custom_character_draw.character_custom.grapic_data_struct.start_angle=20; //字体大小
		custom_character_draw.character_custom.grapic_data_struct.end_angle=20;   //字符长度
		custom_character_draw.character_custom.grapic_data_struct.width=2;				//线条宽度
		custom_character_draw.character_custom.grapic_data_struct.start_x=SCREEN_LENGTH/2-800;
		custom_character_draw.character_custom.grapic_data_struct.start_y=SCREEN_WIDTH/2+300;
		strcpy(tempch,"         NORMOL");
		for(i=0;i<16;i++)
		{
			custom_character_draw.character_custom.data[i]=tempch[i];
		}
	}
	//自定义字符2
	custom_character_draw_2.data_cmd_id=0x0110;
	custom_character_draw_2.sender_ID=sender_id;
	custom_character_draw_2.receiver_ID=receiver_id;
		{
		custom_character_draw_2.character_custom.grapic_data_struct.graphic_name[0]=92;
		custom_character_draw_2.character_custom.grapic_data_struct.graphic_name[1]=92;
		custom_character_draw_2.character_custom.grapic_data_struct.graphic_name[2]=0;
		
		custom_character_draw_2.character_custom.grapic_data_struct.operate_tpye=1;
		custom_character_draw_2.character_custom.grapic_data_struct.graphic_tpye=7;
		custom_character_draw_2.character_custom.grapic_data_struct.layer=2;
		custom_character_draw_2.character_custom.grapic_data_struct.color=6;
		custom_character_draw_2.character_custom.grapic_data_struct.start_angle=20; //字体大小
		custom_character_draw_2.character_custom.grapic_data_struct.end_angle=20;   //字符长度
		custom_character_draw_2.character_custom.grapic_data_struct.width=2;				//线条宽度
		custom_character_draw_2.character_custom.grapic_data_struct.start_x=SCREEN_LENGTH/2-800;
		custom_character_draw_2.character_custom.grapic_data_struct.start_y=SCREEN_WIDTH/2+250;
		strcpy(tempch2,"         Top:ON");
		for(i=0;i<16;i++)
		{
			custom_character_draw_2.character_custom.data[i]=tempch2[i];
		}
	}
		//自定义字符3
	custom_character_draw_3.data_cmd_id=0x0110;
	custom_character_draw_3.sender_ID=sender_id;
	custom_character_draw_3.receiver_ID=receiver_id;
		{
		custom_character_draw_3.character_custom.grapic_data_struct.graphic_name[0]=91;
		custom_character_draw_3.character_custom.grapic_data_struct.graphic_name[1]=91;
		custom_character_draw_3.character_custom.grapic_data_struct.graphic_name[2]=0;
		
		custom_character_draw_3.character_custom.grapic_data_struct.operate_tpye=1;
		custom_character_draw_3.character_custom.grapic_data_struct.graphic_tpye=7;
		custom_character_draw_3.character_custom.grapic_data_struct.layer=2;
		custom_character_draw_3.character_custom.grapic_data_struct.color=6;
		custom_character_draw_3.character_custom.grapic_data_struct.start_angle=20; //字体大小
		custom_character_draw_3.character_custom.grapic_data_struct.end_angle=20;   //字符长度
		custom_character_draw_3.character_custom.grapic_data_struct.width=2;				//线条宽度
		custom_character_draw_3.character_custom.grapic_data_struct.start_x=SCREEN_LENGTH/2-800;
		custom_character_draw_3.character_custom.grapic_data_struct.start_y=SCREEN_WIDTH/2+200;
			strcpy(tempch3," V     value:  ");
		for(i=0;i<16;i++)
		{
			custom_character_draw_3.character_custom.data[i]=tempch3[i];
		}
	}
				//自定义字符4
	custom_character_draw_4.data_cmd_id=0x0110;
	custom_character_draw_4.sender_ID=sender_id;
	custom_character_draw_4.receiver_ID=receiver_id;
		{
		custom_character_draw_4.character_custom.grapic_data_struct.graphic_name[0]=90;
		custom_character_draw_4.character_custom.grapic_data_struct.graphic_name[1]=90;
		custom_character_draw_4.character_custom.grapic_data_struct.graphic_name[2]=0;
		
		custom_character_draw_4.character_custom.grapic_data_struct.operate_tpye=1;
		custom_character_draw_4.character_custom.grapic_data_struct.graphic_tpye=7;
		custom_character_draw_4.character_custom.grapic_data_struct.layer=2;
		custom_character_draw_4.character_custom.grapic_data_struct.color=6;
		custom_character_draw_4.character_custom.grapic_data_struct.start_angle=20; //字体大小
		custom_character_draw_4.character_custom.grapic_data_struct.end_angle=20;   //字符长度
		custom_character_draw_4.character_custom.grapic_data_struct.width=2;				//线条宽度
		custom_character_draw_4.character_custom.grapic_data_struct.start_x=SCREEN_LENGTH/2-800;
		custom_character_draw_4.character_custom.grapic_data_struct.start_y=SCREEN_WIDTH/2+150;
			strcpy(tempch4,"Bullet hatch:OFF");
		for(i=0;i<16;i++)
		{
			custom_character_draw_4.character_custom.data[i]=tempch4[i];
		}
	}
	//自定义数字
	custom_number_draw.data_cmd_id=0x0104;
	custom_number_draw.sender_ID=sender_id;
	custom_number_draw.receiver_ID=receiver_id;
	{
		custom_number_draw.number_custom.number_data_struct[0].graphic_name[0] = 89;
		custom_number_draw.number_custom.number_data_struct[0].graphic_name[1]= 89;
		custom_number_draw.number_custom.number_data_struct[0].graphic_name[1]= 0;//图形名
		//上面三个字节代表的是图形名，用于图形索引，可自行定义
		custom_number_draw.number_custom.number_data_struct[0].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
		custom_number_draw.number_custom.number_data_struct[0].graphic_tpye=5;//图形类型，0为直线，其他的查看用户手册
		custom_number_draw.number_custom.number_data_struct[0].layer=1;//图层数
		custom_number_draw.number_custom.number_data_struct[0].color=6;//颜色
		custom_number_draw.number_custom.number_data_struct[0].start_angle=20;
		custom_number_draw.number_custom.number_data_struct[0].width=2;
		custom_number_draw.number_custom.number_data_struct[0].start_x=SCREEN_LENGTH/2-550;
		custom_number_draw.number_custom.number_data_struct[0].start_y=SCREEN_WIDTH/2+200;
		custom_number_draw.number_custom.number_data_struct[0].num=chassis_move.super_cap_soc*1000;
	}
	
		
		referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_grapic_draw,sizeof(custom_grapic_draw),0);
//		referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_character_draw,sizeof(custom_character_draw),1);
//		referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_character_draw_2,sizeof(custom_character_draw_2),1);
//		referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_character_draw_3,sizeof(custom_character_draw_3),1);
//		referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_character_draw_4,sizeof(custom_character_draw_4),1);
		referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_number_draw,sizeof(custom_number_draw),0);
    while(1)
    {
 			custom_character_draw.character_custom.grapic_data_struct.operate_tpye=1;
			custom_number_draw.number_custom.number_data_struct[0].operate_tpye=1;	
			referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_grapic_draw,sizeof(custom_grapic_draw),0);
			referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_number_draw,sizeof(custom_number_draw),0);
			custom_number_draw.number_custom.number_data_struct[0].num= chassis_move.super_cap_soc*1000;		
		sender_id = (uint8_t)robot_state.robot_id;	
			if( sender_id < 10 )	// 为红方
			{
				receiver_id = sender_id + 0x0100;
			}
			else	// 为蓝方
			{
				receiver_id = sender_id + 256;
			}
		Itemp++;
				//修改操作
		custom_character_draw.character_custom.grapic_data_struct.operate_tpye=2;
			
		custom_number_draw.number_custom.number_data_struct[0].operate_tpye=2;//2为修改，修改之前要初始化增加
		referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_number_draw,sizeof(custom_number_draw),0);
		//模式改变：读到对应的标志位时，更改原来的对应字符串为“XXXXXX：ON/OFF”，再用referee_data_pack_handle重新发送一下数据
		
		referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_grapic_draw,sizeof(custom_grapic_draw),0);
			if(chassis_move.chassis_RC->key.v&KEY_PRESSED_OFFSET_B)
		{
			referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_number_draw,sizeof(custom_number_draw),0);
			referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_grapic_draw,sizeof(custom_grapic_draw),0);
		}
			
			
			vTaskDelay(100);
    }
}

static void U32_To_Str( uint32_t num, char* str )
{
	uint16_t count = 0;
	uint32_t temp_num = num;
	
	do
	{
		count++;
		temp_num = temp_num/10;
	}while( temp_num );
	
	str[count] = '\0';
	while( count-- )
	{
		temp_num = num%10;
		num /= 10;
		str[count] = temp_num + '0';
	}
}
