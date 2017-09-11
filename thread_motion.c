
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:		thread_motion.c
  Author:			Raymond
  Date:				2017.7.7
  Version:        
  Description:    // ������ϸ˵���˳����ļ���ɵ���Ҫ���ܣ�������ģ��
                  // �����Ľӿڣ����ֵ��ȡֵ��Χ�����弰������Ŀ�
                  // �ơ�˳�򡢶����������ȹ�ϵ
  History:        // �޸���ʷ��¼�б���ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸�
                  // �߼��޸����ݼ���  
    1. Date:
       Author:
       Modification:
    2. ...
*************************************************/


#include "stm32f4xx.h"
#include <rtthread.h>
//#include <string.h>
#include <stdio.h>
#include "mower_common.h"
#include "motion_control.h"
#include "global.h"
#include "motor_control.h"

#include "usart_driver.h"
#include "lcd12864_io_spi.h"

/*********Temp Functions******************/

/*******************************************
 ������
 ������
 ע�⣺
 *******************************************/
 
/*******************************************
 �����������ó��� �Ƕȱջ� �߷��� ��ʱ��
 ������speed �ٶ�
       side_length �߳���λΪʱ�� 1���Ӷ�Ӧ�߳�50
 ע�⣺
 *******************************************/
static void mower_motion_square(T_motion* motion,float speed, uint32_t side_length)
{
	uint8_t state = 0;
	uint32_t count = 0;
	
	rt_uint32_t recved;
	
	Motion_Start_2D_Angle(&motion->tracker,1.0f,0.0f,speed);
	while(1)
	{
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

		Motion_Get_Position_2D(&motion->tracker.sense);
		Motion_Get_Sensor(&motion->tracker.sense);
		
		if(state == 0 || state == 2 || state == 4 || state == 6)
		{
			if(count >= side_length)
			{
				state ++;
				count = 0;
			}
			count++;
			Motion_Run_Tracker(&motion->tracker);
		}
		else if(state == 1)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_y < -0.96f)
			{
				Motion_Start_2D_Angle(&motion->tracker,0.0f,-1.0f,speed);
				state ++;
			}
		}
		else if(state == 3)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_x < -0.96f)
			{
				Motion_Start_2D_Angle(&motion->tracker,-1.0f,0.0f,speed);
				state ++;
			}
		}
		else if(state == 5)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_y > 0.96f)
			{
				Motion_Start_2D_Angle(&motion->tracker,0.0f,1.0f,speed);
				state ++;
			}
		}
		else if(state == 7)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_x > 0.96f)
			{
				Motion_Start_2D_Angle(&motion->tracker,1.0f,0.0f,speed);
				state = 0;
			}
		}
		Motion_Process_Motor_Speed(motion);
		update_motor_control();
	}
}

/*******************************************
 �����������ó��� λ�ñջ� �߷��� ��ʱ��
 ������speed �ٶ�
       side_length �߳���λΪ��
 ע�⣺
 *******************************************/
static void mower_motion_square_position(T_motion* motion,float speed, float side_length)
{
	uint8_t state = 0;
	uint32_t count = 0;
	float point_x = 0;
	float point_y = 0;
	
	rt_uint32_t recved;
	
	point_x = side_length;
	point_y = 0.0f;
	Motion_Start_2D_Line(&motion->tracker,point_x,point_y,1.0f,0.0f,speed);
	while(1)
	{
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

		Motion_Get_Position_2D(&motion->tracker.sense);
		Motion_Get_Sensor(&motion->tracker.sense);
		
		if(state == 0 || state == 2 || state == 4 || state == 6)
		{
			float dist2 = (point_x - motion->tracker.sense.pos_x) * (point_x - motion->tracker.sense.pos_x) + (point_y - motion->tracker.sense.pos_y) * (point_y - motion->tracker.sense.pos_y);
			if(dist2 < 0.0025f)  //dist < 0.05m = 5cm
				state ++;

			Motion_Run_Tracker(&motion->tracker);
		}
		else if(state == 1)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_y < -0.99f)
			{
				point_x = side_length;
				point_y = -side_length;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,0.0f,-1.0f,speed);	
				state ++;
			}
		}
		else if(state == 3)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_x < -0.99f)
			{
				point_x = 0.0f;
				point_y = -side_length;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,-1.0f,0.0f,speed);
				state ++;
			}
		}
		else if(state == 5)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_y > 0.99f)
			{
				point_x = 0.0f;
				point_y = 0.0f;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,0.0f,1.0f,speed);
				state ++;
			}
		}
		else if(state == 7)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_x > 0.99f)
			{
				point_x = side_length;
				point_y = 0.0f;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,1.0f,0.0f,speed);
				state = 0;
			}
		}
		Motion_Process_Motor_Speed(motion);
		update_motor_control();
	}
}

/*******************************************
 �����������ó��� �Ƕȱջ� ��Բ ��ʱ��
 ������speed �ٶ�
       side_length �߳���λΪʱ�� 1���Ӷ�Ӧ�߳�50
 ע�⣺
 *******************************************/
static void mower_motion_circle(T_motion* motion,float line_speed, float angle_speed)
{
	float current_angle = 0;
	float tx = 1.0f;
	float ty = 0.0f;
	rt_uint32_t recved;
	uint8_t buf[100] = "hello!!!!!!!!!\n\r";
	while(1)
	{
		volatile float sin = sinf(current_angle);
		volatile float cos = cosf(current_angle);
		volatile float x = tx;
		volatile float y = ty;
		volatile float tx = x*cos-y*sin;
		volatile float ty = x*sin+y*cos;
		
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		
		//rt_debug(buf,16);
		Motion_Get_Position_2D(&motion->tracker.sense);
		Motion_Get_Sensor(&motion->tracker.sense);
		Motion_Start_2D_Angle(&motion->tracker,tx,ty,line_speed);
		Motion_Run_Tracker(&motion->tracker);
		Motion_Process_Motor_Speed(motion);
		
		current_angle -= angle_speed;
		if(current_angle < 0)
		{
			current_angle += 2*3.1415926;
		}
		
		update_motor_control();
	}
}

/*�ڷ����ջ�ռ�ʱ������Ҫ����*/
ALIGN(RT_ALIGN_SIZE)
char thread_motion_stack[1024];
struct rt_thread thread_motion;

void mower_motion_thread(void* parameter)
{
  rt_uint32_t recved;
	T_motion motion;
	
	//�ȴ�IMU GPS��ʼ�� ����Ӧɾ��
	rt_thread_delay(2000);
	
	//��ʼ��motion���� ��ʹ��
	Motion_Init(&motion,1);
	
	//�����ó���
	//mower_motion_square_position(&motion,600,4.0f);//4 meter
	//mower_motion_square(&motion,600,1000);//20 sec
	//mower_motion_circle(&motion,300,0.0031416);//40 sec
	
	Motion_Zigzag_Start(&motion, 0.3, 1.0f, 0.0f, T_MOTION_ZIGZAG_TURN_COUNTERCLOCKWISE)
	//Motion_Start_2D_Angle(&motion.tracker,1.0f,0.0f,500);
	//Motion_Start_Mag_Line(&motion.tracker,500,MOTION_MAG_LINE_DIRECT);
	//Motion_Start_2D_Line(&motion.tracker,0.0f,0.0f,1.0f,0.0f,500);
	//Motion_Zigzag_Start(&motion,1000,1,0,T_MOTION_ZIGZAG_TURN_CLOCKWISE);
	while (1)
	{
		//�ȴ�event 20msһ��
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		
		//���������ݸ���
		Motion_Get_Position_2D(&motion.tracker.sense); //get the position and orientation of the vehicle
		Motion_Get_Sensor(&motion.tracker.sense);      // get other sensor data(sonar, bumper, etc.)



		//Motion_Update(&motion.tracker.path_imu); 		//zigzag state: update nextpoint



		//�ϰ�����ٸ��� �����ڼ�����ײ�ȴ�������
		//Motion_Process_Obstacle(&motion);
		
		//���п��������������ѡ��
		
		//���дŵ��߿�����
		//Motion_Run_Mag_Line(&motion.tracker);
		//����λ�˿�����
		//Motion_Run_Tracker(&motion.tracker);
		//���й��̿��������繤����·���ȣ�
		Motion_Run(&motion);
		
		//������ٶȿ��� ��������ӿڵ���
		Motion_Process_Motor_Speed(&motion);

		//Debug
		//rt_kprintf("angle = %d                x = %d                 y = %d \n\r",(int)(eul_rad[0]*10*57.3), (int)(motion.tracker.sense.pos_x*100), (int)(motion.tracker.sense.pos_y*100));
		//rt_kprintf("left = %d, right =%d \n\r",(int)(motion.tracker.line_vel),(int)(motion.tracker.angular_vel));
		//rt_kprintf("left = %d, right = %d \r\n\r",leftsensor_data,rightsensor_data);
	#if 0//def CUSTOMER_SERIAL_DEBUG
		//rt_kprintf("\r\n...>>>motion_thread");
	#endif
		
		//��������ٶȸ���ָ��
		update_motor_control();
	}
}







