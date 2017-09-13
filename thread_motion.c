
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:		thread_motion.c
  Author:			Raymond
  Date:				2017.7.7
  Version:        
  Description:    // 用于详细说明此程序文件完成的主要功能，与其他模块
                  // 或函数的接口，输出值、取值范围、含义及参数间的控
                  // 制、顺序、独立或依赖等关系
  History:        // 修改历史记录列表，每条修改记录应包括修改日期、修改
                  // 者及修改内容简述  
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
#include "pi.h"
#include "usart_driver.h"
#include "lcd12864_io_spi.h"

/*********Temp Functions******************/

/*******************************************
 描述：
 变量：
 注意：
 *******************************************/
 
/*******************************************
 描述：测试用程序 角度闭环 走方块 逆时针
 变量：speed 速度
       side_length 边长单位为时间 1秒钟对应边长50
 注意：
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
 描述：测试用程序 位置闭环 走方块 逆时针
 变量：speed 速度
       side_length 边长单位为米
 注意：
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
		
		//debug
		//rt_kprintf("linear velocity = %d, angular velocity =%d \n\r",(int) (motion.tracker.line_vel*1000), (int) (motion.tracker.angular_vel*1000) );

		if(state == 0 || state == 2 || state == 4 || state == 6)
		{
			float dist2 = (point_x - motion->tracker.sense.pos_x) * (point_x - motion->tracker.sense.pos_x) + (point_y - motion->tracker.sense.pos_y) * (point_y - motion->tracker.sense.pos_y);
			if(dist2 < 0.0025f)  //dist < 0.05m = 5cm
				state ++;

			Motion_Run_Tracker(&motion->tracker);
		}
		else if(state == 1)
		{
			Motion_Start_2D_Angle(&motion->tracker, 0.0f, -1.0f, speed);
			Motion_Run_Tracker(&motion->tracker);
			motion->tracker.line_vel = 0;
			
			if( fabsf(motion->tracker.sense.dir_y + 1.0f) <= 0.005f )
			{
				point_x = side_length;
				point_y = -side_length;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,0.0f,-1.0f,speed);	
				state ++;
			}
		}
		else if(state == 3)
		{
			Motion_Start_2D_Angle(&motion->tracker, -1.0f, 0.0f, speed);
			Motion_Run_Tracker(&motion->tracker);
			motion->tracker.line_vel = 0;
			
			if( fabsf(motion->tracker.sense.dir_x + 1.0f) <= 0.005f )
			{
				point_x = 0.0f;
				point_y = -side_length;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,-1.0f,0.0f,speed);
				state ++;
			}
		}
		else if(state == 5)
		{
			Motion_Start_2D_Angle(&motion->tracker, 0.0f, 1.0f, speed);
			Motion_Run_Tracker(&motion->tracker);
			motion->tracker.line_vel = 0;
			
			if( fabsf(motion->tracker.sense.dir_y - 1.0f) <= 0.01f )
			{
				point_x = 0.0f;
				point_y = 0.0f;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,0.0f,1.0f,speed);
				state ++;
			}
		}
		else if(state == 7)
		{
			otion_Start_2D_Angle(&motion->tracker, 1.0f, 0.0f, speed);
			Motion_Run_Tracker(&motion->tracker);
			motion->tracker.line_vel = 0;
			
			if( fabsf(motion->tracker.sense.dir_x - 1.0f) <= 0.01f )
			{
				point_x = side_length;
				point_y = 0.0f;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,1.0f,0.0f,speed);
				state = 0;
			}
		}

		/*
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
			//motion->tracker.line_vel = PI_Run2( &(motion->path_imu.direction_pi), - motion->tracker.line_vel);
			motion->tracker.angular_vel = speed;
			//motion->tracker.angular_vel = PI_Run2( &(motion->path_imu.direction_pi), speed - motion->tracker.angular_vel);
			
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
		*/
		Motion_Process_Motor_Speed(motion);
		update_motor_control();
	}
}

/*******************************************
 描述：测试用程序 角度闭环 走圆 逆时针
 变量：speed 速度
       side_length 边长单位为时间 1秒钟对应边长50
 注意：
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

/*在分配堆栈空间时，必须要对其*/
ALIGN(RT_ALIGN_SIZE)
char thread_motion_stack[1024];
struct rt_thread thread_motion;

void mower_motion_thread(void* parameter)
{
  rt_uint32_t recved;
	T_motion motion;
	
	//等待IMU GPS初始化 后期应删除
	rt_thread_delay(2000);
	
	//初始化motion变量 并使能
	Motion_Init(&motion,1);
	
	//测试用程序
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
		//等待event 20ms一次
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		
		//传感器数据更新
		Motion_Get_Position_2D(&motion.tracker.sense); //get the position and orientation of the vehicle
		Motion_Get_Sensor(&motion.tracker.sense);      // get other sensor data(sonar, bumper, etc.)



		//Motion_Update(&motion.tracker.path_imu); 		//zigzag state: update nextpoint



		//障碍物减速更新 （后期加入碰撞等传感器）
		//Motion_Process_Obstacle(&motion);
		
		//运行控制器（根据情况选择）
		
		//运行磁导线控制器
		//Motion_Run_Mag_Line(&motion.tracker);
		//运行位姿控制器
		//Motion_Run_Tracker(&motion.tracker);
		//运行过程控制器（如工字型路径等）
		Motion_Run(&motion);
		
		//电机加速度控制 电机驱动接口调用
		Motion_Process_Motor_Speed(&motion);

		//Debug
		//rt_kprintf("angle = %d                x = %d                 y = %d \n\r",(int)(eul_rad[0]*10*57.3), (int)(motion.tracker.sense.pos_x*100), (int)(motion.tracker.sense.pos_y*100));
		//rt_kprintf("left = %d, right =%d \n\r",(int)(motion.tracker.line_vel),(int)(motion.tracker.angular_vel));
		//rt_kprintf("left = %d, right = %d \r\n\r",leftsensor_data,rightsensor_data);
	#if 0//def CUSTOMER_SERIAL_DEBUG
		//rt_kprintf("\r\n...>>>motion_thread");
	#endif
		
		//电机驱动速度更新指令
		update_motor_control();
	}
}








