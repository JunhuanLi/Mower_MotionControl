/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name 	: pi.h
  Author		: George 	
  Version		: V1.0.0	  
  Date			: 2017/07/21
  Description	: pi 
  
  History:		  
				  
	1. Date 		:
	   Author		:
	   Modification :
	2. ...
	
*******************************************************************************/

/* includes *******************************************************************/
#include "motion_control.h"
#include "motor_control.h"
#include "motion_math.h"

/* macros *********************************************************************/

/* static variables ***********************************************************/


// 求单位向量
static __inline void Motion_Norm_2D(float* x, float* y)
{
	float length = sqrtf((*x)*(*x) + (*y)*(*y));
	if(length != 0)
	{
		*x = *x / length;
		*y = *y / length;
	}
}

// 工字型路径状态机
static void Motion_Run_Zigzag(T_motion* motion)
{
	if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_IDLE)
	{
		return;
	}
	//直线部分
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_LINE)
	{
		//运行位姿控制器
		Motion_Run_Tracker(&motion->tracker);
		//判断是否跨线
		if(motion->tracker.sense.side_l == MOTION_MAG_LINE_OUTSIDE 
			&& motion->tracker.sense.side_r == MOTION_MAG_LINE_OUTSIDE)
		{
			motion->zigzag.state = T_MOTION_ZIGZAG_STATE_TURN;
		}
	}
	//掉头部分
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_TURN)
	{
		float dot_product = (motion->tracker.sense.dir_x*motion->zigzag.heading_x)+(motion->tracker.sense.dir_y*motion->zigzag.heading_y);
		float k = (1- motion->zigzag.blade_bodywidth_ratio * motion->zigzag.blade_overlaping_ratio) / 2;
		
		//计算线速度和角速度
		if(motion->zigzag.turn_dir == T_MOTION_ZIGZAG_TURN_COUNTERCLOCKWISE)
		{
			motion->tracker.line_vel = motion->zigzag.target_vel;
			motion->tracker.angular_vel = motion->zigzag.target_vel/(1-k);
			//motion->tracker.line_vel = 0;
		}
		else
		{
			motion->tracker.line_vel = motion->zigzag.target_vel;
			motion->tracker.angular_vel = -motion->zigzag.target_vel/(1-k);
			//motion->tracker.line_vel = 0;
		}
		
		//利用点乘判断是否完成掉头 之后计算下一阶段参数

		/*if(motion->zigzag.f_r == T_MOTION_ZIGZAG_GO_FOWARD)
		{
			if(dot_product < -0.96f) //~170degree
			{
				//motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
				motion->tracker.angular_vel = 0;
				motion->tracker.line_vel = 0;
			}
			//Tracking_Start_2D_Angle(&motion->tracker,-motion->zigzag.heading_x,-motion->zigzag.heading_y,motion->zigzag.target_vel);
		}
		else
		{
			if(dot_product > 0.96f)
			{
				//motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
				motion->tracker.angular_vel = 0;
				motion->tracker.line_vel = 0;
			}
			//Tracking_Start_2D_Angle(&motion->tracker,motion->zigzag.heading_x,motion->zigzag.heading_y,motion->zigzag.target_vel);
		}*/

		if( fabs(dot_product) > 0.96f)  //~170degree
			{



				//turn finished, need to toggle the next turn direction
				/*
				motion->zigzag.turn_dir += 1;
				if(motion->zigzag.turn_dir>2)
					motion->zigzag.turn_dir=1;
				*/
				

				motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
				motion->tracker.angular_vel = 0;
				motion->tracker.line_vel = 0;
			}
			//Tracking_Start_2D_Angle(&motion->tracker,motion->zigzag.heading_x,motion->zigzag.heading_y,motion->zigzag.target_vel);
		
		
	}
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_EXCEPTION)
	{
		
	}
}
/* funcitons ******************************************************************/

void Motion_Init(T_motion* motion,uint8_t en)
{
	set_motor_control_type(MOTOR_CONTROL_TYPE_SPEED);
	
	motion->motion_state = 											MOTION_STATE_IDLE;
	motion->exception = 											MOTION_EXCEPTION_NONE;
	motion->enable = 												en;
	
	motion->tracker.tracking = 										MOTION_TRACKING_2D_ANGLE;
	motion->tracker.acc = 									MOTION_ACC;
	motion->tracker.vel_l = 										0;
	motion->tracker.vel_r	= 										0;
	
	//obj->error = TRACKING_ERROR_NONE;
	//obj->sensor.mag_polarity = 1;
	Motion_Set_Path_Param(&motion->tracker,0.4);
	Motion_Set_Angle_Param(&motion->tracker,2000,0.1f,500);
	Motion_Set_Mag_Tracking_Param(&motion->tracker,0,0,0);
	Motion_Set_Mag_Gotoline_Param(&motion->tracker,0,0,0);
	Motion_Start_2D_Angle(&motion->tracker,1,0,0);
	
	Motion_Zigzag_Init(motion,0.8,0.6);
}

void Motion_Run(T_motion* motion)
{
	if(motion->motion_state == MOTION_STATE_ZIGZAG)
	{
		Motion_Run_Zigzag(motion);
	}


}

/*
void Motion_Process_Obstacle(T_motion* motion)
{
	//Run Obsticle
	float vel_sonar = motion->tracker.command_vel;
	float vel_mag_line = motion->tracker.command_vel;
	uint16_t sonar = motion->tracker.sense.sonar_l > motion->tracker.sense.sonar_r ? motion->tracker.sense.sonar_l : motion->tracker.sense.sonar_r;
	
	if(sonar > DECELERATION_SONAR_MIN)
	{
		vel_sonar = (float)(1 - (sonar - DECELERATION_SONAR_MIN))/(float)(DECELERATION_SONAR_MAX - DECELERATION_SONAR_MIN) * (float)(1 -DECELERATION_MIN_SPEED)
		           + (float)DECELERATION_MIN_SPEED;
		vel_sonar = vel_sonar * motion->tracker.command_vel;
	}
	
	motion->tracker.target_vel = vel_sonar < vel_mag_line ? vel_sonar : vel_mag_line;
}
*/

void Motion_Process_Motor_Speed(T_motion* motion)
{
	//obj->line_vel = 200;
	//obj->angle_vel = 200;
	int32_t vl = (int32_t)(motion->tracker.line_vel+motion->tracker.angular_vel);
	int32_t vr = (int32_t)(motion->tracker.line_vel-motion->tracker.angular_vel);
	
	//Run Accellation
	if(motion->tracker.vel_l < vl)
	{
		motion->tracker.vel_l += motion->tracker.acc;
		if(motion->tracker.vel_l > vl)
		{
			motion->tracker.vel_l = vl;
		}
	}
	else if(motion->tracker.vel_l > vl)
	{
		motion->tracker.vel_l -= motion->tracker.acc;
		if(motion->tracker.vel_l < vl)
		{
			motion->tracker.vel_l = vl;
		}
	}
	
	if(motion->tracker.vel_r < vr)
	{
		motion->tracker.vel_r += motion->tracker.acc;
		if(motion->tracker.vel_r > vr)
		{
			motion->tracker.vel_r = vr;
		}
	}
	else if(motion->tracker.vel_r > vr)
	{
		motion->tracker.vel_r -= motion->tracker.acc;
		if(motion->tracker.vel_r < vr)
		{
			motion->tracker.vel_r = vr;
		}
	}
	
	//Set Speed
	set_motor_control_speed_s32(motion->tracker.vel_l,motion->tracker.vel_r);
	//set_motor_control_speed_s32(vl,vr);
}


void Motion_Zigzag_Init(T_motion* motion,float blade_bodywidth_ratio,float blade_overlaping_ratio)
{
	motion->zigzag.blade_bodywidth_ratio =                  	blade_bodywidth_ratio;
	motion->zigzag.blade_overlaping_ratio =                     blade_overlaping_ratio;
	motion->zigzag.state = 									    T_MOTION_ZIGZAG_STATE_IDLE;
	motion->zigzag.f_r = 										T_MOTION_ZIGZAG_GO_FOWARD;
	motion->zigzag.state = 										T_MOTION_ZIGZAG_STATE_LINE;
}

void Motion_Zigzag_Start(T_motion* motion,float speed,float heading_x,float heading_y,T_motion_zigzag_turn_dir_type turn_dir)
{
	float x = 													heading_x;
	float y =                             						heading_y;

	Motion_Norm_2D(&x,&y);
	
	motion->zigzag.heading_x = 									x;
	motion->zigzag.heading_y = 									y;
	motion->zigzag.turn_dir = 									turn_dir;
	motion->zigzag.f_r = 										T_MOTION_ZIGZAG_GO_FOWARD;
	motion->zigzag.target_vel = 								speed;
	motion->motion_state = 										MOTION_STATE_ZIGZAG;
	
	Motion_Start_2D_Angle(&motion->tracker, motion->zigzag.heading_x, motion->zigzag.heading_y, motion->zigzag.target_vel);
}

