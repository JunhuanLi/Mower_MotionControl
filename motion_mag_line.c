/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name 	: path_tracking.c
  Author		: George 	
  Version		: V1.0.0	  
  Date			: 2017/07/21
  Description	: path_tracking 
  
  History:		  
				  
	1. Date 		:
	   Author		:
	   Modification :
	2. ...
	
*******************************************************************************/

/* includes *******************************************************************/
#include "motion_mag_line.h"
#include "PI.h"
#include "math.h"
#include "global.h"
#include "usart_driver.h"

/* macros *********************************************************************/

static float Motion_Mag_Line(T_motion_tracker* obj);

// 判断车头朝向
static T_motion_mag_line_orientation_type Motion_Mag_Line_Get_Orientation(T_motion_tracker* obj)
{		
	// 如果有一个传感器没有读数 则离线太远报错
	if(obj->sense.side_l == MOTION_MAG_LINE_MISSING 
		|| obj->sense.side_r == MOTION_MAG_LINE_MISSING)
	{
		return MOTION_MAG_LINE_ORI_MISSING;
	}
	
	// 如果两个传感器在线的同一侧
	if(obj->sense.side_l == obj->sense.side_r)
	{
		uint32_t val_l = Motion_Abs(obj->sense.value_l);
		uint32_t val_r = Motion_Abs(obj->sense.value_r);
		// 如果车在线内
		if(obj->sense.side_l == MOTION_MAG_LINE_INSIDE)
		{
			//如果车头与导线垂直
			if(val_l< val_r * (1.0f + MAG_LINE_FACING_MARGIN)
			&& val_r < val_l * (1.0f + MAG_LINE_FACING_MARGIN))
			{
				return MOTION_MAG_LINE_ORI_IN_NORMAL;
			}
			//如果左边传感器读数较大
			else if(val_l > val_r)
			{
				return MOTION_MAG_LINE_ORI_IN_LHRL;
			}
			//如果右边传感器读数较大
			else
			{
				return MOTION_MAG_LINE_ORI_IN_LLRH;
			}
		}
		//如果车在线外
		else
		{
			//如果车头与导线垂直
			if(val_l< val_r * (1.0f + MAG_LINE_FACING_MARGIN)
			&& val_r < val_l * (1.0f + MAG_LINE_FACING_MARGIN))
			{
				return MOTION_MAG_LINE_ORI_OUT_NORMAL;
			}
			//如果左边传感器读数较大
			else if(val_l > val_r)
			{
				return MOTION_MAG_LINE_ORI_OUT_LHRL;
			}
			//如果右边传感器读数较大
			else
			{
				return MOTION_MAG_LINE_ORI_OUT_LLRH;
			}
		}
	}
	//如果两个传感器在线的不同测
	else
	{
		//如果顺着电流方向循线
		if(obj->path_mag_line.dir == MOTION_MAG_LINE_DIRECT)
		{
			//如果左边传感器在线内 则车身与前进方向对齐
			if(obj->sense.side_l == MOTION_MAG_LINE_INSIDE)
			{
				return MOTION_MAG_LINE_ORI_ALIGNED;
			}
			//否则车身相反
			else
			{
				return MOTION_MAG_LINE_ORI_REVERSE;
			}
		}
		//如果逆着电流方向循线
		else
		{
			//如果左边传感器在线外 则车身相反
			if(obj->sense.side_l == MOTION_MAG_LINE_INSIDE)
			{
				return MOTION_MAG_LINE_ORI_REVERSE;
			}
			//否则车身方向正确
			else
			{
				return MOTION_MAG_LINE_ORI_ALIGNED;
			}
		}
	}
	return MOTION_MAG_LINE_ORI_MISSING;
}

	T_motion_mag_line_orientation_type 			orientation = 0;

static float Motion_Mag_Line(T_motion_tracker* obj)
{
	float 																	left_distance, right_distance;
	static 	T_motion_turn_type 							rot_dir;
	static	T_motion_mag_line_side_type			prev_side;				
	float 																	angular_vel = 0;

	orientation = Motion_Mag_Line_Get_Orientation(obj);	
		
	//如果找不到线报错停车
	if(orientation == MOTION_MAG_LINE_ORI_MISSING)
	{
		obj->error = MOTION_ERROR_NO_MAG_LINE;
		obj->angular_vel = 0;
		obj->line_vel = 0;
		return 0;
	}
	
	if(obj->path_mag_line.state== MOTION_MAG_LINE_STATE_IDLE)
	{
		obj->angular_vel = 0;
		obj->line_vel = 0;
		return 0;
	}
	// 初始状态
	else if(obj->path_mag_line.state == MOTION_MAG_LINE_STATE_START)
	{
		//如果车在线内 则往线的方向走
		if(orientation == MOTION_MAG_LINE_ORI_IN_NORMAL || orientation == MOTION_MAG_LINE_ORI_IN_LHRL || orientation == MOTION_MAG_LINE_ORI_IN_LLRH)
		{
			prev_side = obj->sense.side_l;
			obj->path_mag_line.state = MOTION_MAG_LINE_STATE_GOTOLINE; //Maybe in the wrong direction which should be checked later
		}
		//如果车在线外并且与线不垂直则调整车头方向与线垂直
		else if(orientation == MOTION_MAG_LINE_ORI_OUT_LHRL 
					|| orientation == MOTION_MAG_LINE_ORI_OUT_LHRL)
		{
			obj->path_mag_line.state = MOTION_MAG_LINE_STATE_FINDDIR;
		}
		//如果车在线外且与线垂直则往线的方向走（这里没有处理车头方向相反的情况）
		else if(orientation == MOTION_MAG_LINE_ORI_OUT_NORMAL)
		{
			// may need to turn around
			prev_side = obj->sense.side_l;
			obj->path_mag_line.state = MOTION_MAG_LINE_STATE_GOTOLINE;
		}
		//如果已经在线上且车头方向正确则直接开始循线
		else if(orientation == MOTION_MAG_LINE_ORI_ALIGNED)
		{
			obj->path_mag_line.state = MOTION_MAG_LINE_STATE_TRACELINE;
		}
		//如果在线上但车头方向不对则原地掉头
		else if(orientation == MOTION_MAG_LINE_ORI_REVERSE)
		{
			obj->path_mag_line.state = MOTION_MAG_LINE_STATE_ALIGNING;//May Search for ever, Can be merged with FINDDIR state
			prev_side = MOTION_MAG_LINE_INSIDE;
		}
		obj->angular_vel = 0;
		obj->line_vel = 0;
	}
	else if(obj->path_mag_line.state == MOTION_MAG_LINE_STATE_FINDDIR)
	{
		//左边传感器数大就逆时针转
		if(orientation == MOTION_MAG_LINE_ORI_OUT_LHRL)
		{
			rot_dir = MOTION_TURN_COUNTERCLOCKWISE;
			obj->angular_vel = obj->target_vel;
			obj->line_vel = 0;
		}
		//右边传感器数大就顺时针转
		else if(orientation == MOTION_MAG_LINE_ORI_OUT_LLRH)
		{
			rot_dir = MOTION_TURN_CLOCKWISE;
			obj->angular_vel = -obj->target_vel;
			obj->line_vel = 0;
		}
		//否则就跳回初始状态
		else
		{
			obj->path_mag_line.state = MOTION_MAG_LINE_STATE_START;
			obj->angular_vel = 0;
			obj->line_vel = 0;
		}
	}
	else if(obj->path_mag_line.state == MOTION_MAG_LINE_STATE_GOTOLINE)
	{
		//如果检测到跨线则判断车头方向 确定是开始循线 还是先调整车头方向
		if(prev_side != obj->sense.side_l || prev_side != obj->sense.side_r)
		{
			//obj->path_mag_line.state = MOTION_MAG_LINE_STATE_IDLE;
			if(orientation == MOTION_MAG_LINE_ORI_ALIGNED)
			{
				obj->path_mag_line.state = MOTION_MAG_LINE_STATE_TRACELINE;
			}
			else
			{
				obj->path_mag_line.state = MOTION_MAG_LINE_STATE_ALIGNING;
			}
			obj->angular_vel = 0;
			obj->line_vel = 0;
		}
		//否则根据传感器数值调整车头方向
		else
		{
			if(obj->sense.value_l > obj->sense.value_r)
			{
				obj->angular_vel = 0.25f * obj->target_vel;
			}
			else if(obj->sense.value_l < obj->sense.value_r)
			{
				obj->angular_vel = -0.25f * obj->target_vel;
			}
			else
			{
				obj->angular_vel = 0.0f;
			}
			obj->line_vel = obj->target_vel;
		}	
	}
	else if(obj->path_mag_line.state == MOTION_MAG_LINE_STATE_ALIGNING)
	{	
		//调整完成开始循线
		if(orientation == MOTION_MAG_LINE_ORI_ALIGNED)
		{
			obj->path_mag_line.state = MOTION_MAG_LINE_STATE_TRACELINE;
			obj->angular_vel = 0;
			obj->line_vel = 0;
		}
		//根据循线方向做不同判断
		else if(obj->path_mag_line.dir == MOTION_MAG_LINE_DIRECT)
		{
			//左传感器在线外就逆时针转 否则顺时针转
			if(obj->sense.side_l == MOTION_MAG_LINE_OUTSIDE)
			{
				rot_dir = MOTION_TURN_COUNTERCLOCKWISE;
				obj->angular_vel = obj->target_vel;
				obj->line_vel = 0.25f * obj->target_vel;
			}
			else
			{
				rot_dir = MOTION_TURN_CLOCKWISE;
				obj->angular_vel = -obj->target_vel;
				obj->line_vel = 0.25f * obj->target_vel;
			}
		}
		else if(obj->path_mag_line.dir == MOTION_MAG_LINE_REVERSE)
		{
			if(obj->sense.side_l == MOTION_MAG_LINE_INSIDE)
			{
				rot_dir = MOTION_TURN_COUNTERCLOCKWISE;
				obj->angular_vel = obj->target_vel;
				obj->line_vel = 0.25f * obj->target_vel;
			}
			else
			{
				rot_dir = MOTION_TURN_CLOCKWISE;
				obj->angular_vel = -obj->target_vel;
				obj->line_vel = 0.25f * obj->target_vel;
			}
		}
		//跳到这里就是有问题了
		else
		{
			while(1);
			obj->path_mag_line.state = MOTION_MAG_LINE_STATE_IDLE;
			obj->angular_vel = 0;
			obj->line_vel = 0;
		}
	}
	else if(obj->path_mag_line.state == MOTION_MAG_LINE_STATE_TRACELINE)
	{
		float												err;
		float												pi_out;
		
		//如果两个传感器都在线外或者都在线内则大幅度转向
		if(obj->sense.side_l == obj->sense.side_r)
		{
			if(obj->path_mag_line.dir == MOTION_MAG_LINE_DIRECT)
			{
				if(obj->sense.side_l == MOTION_MAG_LINE_INSIDE)
				{
					obj->angular_vel = -0.75*obj->target_vel;
					obj->line_vel = obj->target_vel;
				}
				else if(obj->sense.side_l == MOTION_MAG_LINE_OUTSIDE)
				{
					obj->angular_vel = 0.75*obj->target_vel;
					obj->line_vel = obj->target_vel;
				}
			}
			else
			{
				if(obj->sense.side_l == MOTION_MAG_LINE_INSIDE)
				{
					obj->angular_vel = 0.75*obj->target_vel;
					obj->line_vel = obj->target_vel;					
				}
				else if(obj->sense.side_l == MOTION_MAG_LINE_OUTSIDE)
				{
					obj->angular_vel = -0.75*obj->target_vel;
					obj->line_vel = obj->target_vel;
				}
			}
		}
		//否则小幅度调整或者不调整
		else
		{
			obj->angular_vel = 0;
			obj->line_vel = obj->target_vel;		
			if(obj->sense.side_l != obj->sense.side_r)
			{
				if(obj->sense.value_l < 15000)
				{
					obj->angular_vel = 0.25f*obj->target_vel;
					obj->line_vel = obj->target_vel;		
				}
				else if(obj->sense.value_r < 15000)
				{
					obj->angular_vel = -0.25f*obj->target_vel;
					obj->line_vel = obj->target_vel;	
				}
				else if(obj->sense.value_l > 26000 && obj->sense.value_r > 26000)
				{
					if(obj->sense.value_l < obj->sense.value_r)
					{
						obj->angular_vel = -0.1f*obj->target_vel;
						obj->line_vel = obj->target_vel;	
					}
					else
					{
						obj->angular_vel = -0.1f*obj->target_vel;
						obj->line_vel = obj->target_vel;	
					}
				}
				else
				{
					obj->angular_vel = 0.0f;
					obj->line_vel = obj->target_vel;
				}
			}
		}

		/*
		else
		{
			left_distance = sqrtf(MAG_LINE_MAX - obj->sense.value_l);
			right_distance = sqrtf(MAG_LINE_MAX - obj->sense.value_r);
			if(left_distance > right_distance)
			{
				rot_dir = MOTION_TURN_CLOCKWISE;
				err = left_distance - right_distance;
			}
			else if(left_distance < right_distance)
			{
				rot_dir = MOTION_TURN_COUNTERCLOCKWISE;
				err = right_distance - left_distance;
			}
			else
			{
				rot_dir = MOTION_TURN_NONE;
				err = 0;
			}
			
			if(err != 0)
				pi_out = PI_Run(&(obj->path_mag_line.mag_tracking_pi),0,err);
			
			if(rot_dir == MOTION_TURN_CLOCKWISE)
			{
				//angular_vel = -pi_out;
				angular_vel = -obj->target_vel * 0.2f;
			}
			else if(rot_dir == MOTION_TURN_COUNTERCLOCKWISE)
			{
				//angular_vel = pi_out;
				angular_vel = obj->target_vel * 0.2f;
			}
			else
			{
				angular_vel = 0;
			}
			
			if(angular_vel > obj->target_vel)
			{
				angular_vel = obj->target_vel;
			}
			else if(angular_vel < -obj->target_vel)
			{
				angular_vel = -obj->target_vel;
			}
			
			obj->angular_vel = angular_vel;
			obj->line_vel = obj->target_vel;
		}
		*/
	}
	else if(obj->path_mag_line.state == MOTION_MAG_LINE_STATE_DONE)		
	{
		obj->path_mag_line.state = MOTION_MAG_LINE_STATE_IDLE;
		obj->angular_vel = 0;
		obj->line_vel = 0;
	}
	return angular_vel;
}

void Motion_Run_Mag_Line(T_motion_tracker* obj)
{
	Motion_Mag_Line(obj);  //angular and line vel will be auto assigned
}

void Motion_Set_Mag_Tracking_Param(T_motion_tracker* obj,float kp, float ki, float il)
{
	obj->path_mag_line.mag_tracking_pi.kp = kp;
	obj->path_mag_line.mag_tracking_pi.ki = ki;
	obj->path_mag_line.mag_tracking_pi.il = il;
}

void Motion_Set_Mag_Gotoline_Param(T_motion_tracker* obj,float kp, float ki, float il)
{
	obj->path_mag_line.mag_gotoline_pi.kp = kp;
	obj->path_mag_line.mag_gotoline_pi.ki = ki;
	obj->path_mag_line.mag_gotoline_pi.il = il;
}

void Motion_Start_Mag_Line(T_motion_tracker* obj,float vel,T_motion_mag_line_dir_type dir)
{
	obj->tracking										= MOTION_TRACKING_MAG_LINE;
	obj->path_mag_line.state	 			= MOTION_MAG_LINE_STATE_START;
	obj->command_vel								= vel;
	obj->target_vel									= vel;
	obj->path_mag_line.dir	 				= dir;
	obj->path_mag_line.mag_gotoline_pi.integral		= 0;
	obj->path_mag_line.mag_tracking_pi.integral		= 0;
}


