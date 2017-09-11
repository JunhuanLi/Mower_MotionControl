/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name 	: pi.h
  Author		: George 	
  Version		: V1.0.0	  
  Date			: 2017/07/21
  Description	: motion_control 
  
  History:		  
				  
	1. Date 		:
	   Author		:
	   Modification :
	2. ...
	
*******************************************************************************/

#ifndef __MOTION_CONTROL_H__
#define __MOTION_CONTROL_H__

/* includes *******************************************************************/

#include "stm32f4xx.h"
#include "typedef.h"
#include "math.h"

#include "motion_imu.h"
#include "motion_types.h"
#include "motion_sense.h"
#include "motion_math.h"
#include "motion_mag_line.h"


/* macros *********************************************************************/

// 异常状态枚举量
typedef enum
{
	MOTION_EXCEPTION_NONE = 0,
	MOTION_EXCEPTION_OBSTACLE,
	MOTION_EXCEPTION_SLIP,
	MOTION_EXCEPTION_TRAPPED
}T_motion_exception_type;

// 过程控制器状态机枚举量
typedef enum
{
	MOTION_STATE_IDLE = 0,
	MOTION_STATE_P2P,
	MOTION_STATE_ZIGZAG,
	MOTION_STATE_MAGLINE,
	MOTION_STATE_RANDOM,
	MOTION_STATE_INVOLUTE,
	MOTION_STATE_FIND_MAGLINE,
	MOTION_STATE_ALIGN_BASE
}T_motion_state_type;

// 工字型控制状态机枚举量
typedef enum 
{
	T_MOTION_ZIGZAG_STATE_IDLE = 0,
	T_MOTION_ZIGZAG_STATE_LINE,
	T_MOTION_ZIGZAG_STATE_TURN,
	T_MOTION_ZIGZAG_STATE_EXCEPTION
}T_motion_zigzag_state_type;

// 工字型控制碰线后转弯方向枚举量
typedef enum
{
	T_MOTION_ZIGZAG_TURN_NONE = 0,
	T_MOTION_ZIGZAG_TURN_COUNTERCLOCKWISE,
	T_MOTION_ZIGZAG_TURN_CLOCKWISE
}T_motion_zigzag_turn_dir_type;

// 工字型路径前进方向枚举量 最开始为foward， 掉头后为reverse,再掉头为 foward
typedef enum
{
	T_MOTION_ZIGZAG_GO_FOWARD = 0,
	T_MOTION_ZIGZAG_GO_REVERSE
}T_motion_zigzag_foward_reverse_type;

// 工字型路径变量
typedef struct
{
	T_motion_zigzag_turn_dir_type											turn_dir;
	T_motion_zigzag_foward_reverse_type										f_r;				//foward or reverse
	T_motion_zigzag_state_type 												state;
	// 刀盘宽度 除以 轮距
	float																	blade_bodywidth_ratio; 
	// 路径不重复覆盖的概率 每次掉头的时候有一部分草要重新割一遍 这个数越小 重新割的越多
	float																	blade_overlaping_ratio; 
	// 直线阶段的目标速度
	float																	target_vel;
	// foward阶段的车身朝向
	float																	heading_x;
	float																	heading_y;

}T_motion_zigzag;

// 过程控制器的结构体
typedef struct
{
	uint8_t																	enable;
	T_motion_tracker														tracker;
	T_motion_state_type														motion_state;
	T_motion_exception_type													exception;
	T_motion_zigzag															zigzag;
}T_motion;

/* static variables ***********************************************************/
/* funcitons ******************************************************************/
void Motion_Init(T_motion* motion,uint8_t en);
void Motion_Run(T_motion* motion);
//障碍物减速更新 （后期加入碰撞等传感器）
void Motion_Process_Obstacle(T_motion* motion);
//电机加速度控制 电机驱动接口调用
void Motion_Process_Motor_Speed(T_motion* motion);

// 工字型路径结构体初始化
void Motion_Zigzag_Init(T_motion* motion,float blade_bodywidth_ratio,float blade_overlaping_ratio); 
// 工字型路径开始函数
void Motion_Zigzag_Start(T_motion* motion,float speed,float heading_x,float heading_y,T_motion_zigzag_turn_dir_type turn_dir);

#endif /* __MOTION_CONTROL_H__ */
