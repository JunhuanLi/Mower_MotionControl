/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name 	: path_tracking.h
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

#ifndef __MOTION_IMU_H__
#define __MOTION_IMU_H__


/* includes *******************************************************************/
#include "stm32f4xx.h"
#include "typedef.h"
#include "pi.h"
#include "motion_math.h"
#include "motion_types.h"

/* macros *********************************************************************/

/* static variables ***********************************************************/

//void Tracking_Init(T_motion_tracker* obj,uint8_t en, T_motion_tracking_type tracking);

// 位姿控制器调用函数
void Motion_Run_Tracker(T_motion_tracker* obj);
void Motion_Run_2D_Angle(T_motion_tracker* obj);
void Motion_Run_2D_Line(T_motion_tracker* obj);
void Motion_Run_3D_Angle(T_motion_tracker* obj);
void Motion_Run_3D_Line(T_motion_tracker* obj);

//将二维方向映射到三维 解决地面倾斜的问题
void Motion_Targert_Convert_3D(T_motion_tracker* obj);
//根据向量值判断是第几象限
extern __inline uint8_t Motion_find_sector_2D(float x, float y){return (x<0)?((y<0)?3:4):((y>0)?1:2);}

//设定位姿控制器参数

// advanc 前瞻点距离 越大运行越平顺 但误差越大 越小误差越小 但不平顺
void Motion_Set_Path_Param(T_motion_tracker* obj,float advanc);

// 姿态控制器的pid参数
void Motion_Set_Angle_Param(T_motion_tracker* obj,float kp, float ki, float il);

// 位姿控制方式 调用此函数后再运行tracker
// 角度控制器 提供角度向量
void Motion_Start_2D_Angle(T_motion_tracker* obj,float dir_x,float dir_y,float vel);
// 沿线控制器 提供角度向量和终点位置
void Motion_Start_2D_Line(T_motion_tracker* obj,float point_x, float point_y, float dir_x, float dir_y, float vel);
// 圆弧控制器 提供圆心和终点位置
void Motion_Start_2D_Arc(T_motion_tracker* obj,float point_x, float point_y, float center_x, float center_y,float vel);
// 追点控制器 提供终点坐标
void Motion_Start_2D_Point(T_motion_tracker* obj,float point_x, float point_y,float vel);

// 方向向量转换为角度 逆时针为正
float Motion_Vect2Angle(float x, float y);

#endif
