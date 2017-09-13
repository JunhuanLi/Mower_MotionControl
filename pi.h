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

#ifndef __PI_H__
#define __PI_H__

/* includes *******************************************************************/

#include "stm32f4xx.h"
#include "typedef.h"
#include "math.h"

/* macros *********************************************************************/

/* static variables ***********************************************************/
typedef struct
{
	float kp;
	float ki;
	float il;
	float fb;
	float ref;
	float err;
	float out;
	float integral;
}T_pi;

/* funcitons ******************************************************************/
void PI_Init(T_pi* pi,float kp, float ki,float il);

extern __inline void PI_Reset(T_pi* pi,float kp,float ki,float il);

extern __inline float PI_Run(T_pi* pi,float fb,float ref);

extern __inline float PI_Run2(T_pi* pi,float err);

#endif /* __PI_H__ */
