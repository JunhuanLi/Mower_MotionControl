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
#include "pi.h"
/* macros *********************************************************************/

/* static variables ***********************************************************/

/* funcitons ******************************************************************/
void PI_Init(T_pi* pi,float kp, float ki,float il)
{
	pi->kp = kp;
	pi->ki = ki;
	pi->il = il;
	pi->fb = 0.0f;
	pi->ref = 0.0f;
	pi->err = 0.0f;
	pi->out = 0.0f;
	pi->integral = 0.0f;
}

extern __inline void PI_Reset(T_pi* pi,float kp,float ki,float il)
{
	PI_Init(pi,kp,ki,il);
}

extern __inline float PI_Run(T_pi* pi,float fb,float ref)
{
	pi->fb = fb;
	pi->ref = ref;
	pi->err = pi->ref - pi->fb;
	pi->integral += pi->ki * pi->err;
	if(fabsf(pi->integral) > fabsf(pi->il))
	{
		pi->integral = pi->il;
	}
	pi->out = pi->kp * pi->err + pi->integral;
	return pi->out;
}

extern __inline float PI_Run2(T_pi* pi,float err)
{
	pi->integral += pi->ki * pi->err;
	if(fabsf(pi->integral) > fabsf(pi->il))
	{
		pi->integral = pi->il;
	}
	pi->out = pi->kp * pi->err + pi->integral;
	return pi->out;