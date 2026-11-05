#include <stdbool.h>
#include "stdio.h"
#include "stdint.h"
#ifndef CONTROL_TOOLS_H_
#define CONTROL_TOOLS_H_
//---------------------------------位置式&增量式PI-------------------------------//
typedef struct PI_Controller
{
    float Kp;
    float Ki;
    float Error;
    float Error_last;
    //位置式PI
    float Pos;
    float integral;
    float integral_min;
    float integral_max;
	//增量式PI
	float Pi_Delta;
    float Delta_Max;
    float Delta_Min;
} PI_Controller;
//---------------------------------PR谐振控制器-------------------------------//
typedef struct {
	float output;
    float u_prev1, u_prev2;
    float y_prev1, y_prev2;
    float b0, b1, b2, a1, a2;
    float kp,kr,T;
    float Wc; /* 谐振带宽参数，rad/s */
    float W0; /* 谐振中心角频率，rad/s */
    float out_limit;
} PR_Controller;
//---------------------增量式PI------------------------//
/* kp、ki 为离散增益；min、max 为增量限幅。 */
void PI_Delta_Init(PI_Controller *Pi, float kp, float ki, float min, float max);
void PI_Delta_Cal(PI_Controller *Pi, float Pi_in);
//---------------------位置式PI------------------------//
/* kp、ki 为离散增益；min、max 为输出及积分限幅。 */
void PI_Pos_Init(PI_Controller *Pi, float kp, float ki, float min, float max);
void PI_Pos_Cal(PI_Controller *Pi, float error);
//---------------------PR函数------------------------//
/* T 为调用周期(s)，Wc 为带宽参数(rad/s)，W0=2*pi*f0 为中心角频率(rad/s)。 */
void PR_Init(PR_Controller* ctrl, float T, float Kp, float Kr,
             float Wc, float W0, float out_limit);
void PR_Cal(PR_Controller* ctrl, float input, float gain) ;
//---------------------Clark变换函数------------------------//
void Clarke_F(float Ua, float Ub, float Uc, float *Ualpha, float *Ubeta);
//void inv_Clarke_F(float Udc, float Ualpha, float Ubeta, float *w1, float *w2, float *w3);

#endif /* CONTROL_TOOLS_H_ */
