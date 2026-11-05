#include "Control_Tools.h"



//----------------------------------------------------------Delta_PI------------------------------------------------------//
/**
 * 增量式PI初始化
 */
void PI_Delta_Init(PI_Controller *Pi, float kp, float ki, float min, float max)
{
    Pi->Kp = kp;
    Pi->Ki = ki;
    Pi->Delta_Min = min;
    Pi->Delta_Max = max;
    Pi->Error = 0.0f;
    Pi->Error_last = 0.0f;
}
/**
 * 增量式PI计算
 */
void PI_Delta_Cal(PI_Controller *Pi, float Pi_in)
{
    float pError;

    pError = Pi_in - Pi->Error_last;

    // PID计算
    Pi->Pi_Delta = (Pi->Kp * pError
                    + Pi->Ki * Pi_in);

    // 限幅
    if (Pi->Pi_Delta > Pi->Delta_Max|| Pi->Pi_Delta < Pi->Delta_Min){
        Pi->Error_last = Pi_in;
        Pi->Pi_Delta = (Pi->Pi_Delta > Pi->Delta_Max)? Pi->Delta_Max : Pi->Delta_Min;
    }
    else{
    	Pi->Error_last = Pi_in;
    }
}
//----------------------------------------------------------Pos_PI------------------------------------------------------//
/**
 * 位置式PI初始化
 */
void PI_Pos_Init(PI_Controller *Pi, float kp, float ki, float min, float max)
{
    Pi->Kp = kp;
    Pi->Ki = ki;
    Pi->integral = 0.0f;
    Pi->integral_min = min;
    Pi->integral_max = max;
    Pi->Error = 0.0f;
    Pi->Pos = 0.0f;
}
/**
 * 位置式PI计算
 */
void PI_Pos_Cal(PI_Controller *Pi, float error)
{
	Pi->Error = Pi->Kp * error;
	float integral_temp = Pi->integral + Pi->Ki * error;
	float output_temp = Pi->Error + integral_temp;
	if (output_temp > Pi->integral_max) {
		Pi->Pos = Pi->integral_max;
	} else if (output_temp < Pi->integral_min) {
		Pi->Pos = Pi->integral_min;
	} else {
		Pi->integral = integral_temp;
		Pi->Pos = output_temp;
	}
	if (Pi->integral > Pi->integral_max) {
		Pi->integral = Pi->integral_max;
	} else if (Pi->integral < Pi->integral_min) {
		Pi->integral = Pi->integral_min;
	}
}
//----------------------------------------------------------PR------------------------------------------------------//
/**
 * 准PR初始化：T为调用周期(s)，Wc为带宽参数(rad/s)，W0为中心角频率(rad/s)。
 * G(s) = Kp + 2*Kr*Wc*s / (s*s + 2*Wc*s + W0*W0)，采用双线性变换。
 */
void PR_Init(PR_Controller* ctrl, float T, float Kp, float Kr,float Wc, float W0, float out_limit) {
    float a0 = (4.0f / (T * T)) + (4.0f * Wc / T) + W0 * W0;
    float a1 = (-8.0f / (T * T)) + 2.0f * W0 * W0;
    float a2 = (4.0f / (T * T)) - (4.0f * Wc / T) + W0 * W0;
    float b0 = (4.0f * Kp) / (T * T) + (4.0f * Wc * (Kp + Kr) / T) + Kp * W0 * W0;
    float b1 = (-8.0f * Kp) / (T * T) + 2.0f * Kp * W0 * W0;
    float b2 = (4.0f * Kp) / (T * T) - (4.0f * Wc * (Kp + Kr) / T) + Kp * W0 * W0;
    ctrl->T=T;
    ctrl->kp=Kp;
    ctrl->kr=Kr;
    ctrl->W0=W0;
    ctrl->Wc=Wc;
    ctrl->b0 = b0 / a0;
    ctrl->b1 = b1 / a0;
    ctrl->b2 = b2 / a0;
    ctrl->a1 = a1 / a0;
    ctrl->a2 = a2 / a0;
    ctrl->u_prev1 = 0.0f;
    ctrl->u_prev2 = 0.0f;
    ctrl->y_prev1 = 0.0f;
    ctrl->y_prev2 = 0.0f;
    ctrl->output=0;
    ctrl->out_limit=out_limit;
}
/**
 * PR谐振滤波计算
 */
void PR_Cal(PR_Controller* ctrl, float input, float gain) {
	ctrl->output =
        ctrl->b0 * input +
        ctrl->b1 * ctrl->u_prev1 +
        ctrl->b2 * ctrl->u_prev2 -
        ctrl->a1 * ctrl->y_prev1 -
        ctrl->a2 * ctrl->y_prev2;
    ctrl->u_prev2 = ctrl->u_prev1;
    ctrl->u_prev1 = input;
    ctrl->y_prev2 = ctrl->y_prev1;
    ctrl->y_prev1 = ctrl->output;

	ctrl->output*=gain;
	ctrl->output = ctrl->output > ctrl->out_limit ? ctrl->out_limit :
			(ctrl->output < -ctrl->out_limit ? -ctrl->out_limit : ctrl->output);
}
//----------------------------------------------------------Clark & inv_Clark------------------------------------------------------//
/**
 * Clark变换计算
 */
void Clarke_F(float Ua, float Ub, float Uc, float *Ualpha, float *Ubeta)
{
    // α分量 = (2/3) * [Ua - 0.5*(Ub+Uc)]
    *Ualpha = (0.6666666667f) * (Ua-(Ub+Uc)/2.0f);
    // β分量 = (1/√3) * (Ub-Uc)
    *Ubeta = (0.5773502692f) * (Ub-Uc);
}
/**
 * 反Clark变换计算
 */
// void inv_Clarke_F(float Udc, float Ualpha, float Ubeta, float *w1, float *w2, float *w3)
// {
// 	//用ARR计数器的一半来做归一化，记得要改成输出PWM的定时器！！！！！！！！！！！！！
//     float k = (float)(TIM1->ARR + 1)/ Udc/ 2;
//     *w1 = k * (Udc + 2.0f * Ualpha);
//     *w2 = k * (Udc - Ualpha + sqrtf(3.0f) * Ubeta);
//     *w3 = k * (Udc - Ualpha - sqrtf(3.0f) * Ubeta);
// }
