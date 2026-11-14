#ifndef SINGLE_RECTIFIER_CONTROL_H
#define SINGLE_RECTIFIER_CONTROL_H

#include <stdint.h>

/* 采样输入与锁相环输出：中断里更新，主循环读取快照。 */
typedef struct
{
    float v1;        /* 最新一帧电压瞬时值，已去偏置；未标定前是 ADC 换算单位 */
    float theta;     /* 电网电压相位，rad，0 约对应电压正弦正向过零 */
    float frequency; /* PLL 估计的电网频率，Hz */
    uint8_t PLL_done; /* PLL 锁定标志，0 未锁定，1 锁定 */
    float U_source_d; //电网侧d轴分量
    float U_source_q; //电网侧q轴分量
    float Id;         /* 交流电流的 d 轴分量 */
    float Iq;         /* 交流电流的 q 轴分量 */
} SingleRectifier_ControlState;

/* 重新采样或上电时调用，清除 PLL 状态。 */
void SingleRectifier_ControlReset(void);

/* 每个 ADC 采样帧调用一次，输入为去偏置后的电压 V1 和电流 I1。
 * 返回 1 表示本帧跨过电压正向过零点，供采样偏置按电网周期结算。
 */
uint32_t SingleRectifier_ControlStep(float v1, float i1);

/* 读取快照；可在主循环调用。 */
void SingleRectifier_GetState(SingleRectifier_ControlState *state);

#endif /* SINGLE_RECTIFIER_CONTROL_H */
