#ifndef SINGLE_RECTIFIER_CONTROL_H
#define SINGLE_RECTIFIER_CONTROL_H

#include <stdint.h>

/* 采样与锁相环的公开状态：中断里更新，主循环读取快照。 */
typedef struct
{
    float v1;           /* 最新一帧电压瞬时值，已去偏置；标定前仍是 ADC 换算单位 */
    float i1;           /* 最新一帧电流瞬时值，已去偏置 */
    float theta;        /* 电网电压相位，rad，0 约对应电压正弦正向过零 */
    float frequency;    /* PLL 估计的电网频率，Hz */
    float voltage_peak; /* SOGI 估计的电压峰值 */
    uint32_t locked;    /* 1 = 相位误差连续多帧低于门限 */
} SingleRectifier_ControlState;

/* 重新采样或上电时调用，清除 PLL 状态。 */
void SingleRectifier_ControlReset(void);

/* 每个 ADC 采样帧调用一次（DMA 半满/全满回调），输入为去偏置后的 V1、I1。
 * 返回 1 表示本帧跨过电压正向过零点，供采样偏置按电网周期结算。
 */
uint32_t SingleRectifier_ControlStep(float v1, float i1);

/* 读取一致快照；可在主循环调用。 */
void SingleRectifier_GetState(SingleRectifier_ControlState *state);

#endif /* SINGLE_RECTIFIER_CONTROL_H */
