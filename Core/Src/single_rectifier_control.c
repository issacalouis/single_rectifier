#include "single_rectifier_control.h"

#include <math.h>

#include "single_rectifier_control_private.h"

static float SingleRectifier_TableLookup(float pos)
{
    uint32_t index = (uint32_t)pos;

    return sr_sin_table[index] + (pos - (float)index) *
           (sr_sin_table[index + 1U] - sr_sin_table[index]);
}

static void SingleRectifier_SinCos(float theta, float *sin_out, float *cos_out)
{
    float pos;

    pos = theta * SR_SIN_INDEX_SCALE;
    *sin_out = SingleRectifier_TableLookup(pos);

    /* cos(theta) = sin(theta + 90 度)，90 度对应四分之一张表。 */
    pos = pos + 0.25f * (float)SR_SIN_TABLE_SIZE;
    if (pos >= (float)SR_SIN_TABLE_SIZE)
    {
        pos -= (float)SR_SIN_TABLE_SIZE;
    }
    *cos_out = SingleRectifier_TableLookup(pos);
}

void SingleRectifier_ControlReset(void)
{
    sr_pll.x1 = 0.0f;
    sr_pll.x2 = 0.0f;
    sr_pll.integral = 0.0f;

    sr_state.v1 = 0.0f;
    sr_state.theta = 0.0f;
    sr_state.frequency = SR_GRID_FREQ_HZ;
}

uint32_t SingleRectifier_ControlStep(float v1)
{
    float error;
    float peak;
    float omega;
    float sin_theta;
    float cos_theta;
    float phase_error;

    sr_state.v1 = v1;

    /* SOGI-QSG：x1 与输入同相，x2 滞后 90 度。一阶离散使锁定后 theta
     * 略超前电压相位，过零窗口仍覆盖整周期，偏置均值不受影响。 */
    error = v1 - sr_pll.x1;
    sr_pll.x1 += SR_SAMPLE_PERIOD_S * SR_GRID_OMEGA * (SR_SOGI_K * error - sr_pll.x2);
    sr_pll.x2 += SR_SAMPLE_PERIOD_S * SR_GRID_OMEGA * sr_pll.x1;
    peak = sqrtf(sr_pll.x1 * sr_pll.x1 + sr_pll.x2 * sr_pll.x2);

    SingleRectifier_SinCos(sr_state.theta, &sin_theta, &cos_theta);
    phase_error = (sr_pll.x1 * cos_theta + sr_pll.x2 * sin_theta) / (peak + SR_PLL_PEAK_EPS);

    sr_pll.integral += SR_PLL_KI * phase_error * SR_SAMPLE_PERIOD_S;
    omega = SR_GRID_OMEGA + SR_PLL_KP * phase_error + sr_pll.integral;

    sr_state.frequency = omega / TWO_PI;
    sr_state.theta += omega * SR_SAMPLE_PERIOD_S;
    if (sr_state.theta >= TWO_PI)
    {
        sr_state.theta -= TWO_PI;
        return 1U;
    }
    return 0U;
}

void SingleRectifier_GetState(SingleRectifier_ControlState *state)
{
    *state = sr_state;
}
