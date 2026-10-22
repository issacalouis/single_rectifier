#include "single_rectifier_control.h"

#include <math.h>

/* 采样帧率 = TIM8 触发 ADC 的频率：16 MHz、中心对齐 ARR=800，载波 10 kHz，
 * 每个载波周期上溢、下溢各触发一次。改时钟树或定时器后同步修改。 */
#define SR_SAMPLE_RATE_HZ    20000.0f
#define SR_SAMPLE_PERIOD_S   (1.0f / SR_SAMPLE_RATE_HZ)

#define SR_TWO_PI            6.28318530717958f

/* 电网标称 50 Hz。SOGI 谐振频率固定为标称值，频差由 PLL 的 PI 项补偿。 */
#define SR_GRID_FREQ_HZ      50.0f
#define SR_GRID_OMEGA        (SR_TWO_PI * SR_GRID_FREQ_HZ)

/* SOGI 阻尼增益。 */
#define SR_SOGI_K            1.41421356f

/* 二阶 PLL：自然频率 15 Hz、阻尼比 0.707。 */
#define SR_PLL_OMEGA_N       (SR_TWO_PI * 15.0f)
#define SR_PLL_DAMPING       0.70710678f
#define SR_PLL_KP            (2.0f * SR_PLL_DAMPING * SR_PLL_OMEGA_N)
#define SR_PLL_KI            (SR_PLL_OMEGA_N * SR_PLL_OMEGA_N)

/* 除法分母保护：输入幅度为 0 时避免 0/0。 */
#define SR_PLL_PEAK_EPS      1.0e-6f

/* 正弦查表：128 点加末端 0，线性插值。 */
#define SR_SIN_TABLE_SIZE    128U
#define SR_SIN_INDEX_SCALE   (128.0f / SR_TWO_PI)

typedef struct
{
    float x1;
    float x2;
    float integral;
} SingleRectifier_Pll;

static SingleRectifier_Pll sr_pll;
static SingleRectifier_ControlState sr_state;

static const float sr_sin_table[SR_SIN_TABLE_SIZE + 1U] = {
    0.00000000f,
    0.04906767f,
    0.09801714f,
    0.14673047f,
    0.19509032f,
    0.24298018f,
    0.29028468f,
    0.33688985f,
    0.38268343f,
    0.42755509f,
    0.47139674f,
    0.51410274f,
    0.55557023f,
    0.59569930f,
    0.63439328f,
    0.67155895f,
    0.70710678f,
    0.74095113f,
    0.77301045f,
    0.80320753f,
    0.83146961f,
    0.85772861f,
    0.88192126f,
    0.90398929f,
    0.92387953f,
    0.94154407f,
    0.95694034f,
    0.97003125f,
    0.98078528f,
    0.98917651f,
    0.99518473f,
    0.99879546f,
    1.00000000f,
    0.99879546f,
    0.99518473f,
    0.98917651f,
    0.98078528f,
    0.97003125f,
    0.95694034f,
    0.94154407f,
    0.92387953f,
    0.90398929f,
    0.88192126f,
    0.85772861f,
    0.83146961f,
    0.80320753f,
    0.77301045f,
    0.74095113f,
    0.70710678f,
    0.67155895f,
    0.63439328f,
    0.59569930f,
    0.55557023f,
    0.51410274f,
    0.47139674f,
    0.42755509f,
    0.38268343f,
    0.33688985f,
    0.29028468f,
    0.24298018f,
    0.19509032f,
    0.14673047f,
    0.09801714f,
    0.04906767f,
    0.00000000f,
    -0.04906767f,
    -0.09801714f,
    -0.14673047f,
    -0.19509032f,
    -0.24298018f,
    -0.29028468f,
    -0.33688985f,
    -0.38268343f,
    -0.42755509f,
    -0.47139674f,
    -0.51410274f,
    -0.55557023f,
    -0.59569930f,
    -0.63439328f,
    -0.67155895f,
    -0.70710678f,
    -0.74095113f,
    -0.77301045f,
    -0.80320753f,
    -0.83146961f,
    -0.85772861f,
    -0.88192126f,
    -0.90398929f,
    -0.92387953f,
    -0.94154407f,
    -0.95694034f,
    -0.97003125f,
    -0.98078528f,
    -0.98917651f,
    -0.99518473f,
    -0.99879546f,
    -1.00000000f,
    -0.99879546f,
    -0.99518473f,
    -0.98917651f,
    -0.98078528f,
    -0.97003125f,
    -0.95694034f,
    -0.94154407f,
    -0.92387953f,
    -0.90398929f,
    -0.88192126f,
    -0.85772861f,
    -0.83146961f,
    -0.80320753f,
    -0.77301045f,
    -0.74095113f,
    -0.70710678f,
    -0.67155895f,
    -0.63439328f,
    -0.59569930f,
    -0.55557023f,
    -0.51410274f,
    -0.47139674f,
    -0.42755509f,
    -0.38268343f,
    -0.33688985f,
    -0.29028468f,
    -0.24298018f,
    -0.19509032f,
    -0.14673047f,
    -0.09801714f,
    -0.04906767f,
    0.00000000f
};

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
    sr_state.i1 = 0.0f;
    sr_state.theta = 0.0f;
    sr_state.frequency = SR_GRID_FREQ_HZ;
}

uint32_t SingleRectifier_ControlStep(float v1, float i1)
{
    float error;
    float peak;
    float omega;
    float sin_theta;
    float cos_theta;
    float phase_error;

    sr_state.v1 = v1;
    sr_state.i1 = i1;

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

    sr_state.frequency = omega / SR_TWO_PI;
    sr_state.theta += omega * SR_SAMPLE_PERIOD_S;
    if (sr_state.theta >= SR_TWO_PI)
    {
        sr_state.theta -= SR_TWO_PI;
        return 1U;
    }
    return 0U;
}

void SingleRectifier_GetState(SingleRectifier_ControlState *state)
{
    *state = sr_state;
}
