#include "pid.h"

#include <math.h>

static float Limit(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

void PID_Init(PID_t *pid,
              float kp, float ki, float kd, float kff,
              float dt,
              float integral_limit, float output_limit,
              float deadband, float int_sep_err)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = kff;
    pid->dt = dt;

    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->deadband = deadband;
    pid->int_sep_err = int_sep_err;

    PID_Reset(pid);
}

float PID_Calc(PID_t *pid, float ref, float fdb)
{
    float err;
    float ref_dot;
    float p_out, i_out, d_out, ff_out;

    err = ref - fdb;

    /*
     * 死区：误差小于死区宽度时认为已经到位。
     * 输出 0、清积分，防止电机在目标位置附近来回抖动。
     */
    if (fabsf(err) < pid->deadband)
    {
        pid->integral = 0.0f;
        pid->prev_err = err;
        pid->prev_ref = ref;
        pid->output = 0.0f;
        return 0.0f;
    }

    /* 比例项 */
    p_out = pid->kp * err;

    /*
     * 积分项（积分分离）：
     * 误差很大时（比如刚给阶跃、电机还没动）不积分，
     * 否则积分会一路累积到饱和，到目标附近时造成大超调。
     * int_sep_err = 0 表示关闭积分分离（始终积分）。
     */
    if ((pid->int_sep_err <= 0.0f) || (fabsf(err) <= pid->int_sep_err))
    {
        pid->integral += err * pid->dt;
        pid->integral = Limit(pid->integral,
                              -pid->integral_limit,
                              pid->integral_limit);
    }
    i_out = pid->ki * pid->integral;

    /* 微分项（对误差微分） */
    d_out = pid->kd * (err - pid->prev_err) / pid->dt;

    /*
     * 前馈项（角速度一阶前馈）：
     * 目标是角度时，目标角速度 = d(ref)/dt。
     * 前馈把"目标要动多快"直接加进输出，电机不用等误差
     * 积出来才加速，能明显加快响应。
     */
    ref_dot = (ref - pid->prev_ref) / pid->dt;
    ff_out = pid->kff * ref_dot;

    pid->output = p_out + i_out + d_out + ff_out;
    pid->output = Limit(pid->output,
                        -pid->output_limit,
                        pid->output_limit);

    pid->prev_err = err;
    pid->prev_ref = ref;

    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
    pid->prev_ref = 0.0f;
    pid->output = 0.0f;
}
