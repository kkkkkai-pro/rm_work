/*
 * pid.h
 *
 * 位置式 PID，带以下可开关的增强功能（对应学习要求）：
 *   1. 积分限幅 + 输出限幅（必备，防积分饱和）
 *   2. 死区 PID       ：|误差| < deadband 时认为到位，输出 0 并清积分
 *   3. 积分分离 PID   ：|误差| > int_sep_err 时不积分，防止大误差阶段积分饱和
 *   4. 前馈 PID       ：out += kff * (目标值微分)，即"角速度一阶前馈"
 * 不需要的功能把对应参数置 0 即可。
 */
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* ----- 由用户设定的参数 ----- */
    float kp;
    float ki;
    float kd;
    float kff;            /* 前馈系数：目标值一阶导数的增益 */
    float dt;             /* 控制周期（秒），如 0.001 */

    float integral_limit; /* 积分限幅 */
    float output_limit;   /* 输出限幅 */

    float deadband;       /* 死区：|err| 小于该值时输出 0 */
    float int_sep_err;    /* 积分分离阈值：|err| 大于该值时不积分 */

    /* ----- 内部状态（用户不要直接改） ----- */
    float integral;       /* 积分累加器 */
    float prev_err;       /* 上一次的误差 */
    float prev_ref;       /* 上一次的目标值（前馈用） */
    float output;         /* 最近一次输出 */
} PID_t;

/* 初始化。不需要的功能对应参数填 0。 */
void PID_Init(PID_t *pid,
              float kp, float ki, float kd, float kff,
              float dt,
              float integral_limit, float output_limit,
              float deadband, float int_sep_err);

/* 计算一次 PID。ref = 目标值，fdb = 反馈值，返回输出。 */
float PID_Calc(PID_t *pid, float ref, float fdb);

/* 清空积分和历史误差（电机掉线、切换目标时调用）。 */
void PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
