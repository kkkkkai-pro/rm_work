#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    
    float kp;
    float ki;
    float kd;
    float kff;            /* 前馈系数：目标值一阶导数的增益 */
    float dt;             /* 控制周期（秒），如 0.001 */

    float integral_limit; /* 积分限幅 */
    float output_limit;   /* 输出限幅 */

    float deadband;       /* 死区：|err| 小于该值时输出 0 */
    float int_sep_err;    /* 积分分离阈值：|err| 大于该值时不积分 */

    
    float integral;       /* 积分累加器 */
    float prev_err;       /* 上一次的误差 */
    float prev_ref;       /* 上一次的目标值（前馈用） */
    float output;         /* 最近一次输出 */
} PID_t;

void PID_Init(PID_t *pid,
              float kp, float ki, float kd, float kff,
              float dt,
              float integral_limit, float output_limit,
              float deadband, float int_sep_err);

float PID_Calc(PID_t *pid, float ref, float fdb);

void PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif 
