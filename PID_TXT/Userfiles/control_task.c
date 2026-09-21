#include "control_task.h"

#include "cmsis_os.h"
#include "pid.h"
#include "gm6020.h"

static PID_t s_angle_pid;   /* 外环：角度 */
static PID_t s_speed_pid;   /* 内环：速度 */


volatile float g_target_angle_deg;   /* 目标角度（在 Ozone 里改它给阶跃） */
volatile float g_motor_angle_deg;    /* 电机当前连续角度（波形主变量） */
volatile float g_motor_speed_rpm;    /* 电机当前转速 */
volatile float g_target_speed_rpm;   /* 角度环输出的目标速度 */
volatile float g_angle_error_deg;    /* 角度误差 */
volatile float g_target_current;     /* 速度环输出的电流值 */
volatile uint8_t g_motor_online;     /* 电机在线标志 */

/* PID 参数（角度环） */
volatile float g_angle_kp   = 8.0f;
volatile float g_angle_ki   = 0.2f;
volatile float g_angle_kd   = 0.0f;
volatile float g_angle_kff  = 0.0f;   /* 前馈系数，先保持 0 */
volatile float g_angle_deadband  = 0.05f;  /* 死区 ±0.05° */
volatile float g_angle_int_sep   = 5.0f;   /* 误差 >5° 时不积分 */

/* PID 参数（速度环） */
volatile float g_speed_kp   = 6.0f;
volatile float g_speed_ki   = 0.6f;
volatile float g_speed_kd   = 0.0f;

/* 输出限幅 */
volatile float g_angle_out_max  = 320.0f;   /* 角度环输出 = 目标速度，GM6020 约 ±320 rpm */
volatile float g_speed_out_max  = 10000.0f; /* 速度环输出 = 电流值上限 */
volatile float g_speed_i_max    = 5000.0f;  /* 速度环积分限幅 */


static uint8_t s_target_initialized;  /* 目标角度是否已初始化为当前位置 */

void MotorControlTask(void const *argument)
{
    float target_speed;
    float current;

    (void)argument;

    
    PID_Init(&s_angle_pid,
             g_angle_kp, g_angle_ki, g_angle_kd, g_angle_kff,
             0.001f,        /* 控制周期 1 ms */
             100.0f,        /* 角度环积分限幅（目前 ki 默认为 0，不起作用；在线加 ki 时生效） */
             g_angle_out_max,
             g_angle_deadband,
             g_angle_int_sep);

    PID_Init(&s_speed_pid,
             g_speed_kp, g_speed_ki, g_speed_kd, 0.0f,
             0.001f,
             g_speed_i_max,
             g_speed_out_max,
             0.0f,          /* 速度环不用死区 */
             0.0f);         /* 速度环不用积分分离 */

    for (;;)
    {
        s_angle_pid.kp = g_angle_kp;
        s_angle_pid.ki = g_angle_ki;
        s_angle_pid.kd = g_angle_kd;
        s_angle_pid.kff = g_angle_kff;
        s_angle_pid.deadband = g_angle_deadband;
        s_angle_pid.int_sep_err = g_angle_int_sep;
        s_angle_pid.output_limit = g_angle_out_max;

        s_speed_pid.kp = g_speed_kp;
        s_speed_pid.ki = g_speed_ki;
        s_speed_pid.kd = g_speed_kd;
        s_speed_pid.integral_limit = g_speed_i_max;
        s_speed_pid.output_limit = g_speed_out_max;

        
        if ((g_motor.online == 0u) ||
            ((HAL_GetTick() - g_motor.last_update_ms) > GM6020_TIMEOUT_MS))
        {
            g_motor.online = 0u;
            g_motor_online = 0u;
            PID_Reset(&s_angle_pid);
            PID_Reset(&s_speed_pid);
            osDelay(1);
            continue;   /* 不在线：一帧都不发 */
        }
        g_motor_online = 1u;

        
        if (s_target_initialized == 0u)
        {
            g_target_angle_deg = g_motor.total_angle_deg;
            s_target_initialized = 1u;
            PID_Reset(&s_angle_pid);
            PID_Reset(&s_speed_pid);
        }

        
        g_motor_angle_deg  = g_motor.total_angle_deg;
        g_motor_speed_rpm  = (float)g_motor.speed_rpm;
        g_angle_error_deg  = g_target_angle_deg - g_motor_angle_deg;

        
        target_speed = PID_Calc(&s_angle_pid,
                                g_target_angle_deg,
                                g_motor_angle_deg);
        g_target_speed_rpm = target_speed;

       
        current = PID_Calc(&s_speed_pid,
                           target_speed,
                           g_motor_speed_rpm);
        g_target_current = current;

        
        GM6020_SendCurrent((int16_t)current);

        osDelay(1);
    }
}
