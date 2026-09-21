#ifndef GM6020_H
#define GM6020_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can.h"

#define GM6020_MOTOR_ID     1u


#if (GM6020_MOTOR_ID <= 4u)
#define GM6020_TX_ID        0x1FFu
#else
#define GM6020_TX_ID        0x2FFu
#endif
#define GM6020_RX_ID        (0x204u + GM6020_MOTOR_ID)

#define GM6020_CURRENT_MAX  10000

#define GM6020_TIMEOUT_MS   100u

#define GM6020_ENCODER_FULL 8192.0f

/* ======================================================= */

typedef struct
{
    /* 原始反馈 */
    uint16_t raw_angle;          /* 机械角原始值 0~8191 */
    int16_t  speed_rpm;          /* 转速 rpm */
    int16_t  current;            /* 转矩电流（反馈值） */
    uint8_t  temperature;        /* 温度 ℃ */

    /* 解算值 */
    float    angle_deg;          /* 当前圈内的角度 0~360 */
    float    total_angle_deg;    /* 连续累计角度（跨圈不归零），位置环用它 */

    /* 状态 */
    uint32_t last_update_ms;     /* 最后一次收到反馈的 HAL_GetTick() 时间 */
    uint8_t  online;             /* 1 = 在线 */
} GM6020_t;

extern volatile GM6020_t g_motor; /* 电机状态（volatile 供 Ozone 观察） */


extern volatile uint32_t g_can_rx_count;
extern volatile uint32_t g_can_rx_motor_count;

void CAN_UserInit(void);

void GM6020_SendCurrent(int16_t current);

#ifdef __cplusplus
}

#endif

#endif 
