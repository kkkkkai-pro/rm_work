#ifndef USERFILES_GM6020_H
#define USERFILES_GM6020_H

#include "main.h"

typedef struct
{
    uint16_t angle_ecd;       /* 编码器角度，0~8191 */
    int16_t  speed_rpm;       /* 转速 */
    int16_t  current_raw;     /* 实际电流原始值 */
    uint8_t  temperature;     /* 温度 */
    uint32_t last_update_ms;  /* 最近一次收到数据的时间 */
    uint8_t  online;          /* 在线标志 */
} GM6020_Motor_t;

extern volatile GM6020_Motor_t g_can1_motor;
extern volatile GM6020_Motor_t g_can2_motor;

void GM6020_Reset(volatile GM6020_Motor_t *motor);

void GM6020_ParseFeedback(volatile GM6020_Motor_t *motor,
                          const uint8_t *data);

void GM6020_CheckOnline(volatile GM6020_Motor_t *motor,
                        uint32_t now_ms,
                        uint32_t timeout_ms);

#endif