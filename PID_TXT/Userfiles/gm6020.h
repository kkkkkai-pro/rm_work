/*
 * gm6020.h
 *
 * GM6020 电机驱动：
 *   - CAN 反馈接收与解码（机械角、转速、转矩电流、温度）
 *   - 机械角 0~360 解算为连续角度（可累计多圈）
 *   - 电流发送（ID1 电机用 0x1FF 标识符，数据在 Byte0/Byte1）
 *
 * 接线前请确认（RoboMaster 开发板 C 型）：
 *   CAN1 口（2-pin）：1脚=CANL，2脚=CANH（线：黑=CANL 红=CANH）
 *   C 板与 GM6020 必须同一个电源供电（共地，CAN1 口没有 GND 脚）
 *   电机拨码 ID = 1，供电 24V
 */
#ifndef GM6020_H
#define GM6020_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can.h"

/* ================= 可按现场情况修改的宏 ================= */

/* 电机拨码 ID：反馈帧 ID = 0x204 + ID，ID1 -> 0x205 */
#define GM6020_MOTOR_ID     1u

/* 发送帧标识符：ID1~4 用 0x1FF，ID5~7 用 0x2FF */
#if (GM6020_MOTOR_ID <= 4u)
#define GM6020_TX_ID        0x1FFu
#else
#define GM6020_TX_ID        0x2FFu
#endif
#define GM6020_RX_ID        (0x204u + GM6020_MOTOR_ID)

/* 电流值范围 ±10000（GM6020 手册规定） */
#define GM6020_CURRENT_MAX  10000

/* 反馈超时判定时间（毫秒），超过则认为电机掉线 */
#define GM6020_TIMEOUT_MS   100u

/* 编码器一圈的刻度 */
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

/* Ozone 诊断用：收到的 CAN 帧总数 / 其中本电机反馈帧数 */
extern volatile uint32_t g_can_rx_count;
extern volatile uint32_t g_can_rx_motor_count;

/*
 * CAN 用户初始化：配置过滤器、启动 CAN、开接收中断。
 * 必须在 MX_CAN1_Init() 之后调用一次（放在 main.c 的 USER CODE 2 里）。
 */
void CAN_UserInit(void);

/* 发送电流控制值，范围 ±GM6020_CURRENT_MAX */
void GM6020_SendCurrent(int16_t current);

#ifdef __cplusplus
}

#endif

#endif /* GM6020_H */
