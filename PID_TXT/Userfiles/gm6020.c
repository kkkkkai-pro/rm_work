#include "gm6020.h"

volatile GM6020_t g_motor;
volatile uint32_t g_can_rx_count;
volatile uint32_t g_can_rx_motor_count;



void CAN_UserInit(void)
{
    CAN_FilterTypeDef filter;

    filter.FilterMode = CAN_FILTERMODE_IDMASK;     /* 掩码模式 */
    filter.FilterScale = CAN_FILTERSCALE_32BIT;    /* 32 位过滤器 */
    filter.FilterIdHigh = 0x0000;                  /* ID 全 0     */
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;              /* 掩码全 0 = 全部接收 */
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;    /* 放进 FIFO0 */
    filter.FilterBank = 0;                         /* 用过滤器组 0 */
    filter.FilterActivation = ENABLE;

    
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}



static void FeedbackDecode(const uint8_t data[8])
{
    static uint16_t s_last_raw;      /* 上一次的原始角度，用于跨圈判断 */
    static uint8_t  s_first_frame;   /* 是否已收到第一帧 */
    int32_t delta;

    g_motor.raw_angle   = ((uint16_t)data[0] << 8) | data[1];
    g_motor.speed_rpm   = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    g_motor.current     = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    g_motor.temperature = data[6];

    /* 圈内角度 0~360 */
    g_motor.angle_deg =
        (float)g_motor.raw_angle * 360.0f / GM6020_ENCODER_FULL;

    if (s_first_frame == 0u)
    {
        /* 第一帧：以当前位置作为连续角度的零点 */
        s_last_raw = g_motor.raw_angle;
        g_motor.total_angle_deg = 0.0f;
        s_first_frame = 1u;
    }
    else
    {
        delta = (int32_t)g_motor.raw_angle - (int32_t)s_last_raw;

        /* 差值超过半圈 = 跨圈跳变，做 8192 补偿 */
        if (delta > (int32_t)(GM6020_ENCODER_FULL / 2.0f))
        {
            delta -= (int32_t)GM6020_ENCODER_FULL;
        }
        else if (delta < -(int32_t)(GM6020_ENCODER_FULL / 2.0f))
        {
            delta += (int32_t)GM6020_ENCODER_FULL;
        }

        g_motor.total_angle_deg +=
            (float)delta * 360.0f / GM6020_ENCODER_FULL;
        s_last_raw = g_motor.raw_angle;
    }

    g_motor.last_update_ms = HAL_GetTick();
    g_motor.online = 1u;
}

/*
 * HAL 的 CAN 接收回调：每收到一帧进一次。
 * 任何帧都让 g_can_rx_count 加 1，方便判断"线上有没有数据"。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (hcan->Instance != CAN1)
    {
        return;
    }
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                             &header, data) != HAL_OK)
    {
        return;
    }

    g_can_rx_count++;

    if ((header.IDE == CAN_ID_STD) && (header.StdId == GM6020_RX_ID))
    {
        g_can_rx_motor_count++;
        FeedbackDecode(data);
    }
}

/* ---------- 电流发送 ---------- */

void GM6020_SendCurrent(int16_t current)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t mailbox;

    if (current > GM6020_CURRENT_MAX)
    {
        current = GM6020_CURRENT_MAX;
    }
    if (current < -GM6020_CURRENT_MAX)
    {
        current = -GM6020_CURRENT_MAX;
    }

    tx_header.StdId = GM6020_TX_ID;
    tx_header.ExtId = 0u;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8u;
    tx_header.TransmitGlobalTime = DISABLE;

    
    tx_data[0] = (uint8_t)((uint16_t)current >> 8);
    tx_data[1] = (uint8_t)((uint16_t)current & 0xFF);

    (void)HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox);
}

