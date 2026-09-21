#include "can_app.h"
#include "gm6020.h"
#include "stm32f4xx_hal.h"
#include "can.h"      /* 提供 hcan1 的 extern 声明 */

#include <string.h>   /* memset */



static void CAN_Filter_Init(void)
{
    CAN_FilterTypeDef filter;

    memset(&filter, 0, sizeof(filter));

    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    /*
     * ID 和 Mask 都为 0：
     * 先接收所有 CAN 帧，方便调试。
     */
    filter.FilterIdHigh = 0x0000U;
    filter.FilterIdLow = 0x0000U;
    filter.FilterMaskIdHigh = 0x0000U;
    filter.FilterMaskIdLow = 0x0000U;

    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;

    /*
     * F407 的 CAN1、CAN2 共用 28 个过滤器组：
     * CAN1 使用 0~13
     * CAN2 使用 14~27
     */
    filter.SlaveStartFilterBank = 14U;

    /* CAN1 使用过滤器 0 */
    filter.FilterBank = 0U;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* CAN2 使用过滤器 14 */
    filter.FilterBank = 14U;

    if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_App_Init(void)
{
    GM6020_Reset(&g_can1_motor);
    GM6020_Reset(&g_can2_motor);

    CAN_Filter_Init();

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(
            &hcan1,
            CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(
            &hcan2,
            CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

static void CAN_DispatchFrame(CAN_HandleTypeDef *hcan,
                              CAN_RxHeaderTypeDef *rx_header,
                              uint8_t *rx_data)
{
    if (rx_header->IDE != CAN_ID_STD)
    {
        return;
    }

    if (rx_header->RTR != CAN_RTR_DATA)
    {
        return;
    }

    if (rx_header->DLC < 7U)
    {
        return;
    }

    if (hcan->Instance == CAN1)
    {
        if (rx_header->StdId ==
            GM6020_FEEDBACK_ID(CAN1_GM6020_MOTOR_ID))
        {
            GM6020_ParseFeedback(&g_can1_motor, rx_data);
        }
    }
    else if (hcan->Instance == CAN2)
    {
        if (rx_header->StdId ==
            GM6020_FEEDBACK_ID(CAN2_GM6020_MOTOR_ID))
        {
            GM6020_ParseFeedback(&g_can2_motor, rx_data);
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan,
                             CAN_RX_FIFO0,
                             &rx_header,
                             rx_data) != HAL_OK)
    {
        return;
    }

    CAN_DispatchFrame(hcan, &rx_header, rx_data);
}

/*
 * 后续 PID 控制时使用。
 * 当前“只读取数据”的任务可以暂时不调用。
 */
HAL_StatusTypeDef GM6020_SendCurrent(CAN_HandleTypeDef *hcan,
                                     uint8_t motor_id,
                                     int16_t current)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    uint32_t command_id;
    uint8_t index;

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    memset(&tx_header, 0, sizeof(tx_header));
    memset(tx_data, 0, sizeof(tx_data));

    if ((motor_id >= 1U) && (motor_id <= 4U))
    {
        command_id = 0x1FFU;
        index = (uint8_t)(motor_id - 1U);
    }
    else if ((motor_id >= 5U) && (motor_id <= 8U))
    {
        command_id = 0x2FFU;
        index = (uint8_t)(motor_id - 5U);
    }
    else
    {
        return HAL_ERROR;
    }

    tx_header.StdId = command_id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_data[index * 2U] =
        (uint8_t)(((uint16_t)current) >> 8U);

    tx_data[index * 2U + 1U] =
        (uint8_t)current;

    return HAL_CAN_AddTxMessage(hcan,
                                &tx_header,
                                tx_data,
                                &tx_mailbox);
}