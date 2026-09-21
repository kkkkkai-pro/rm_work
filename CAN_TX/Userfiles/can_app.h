#ifndef USERFILES_CAN_APP_H
#define USERFILES_CAN_APP_H

#include "main.h"
#include "can.h"

/*
 * 如果两个电机都是 ID=1，不需要修改。
 * 如果电机拨码或配置过 ID，在这里改。
 */
#define CAN1_GM6020_MOTOR_ID    1U
#define CAN2_GM6020_MOTOR_ID    1U

#define GM6020_FEEDBACK_ID(id)  ((uint32_t)(0x204U + (id)))

void CAN_App_Init(void);

HAL_StatusTypeDef GM6020_SendCurrent(CAN_HandleTypeDef *hcan,
                                     uint8_t motor_id,
                                     int16_t current);

#endif