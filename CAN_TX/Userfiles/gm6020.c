#include "gm6020.h"

volatile GM6020_Motor_t g_can1_motor = {0};
volatile GM6020_Motor_t g_can2_motor = {0};

void GM6020_Reset(volatile GM6020_Motor_t *motor)
{
    motor->angle_ecd      = 0U;
    motor->speed_rpm      = 0;
    motor->current_raw    = 0;
    motor->temperature    = 0U;
    motor->last_update_ms = 0U;
    motor->online         = 0U;
}

void GM6020_ParseFeedback(volatile GM6020_Motor_t *motor,
                          const uint8_t *data)
{
    motor->angle_ecd =
        (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);

    motor->speed_rpm =
        (int16_t)(((uint16_t)data[2] << 8U) | data[3]);

    motor->current_raw =
        (int16_t)(((uint16_t)data[4] << 8U) | data[5]);

    motor->temperature = data[6];

    motor->last_update_ms = HAL_GetTick();
    motor->online = 1U;
}

void GM6020_CheckOnline(volatile GM6020_Motor_t *motor,
                        uint32_t now_ms,
                        uint32_t timeout_ms)
{
    if ((uint32_t)(now_ms - motor->last_update_ms) > timeout_ms)
    {
        motor->online = 0U;
    }
}