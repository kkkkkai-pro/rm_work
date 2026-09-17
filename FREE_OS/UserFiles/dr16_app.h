#ifndef DR16_APP_H
#define DR16_APP_H

#include "main.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>

#define DR16_FRAME_LENGTH       18U
#define DR16_RX_BUFFER_LENGTH   64U

typedef struct
{
    uint8_t data[DR16_RX_BUFFER_LENGTH];
    uint16_t length;
} dr16_rx_chunk_t;

typedef struct
{
    int16_t ch[4];
    uint8_t sw[2];
} dr16_data_t;

void DR16_AppInit(UART_HandleTypeDef *huart);

bool DR16_Decode(const uint8_t frame[DR16_FRAME_LENGTH],
                 dr16_data_t *data);

#endif