#include "dr16_app.h"
#include "led_color.h"

#include <string.h>

static UART_HandleTypeDef *s_huart;

static uint8_t s_uart_rx_buffer[DR16_RX_BUFFER_LENGTH];

static QueueHandle_t s_rx_queue;
static QueueHandle_t s_data_queue;

static bool IsChannelValueValid(uint16_t value)
{
    return value >= 200U && value <= 1800U;
}

bool DR16_Decode(const uint8_t frame[DR16_FRAME_LENGTH],
                 dr16_data_t *data)
{
    uint16_t raw_ch0;
    uint16_t raw_ch1;
    uint16_t raw_ch2;
    uint16_t raw_ch3;

    uint8_t sw1;
    uint8_t sw2;

    raw_ch0 = ((uint16_t)frame[0]
             | ((uint16_t)frame[1] << 8U)) & 0x07FFU;

    raw_ch1 = (((uint16_t)frame[1] >> 3U)
             | ((uint16_t)frame[2] << 5U)) & 0x07FFU;

    raw_ch2 = (((uint16_t)frame[2] >> 6U)
             | ((uint16_t)frame[3] << 2U)
             | ((uint16_t)frame[4] << 10U)) & 0x07FFU;

    raw_ch3 = (((uint16_t)frame[4] >> 1U)
             | ((uint16_t)frame[5] << 7U)) & 0x07FFU;

    sw1 = (frame[5] >> 4U) & 0x03U;
    sw2 = (frame[5] >> 6U) & 0x03U;

    if (!IsChannelValueValid(raw_ch0)
        || !IsChannelValueValid(raw_ch1)
        || !IsChannelValueValid(raw_ch2)
        || !IsChannelValueValid(raw_ch3))
    {
        return false;
    }

    if (sw1 < 1U || sw1 > 3U
        || sw2 < 1U || sw2 > 3U)
    {
        return false;
    }

    data->ch[0] = (int16_t)raw_ch0 - 1024;
    data->ch[1] = (int16_t)raw_ch1 - 1024;
    data->ch[2] = (int16_t)raw_ch2 - 1024;
    data->ch[3] = (int16_t)raw_ch3 - 1024;

    data->sw[0] = sw1;
    data->sw[1] = sw2;

    return true;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                uint16_t Size)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    dr16_rx_chunk_t chunk;

    if (huart != s_huart)
    {
        return;
    }

    if (s_rx_queue == NULL)
    {
        return;
    }

    if (Size > DR16_RX_BUFFER_LENGTH)
    {
        Size = DR16_RX_BUFFER_LENGTH;
    }

    memcpy(chunk.data, s_uart_rx_buffer, Size);
    chunk.length = Size;

    (void)xQueueSendFromISR(s_rx_queue,
                            &chunk,
                            &higher_priority_task_woken);

    /*
     * 每次收到空闲中断后都必须重新开启接收
     */
    (void)HAL_UARTEx_ReceiveToIdle_IT(s_huart,
                                      s_uart_rx_buffer,
                                      DR16_RX_BUFFER_LENGTH);

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void DR16_ReceiveTask(void *argument)
{
    dr16_rx_chunk_t chunk;
    uint8_t frame[DR16_FRAME_LENGTH];
    uint8_t used = 0U;
    dr16_data_t data;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(s_rx_queue,
                          &chunk,
                          portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        for (uint16_t i = 0U; i < chunk.length; i++)
        {
            frame[used] = chunk.data[i];
            used++;

            if (used < DR16_FRAME_LENGTH)
            {
                continue;
            }

            if (DR16_Decode(frame, &data))
            {
                /*
                 * data_queue 长度为 1，只保留最新数据
                 */
                (void)xQueueOverwrite(s_data_queue, &data);
                used = 0U;
            }
            else
            {
                /*
                 * 当前帧无效，向前移动一个字节重新寻找帧边界
                 */
                memmove(frame,
                        &frame[1],
                        DR16_FRAME_LENGTH - 1U);

                used = DR16_FRAME_LENGTH - 1U;
            }
        }
    }
}

static void LED_Task(void *argument)
{
    dr16_data_t data;

    (void)argument;

    Led_SetColor(LED_COLOR_OFF);

    for (;;)
    {
        if (xQueueReceive(s_data_queue,
                          &data,
                          pdMS_TO_TICKS(200)) == pdTRUE)
        {
            
            if (data.sw[0] == 1U)
            {
                Led_SetColor(LED_COLOR_RED);
            }
            else if (data.sw[0] == 2U)
            {
                Led_SetColor(LED_COLOR_GREEN);
            }
            else if (data.sw[0] == 3U)
            {
                Led_SetColor(LED_COLOR_BLUE);
            }
            else
            {
                Led_SetColor(LED_COLOR_OFF);
            }
        }
        else
        {
            Led_SetColor(LED_COLOR_OFF);
        }
    }
}

void DR16_AppInit(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    s_rx_queue = xQueueCreate(4U,
                               sizeof(dr16_rx_chunk_t));

    s_data_queue = xQueueCreate(1U,
                                sizeof(dr16_data_t));

    configASSERT(s_rx_queue != NULL);
    configASSERT(s_data_queue != NULL);

    configASSERT(xTaskCreate(DR16_ReceiveTask,
                             "dr16_rx",
                             256U,
                             NULL,
                             tskIDLE_PRIORITY + 3U,
                             NULL) == pdPASS);

    configASSERT(xTaskCreate(LED_Task,
                             "led",
                             256U,
                             NULL,
                             tskIDLE_PRIORITY + 2U,
                             NULL) == pdPASS);

    Led_SetColor(LED_COLOR_OFF);

    configASSERT(HAL_UARTEx_ReceiveToIdle_IT(
                     s_huart,
                     s_uart_rx_buffer,
                     DR16_RX_BUFFER_LENGTH) == HAL_OK);
}