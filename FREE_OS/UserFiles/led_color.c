#include "led_color.h"

#include <stdbool.h>

#ifndef LED_ACTIVE_LOW
#define LED_ACTIVE_LOW 0
#endif

static void Led_WritePin(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    GPIO_PinState on_state;
    GPIO_PinState off_state;

#if LED_ACTIVE_LOW
    on_state = GPIO_PIN_RESET;
    off_state = GPIO_PIN_SET;
#else
    on_state = GPIO_PIN_SET;
    off_state = GPIO_PIN_RESET;
#endif

    HAL_GPIO_WritePin(port, pin, on ? on_state : off_state);
}

void Led_SetColor(led_color_t color)
{
    Led_WritePin(LED_R_GPIO_Port,
                 LED_R_Pin,
                 color == LED_COLOR_RED);

    Led_WritePin(LED_G_GPIO_Port,
                 LED_G_Pin,
                 color == LED_COLOR_GREEN);

    Led_WritePin(LED_B_GPIO_Port,
                 LED_B_Pin,
                 color == LED_COLOR_BLUE);
}