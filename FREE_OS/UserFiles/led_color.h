#ifndef LED_COLOR_H
#define LED_COLOR_H

#include "main.h"

typedef enum
{
    LED_COLOR_OFF = 0,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE
} led_color_t;

void Led_SetColor(led_color_t color);

#endif