#ifndef __WS2812_H
#define __WS2812_H

#include <Adafruit_NeoPixel.h>

#define WS2812_PIN  48

void ws2812Init(void);
void ws2812SetColor(int color);

#endif

