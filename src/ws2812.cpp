#include "ws2812.h"

Adafruit_NeoPixel strip(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);

void ws2812Init(void) {
  strip.begin();
  strip.setBrightness(10);
  ws2812SetColor(0);
}

void ws2812SetColor(int color) {
  if (color == 0) {
    strip.setPixelColor(0, strip.Color(0, 0, 0));       // Off
  } else if (color == 1) {
    strip.setPixelColor(0, strip.Color(255, 0, 0));     // Red
  } else if (color == 2) {
    strip.setPixelColor(0, strip.Color(0, 255, 0));     // Green
  } else if (color == 3) {
    strip.setPixelColor(0, strip.Color(0, 0, 255));     // Blue
  }
  strip.show();
}
