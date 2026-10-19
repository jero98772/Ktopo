#pragma once
#include <Arduino.h>
#define NEO_GRB 0x52
#define NEO_KHZ800 0x0000
class Adafruit_NeoPixel { public: Adafruit_NeoPixel(uint16_t n, int16_t pin, uint16_t type);
 void begin(); void show(); void clear(); void setBrightness(uint8_t);
 void setPixelColor(uint16_t n, uint32_t c); uint32_t getPixelColor(uint16_t n) const;
 static uint32_t Color(uint8_t r, uint8_t g, uint8_t b);
 static uint32_t ColorHSV(uint16_t hue, uint8_t sat=255, uint8_t val=255);
 static uint32_t gamma32(uint32_t x); };
