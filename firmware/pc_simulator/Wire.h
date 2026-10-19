#pragma once
#include <Arduino.h>
class TwoWire { public: bool begin(int sda, int scl); bool setClock(uint32_t);
 void beginTransmission(int); size_t write(uint8_t); uint8_t endTransmission(bool stop=true);
 uint8_t requestFrom(int address, int size); int read(); };
extern TwoWire Wire;
