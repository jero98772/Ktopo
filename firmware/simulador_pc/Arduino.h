#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <algorithm>
using std::min; using std::max;
#define ESP_ARDUINO_VERSION_MAJOR 3
#define ARDUINO_ISR_ATTR
#define HIGH 1
#define LOW 0
#define INPUT 1
#define OUTPUT 2
#define INPUT_PULLUP 3
#define RISING 1
uint32_t millis(); uint32_t micros(); void delay(uint32_t);
void pinMode(int, int); void digitalWrite(int, int); int digitalRead(int);
uint32_t analogReadMilliVolts(int);
int digitalPinToInterrupt(int);
void attachInterrupt(int, void(*)(), int);
void noInterrupts(); void interrupts();
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t res);
bool ledcWrite(uint8_t pin, uint32_t duty);
class Stream { public:
  virtual int available(){return 0;} virtual int read(){return -1;}
  size_t printf(const char*, ...) __attribute__((format(printf,2,3)));
  size_t println(const char*); size_t print(const char*); };
class HardwareSerial : public Stream { public: void begin(unsigned long); };
#ifndef SIM
extern HardwareSerial Serial;
#endif
