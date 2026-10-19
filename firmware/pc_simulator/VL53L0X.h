#pragma once
#include <Arduino.h>
class VL53L0X { public:
 enum regAddr { SYSTEM_INTERRUPT_CLEAR=0x0B, RESULT_INTERRUPT_STATUS=0x13, RESULT_RANGE_STATUS=0x14 };
 void setAddress(uint8_t); bool init(bool io_2v8=true); void setTimeout(uint16_t);
 bool setMeasurementTimingBudget(uint32_t); void startContinuous(uint32_t period_ms=0);
 void writeReg(uint8_t reg, uint8_t value); uint8_t readReg(uint8_t reg); uint16_t readReg16Bit(uint8_t reg); };
