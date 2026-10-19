#pragma once
#include <Arduino.h>
class BluetoothSerial : public Stream { public: bool begin(const char* name); };
