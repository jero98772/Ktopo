// PC simulator to test the firmware logic without hardware
// Build and run:  g++ -std=gnu++17 -DSIM -I. sim.cpp -o sim && ./sim
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>
#include <string>
#include <deque>
#include <map>
uint32_t T_MS = 0;
uint32_t millis(){ return T_MS; } uint32_t micros(){ return T_MS*1000; } void delay(uint32_t d){ T_MS += d; }
std::map<int,int> PINS; void pinMode(int,int){} void digitalWrite(int p,int v){PINS[p]=v;} int digitalRead(int p){return PINS[p];}
uint32_t VBAT_MV = 2700; uint32_t analogReadMilliVolts(int){ return VBAT_MV; }
int digitalPinToInterrupt(int p){return p;} void attachInterrupt(int, void(*)(), int){} void noInterrupts(){} void interrupts(){}
std::map<int,uint32_t> DUTY; bool ledcAttach(uint8_t, uint32_t, uint8_t){return true;} bool ledcWrite(uint8_t p, uint32_t d){DUTY[p]=d; return true;}
std::deque<char> USB_IN, BT_IN; std::string USB_OUT;
size_t Stream::printf(const char* f, ...){ char b[512]; va_list a; va_start(a,f); int n=vsnprintf(b,512,f,a); va_end(a); USB_OUT+=b; return n; }
size_t Stream::println(const char* s){ USB_OUT+=s; USB_OUT+="\n"; return 0;} size_t Stream::print(const char* s){USB_OUT+=s; return 0;}
struct USBSerial : HardwareSerial { int available() override { return USB_IN.size(); } int read() override { char c=USB_IN.front(); USB_IN.pop_front(); return c; } };
void HardwareSerial::begin(unsigned long){}
USBSerial SerialImpl;
#define Serial SerialImpl
bool BluetoothSerial::begin(const char*){return true;}
TwoWire Wire; bool TwoWire::begin(int,int){return true;} bool TwoWire::setClock(uint32_t){return true;}
void TwoWire::beginTransmission(int){} size_t TwoWire::write(uint8_t){return 1;}
uint8_t TwoWire::endTransmission(bool){return 0;}
int16_t IMU_AX=0, IMU_AY=0, IMU_AZ=8192; int imu_idx=0; int16_t imu_buf[7];
uint8_t TwoWire::requestFrom(int, int n){ imu_buf[0]=IMU_AX; imu_buf[1]=IMU_AY; imu_buf[2]=IMU_AZ; imu_buf[3]=0; imu_buf[4]=0; imu_buf[5]=0; imu_buf[6]=0; imu_idx=0; return n; }
int TwoWire::read(){ int i=imu_idx/2; int hi=(imu_idx%2)==0; imu_idx++; uint16_t v=(uint16_t)imu_buf[i]; return hi ? (v>>8)&0xff : v&0xff; }
uint16_t TOF_MM = 1500;
void VL53L0X::setAddress(uint8_t){} bool VL53L0X::init(bool){return true;} void VL53L0X::setTimeout(uint16_t){}
bool VL53L0X::setMeasurementTimingBudget(uint32_t){return true;} void VL53L0X::startContinuous(uint32_t){}
void VL53L0X::writeReg(uint8_t, uint8_t){} uint8_t VL53L0X::readReg(uint8_t){return 0x04;} uint16_t VL53L0X::readReg16Bit(uint8_t){return TOF_MM;}
Adafruit_NeoPixel::Adafruit_NeoPixel(uint16_t, int16_t, uint16_t){} void Adafruit_NeoPixel::begin(){} void Adafruit_NeoPixel::show(){}
void Adafruit_NeoPixel::clear(){} void Adafruit_NeoPixel::setBrightness(uint8_t){} void Adafruit_NeoPixel::setPixelColor(uint16_t, uint32_t){}
uint32_t Adafruit_NeoPixel::getPixelColor(uint16_t) const {return 0;} uint32_t Adafruit_NeoPixel::Color(uint8_t r,uint8_t g,uint8_t b){return (r<<16)|(g<<8)|b;}
uint32_t Adafruit_NeoPixel::ColorHSV(uint16_t,uint8_t,uint8_t){return 0;} uint32_t Adafruit_NeoPixel::gamma32(uint32_t x){return x;}
struct BTImpl : BluetoothSerial { int available() override { return BT_IN.size(); } int read() override { char c=BT_IN.front(); BT_IN.pop_front(); return c; } };
#define BluetoothSerial BTImpl
#include "../ktopo_esp32/ktopo_esp32.ino"
#undef BluetoothSerial
void send(const char* s){ while(*s) USB_IN.push_back(*s++); }
void run(int ms, const char* periodic=nullptr, int every=100){ for(int i=0;i<ms;i++){ if(periodic && i%every==0) send(periodic); T_MS++; loop(); } }
// left side output (with INVERT_LEFT = false) = RPWM duty - LPWM duty
int dutyL(){ return (int)DUTY[LEFT_RPWM]-(int)DUTY[LEFT_LPWM]; }
int dutyR(){ return (int)DUTY[RIGHT_LPWM]-(int)DUTY[RIGHT_RPWM]; } // inverted
#define CHECK(c, msg) printf("%s  %s\n", (c)?"[OK]  ":"[FAIL]", msg)
int main(){
  setup();
  printf("setup: %s", USB_OUT.c_str()); USB_OUT.clear();
  run(1000, "V 50 50\n", 100);
  printf("outputs after 1 s of V 50 50: left=%.1f right=%.1f dutyL=%d dutyR=%d en=%d\n", outLeft, outRight, dutyL(), dutyR(), PINS[LEFT_EN]);
  CHECK(fabs(outLeft-50)<0.1 && dutyL()>0 && dutyR()>0, "drives with V 50 50, both sides forward");
  run(1000);
  CHECK(outLeft==0 && outRight==0, "watchdog: stops when the Raspberry Pi goes silent");
  TOF_MM = 200; run(500, "V 60 60\n", 100);
  CHECK(fabs(outLeft)<0.1 && obstacle, "obstacle at 20 cm: no forward motion");
  run(500, "V 60 -60\n", 100);
  CHECK(outLeft>50 && outRight<-50, "obstacle: can still spin in place");
  run(500, "V -40 -40\n", 100);
  CHECK(outLeft<-30, "obstacle: can still reverse");
  TOF_MM = 1500;
  run(3000, "T 80\n", 100);
  printf("drill: out=%.1f jammed=%d\n", outDrill, drillJammed);
  CHECK(drillJammed, "drill with no encoder pulses -> jam detected and power cut");
  send("T 0\n"); run(50);
  for(int i=0;i<3000;i++){ if(i%100==0) send("T 80\n"); if(i%2==0) cntDrill+=6; T_MS++; loop(); }
  printf("drill with encoder turning: out=%.1f rpm=%.0f jammed=%d\n", outDrill, rpmDrill, drillJammed);
  CHECK(!drillJammed && outDrill>79, "a normally spinning drill is not flagged as jammed");
  send("T 0\n"); run(2000, "P\n", 100);
  BT_IN.push_back('A'); run(800);
  CHECK(manualMode && outLeft>79, "Bluetooth 'A' (old command): forward in manual mode");
  run(2000);
  CHECK(outLeft>79, "manual mode is not stopped by the watchdog");
  BT_IN.push_back('S'); run(800);
  CHECK(outLeft==0, "Bluetooth 'S': stop");
  BT_IN.push_back('0'); run(50);
  CHECK(!manualMode, "Bluetooth '0': hands control back to the AI");
  IMU_AX = -6000; run(3000, "V 50 50\n", 100);
  printf("pitch=%.1f tilted=%d outLeft=%.1f\n", pitch, tilted, outLeft);
  CHECK(tilted && outLeft==0, "tilted more than 35 degrees: everything shuts off");
  IMU_AX = 0; run(4000, "V 50 50\n", 100);
  CHECK(!tilted && outLeft>49, "obeys again once level");
  send("X\n"); run(20, "V 50 50\n", 10);
  CHECK(outLeft==0 && PINS[LEFT_EN]==0, "X: immediate emergency stop with drivers disabled");
  run(500, "V 50 50\n", 100);
  CHECK(outLeft==0, "emergency stop stays latched");
  send("R\n"); run(500, "V 50 50\n", 100);
  CHECK(outLeft>40, "R: clears the emergency stop");
  VBAT_MV = 2200; run(7000, "V 80 80\n", 100);
  printf("vbat=%.2f low=%d critical=%d outLeft=%.1f\n", vbat, battLow, battCritical, outLeft);
  CHECK(battLow && fabs(outLeft-40)<0.5, "low battery: half speed and no drill");
  VBAT_MV = 2050; run(5000, "V 80 80\n", 100);
  CHECK(battCritical && outLeft==0, "critical battery: everything off");
  VBAT_MV = 2700; run(6000, "P\n", 100);
  CHECK(!battCritical && !battLow, "battery swapped: alarms cleared");
  BT_IN.push_back('1'); run(200);
  CHECK(showStep>=0, "show '1' starts without blocking");
  run(8000);
  CHECK(showStep<0 && outDrill==0, "show ends and leaves everything still");
  USB_OUT.clear(); run(100, "P\n", 50);
  size_t p = USB_OUT.find("E "); printf("telemetry: %s", USB_OUT.substr(p, USB_OUT.find('\n',p)-p+1).c_str());
  return 0;
}
