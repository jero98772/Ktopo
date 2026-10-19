// Simulador en PC para probar la lógica del firmware
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
#include "../topo_esp32/topo_esp32.ino"
#undef BluetoothSerial
void send(const char* s){ while(*s) USB_IN.push_back(*s++); }
void run(int ms, const char* periodic=nullptr, int every=100){ for(int i=0;i<ms;i++){ if(periodic && i%every==0) send(periodic); T_MS++; loop(); } }
// la salida del lado izquierdo (con INVERTIR_IZQ=false) = duty RPWM - duty LPWM
int dutyL(){ return (int)DUTY[IZQ_RPWM]-(int)DUTY[IZQ_LPWM]; }
int dutyR(){ return (int)DUTY[DER_LPWM]-(int)DUTY[DER_RPWM]; } // invertido
int dutyT(){ return (int)DUTY[TAL_RPWM]-(int)DUTY[TAL_LPWM]; }
#define CHECK(c, msg) printf("%s  %s\n", (c)?"[OK] ":"[FALLA]", msg)
int main(){
  setup();
  printf("setup: %s", USB_OUT.c_str()); USB_OUT.clear();
  run(1000, "V 50 50\n", 100);
  printf("salidas tras 1 s de V 50 50: izq=%.1f der=%.1f dutyL=%d dutyR=%d en=%d\n", salIzq, salDer, dutyL(), dutyR(), PINS[IZQ_EN]);
  CHECK(fabs(salIzq-50)<0.1 && dutyL()>0 && dutyR()>0, "avanza con V 50 50, ambos lados hacia adelante");
  run(1000);
  CHECK(salIzq==0 && salDer==0, "watchdog: frena si la Raspberry deja de mandar");
  TOF_MM = 200; run(500, "V 60 60\n", 100);
  CHECK(fabs(salIzq)<0.1 && obstaculo, "obstáculo a 20 cm: no avanza");
  run(500, "V 60 -60\n", 100);
  CHECK(salIzq>50 && salDer<-50, "obstáculo: sí puede girar en su sitio");
  run(500, "V -40 -40\n", 100);
  CHECK(salIzq<-30, "obstáculo: sí puede retroceder");
  TOF_MM = 1500;
  run(3000, "T 80\n", 100);
  printf("taladro: sal=%.1f trabado=%d\n", salTal, taladroTrabado);
  CHECK(taladroTrabado, "taladro sin pulsos de encoder -> se detecta traba y se corta");
  send("T 0\n"); run(50); 
  for(int i=0;i<3000;i++){ if(i%100==0) send("T 80\n"); if(i%2==0) cntTal+=6; T_MS++; loop(); }
  printf("taladro con encoder girando: sal=%.1f rpm=%.0f trabado=%d\n", salTal, rpmTal, taladroTrabado);
  CHECK(!taladroTrabado && salTal>79, "taladro girando normal no se marca como trabado");
  send("T 0\n"); run(2000, "P\n", 100);
  BT_IN.push_back('A'); run(800);
  CHECK(modoManual && salIzq>79, "Bluetooth 'A' (comando viejo): adelante en modo manual");
  run(2000);
  CHECK(salIzq>79, "modo manual no se frena por el watchdog");
  BT_IN.push_back('S'); run(800);
  CHECK(salIzq==0, "Bluetooth 'S': stop");
  BT_IN.push_back('0'); run(50);
  CHECK(!modoManual, "Bluetooth '0': devuelve el control a la IA");
  IMU_AX = -6000; run(3000, "V 50 50\n", 100);
  printf("pitch=%.1f inclinado=%d salIzq=%.1f\n", pitch, inclinado, salIzq);
  CHECK(inclinado && salIzq==0, "inclinado >35 grados: todo se apaga");
  IMU_AX = 0; run(4000, "V 50 50\n", 100);
  CHECK(!inclinado && salIzq>49, "al enderezarse vuelve a obedecer");
  send("X\n"); run(20, "V 50 50\n", 10);
  CHECK(salIzq==0 && PINS[IZQ_EN]==0, "X: parada de emergencia inmediata con drivers deshabilitados");
  run(500, "V 50 50\n", 100);
  CHECK(salIzq==0, "emergencia queda bloqueada");
  send("R\n"); run(500, "V 50 50\n", 100);
  CHECK(salIzq>40, "R: borra la emergencia");
  VBAT_MV = 2200; run(7000, "V 80 80\n", 100);
  printf("vbat=%.2f baja=%d critica=%d salIzq=%.1f\n", vbat, batBaja, batCritica, salIzq);
  CHECK(batBaja && fabs(salIzq-40)<0.5, "batería baja: velocidad al 50 % y sin taladro");
  VBAT_MV = 2050; run(5000, "V 80 80\n", 100);
  CHECK(batCritica && salIzq==0, "batería crítica: todo apagado");
  VBAT_MV = 2700; run(6000, "P\n", 100);
  CHECK(!batCritica && !batBaja, "batería cambiada: se borran las alarmas");
  BT_IN.push_back('1'); run(200);
  CHECK(showPaso>=0, "show '1' arranca sin bloquear");
  run(8000);
  CHECK(showPaso<0 && salTal==0, "show termina y deja todo quieto");
  USB_OUT.clear(); run(100, "P\n", 50);
  size_t p = USB_OUT.find("E "); printf("telemetría: %s", USB_OUT.substr(p, USB_OUT.find('\n',p)-p+1).c_str());
  return 0;
}
