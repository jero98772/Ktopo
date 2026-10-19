/* =====================================================================================
 *  Ktopo — ESP32 firmware  ("the spinal cord")
 *  -------------------------------------------------------------------------------------
 *  Board:     ESP32 Dev Module (ESP32-WROOM-32, 30- or 38-pin DevKit V1)
 *  Core:      Arduino-ESP32 3.x  (also compiles with 2.x)
 *  Libraries: "VL53L0X" by Pololu  +  "Adafruit NeoPixel"   (Library Manager)
 *
 *  What it does:
 *   - Drives 3 BTS7960 drivers: LEFT side, RIGHT side and DRILL, with smooth ramps
 *     (the old code used a for loop with no delay, so its "ramp" lasted microseconds).
 *   - Reads 3 encoders, 2 VL53L0X distance sensors, the MPU6050 gyro and the battery voltage.
 *   - REFLEXES that keep working even if the Raspberry Pi hangs: it won't drive into
 *     obstacles, shuts everything off if it tips over, cuts the drill if it jams, protects
 *     the battery and stops if the Raspberry Pi stops talking (watchdog).
 *   - Takes commands from the Raspberry Pi over USB and from a phone over classic Bluetooth.
 *     Over Bluetooth it accepts THE SAME LETTERS as your old code: A a i I d D S M m 1 0
 *   - WS2812 LED ring: while the drill spins it shows two comets running in opposite directions.
 *
 *  USB protocol (115200 baud, one command per line):
 *     V <left> <right>   speed of each side, -100..100 (%)
 *     T <drill>          drill speed, -100..100 (%)
 *     L <mode>           lights: 0 off, 1 breathe, 2 searching, 3 target, 4 party, 5 headlights
 *     B <n>              beep n times
 *     P                  ping (keeps the link alive)
 *     X                  EMERGENCY STOP (stays latched until R)
 *     R                  clears faults (emergency stop, drill jam)
 *     ?                  prints version and status
 *     The old single letters (A, a, i, ...) are also accepted alone on a line.
 *
 *  Telemetry to the Raspberry Pi (20 times per second):
 *     E <rpmLeft> <rpmRight> <rpmDrill> <distLeft_mm> <distRight_mm> <pitch> <roll> <vbat> <flags>
 *       <outLeft> <outRight> <outDrill>
 * ===================================================================================== */
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#define VERSION "Ktopo firmware 1.1"

// -------------------------------------- PINS -----------------------------------------
const int LEFT_RPWM = 25, LEFT_LPWM = 26, LEFT_EN = 27;      // BTS7960 left side (R_EN + L_EN tied)
const int RIGHT_RPWM = 18, RIGHT_LPWM = 19, RIGHT_EN = 23;   // BTS7960 right side
const int DRILL_RPWM = 16, DRILL_LPWM = 17, DRILL_EN = 4;    // BTS7960 drill
const int ENC_LEFT_A = 34, ENC_LEFT_B = 35;                  // encoder, rear left motor
const int ENC_RIGHT_A = 36, ENC_RIGHT_B = 39;                // encoder, rear right motor (VP / VN)
const int ENC_DRILL_A = 33;                                  // drill motor encoder (channel A only)
const int I2C_SDA = 21, I2C_SCL = 22;                        // MPU6050 + 2x VL53L0X
const int TOF_LEFT_XSHUT = 13, TOF_RIGHT_XSHUT = 15;
const int LED_PIN = 5;                                       // WS2812 strip DIN (through a 330 ohm resistor)
const int BUZZER_PIN = 14;                                   // active buzzer through an NPN transistor
const int VBAT_PIN = 32;                                     // 100k / 27k voltage divider
const int STATUS_LED = 2;                                    // on-board blue LED

// ------------------------------------- SETTINGS --------------------------------------
// If one side drives backwards, flip its INVERT flag (the motors on each side are mirrored).
const bool INVERT_LEFT = false;
const bool INVERT_RIGHT = true;
const bool INVERT_DRILL = false;
const float DRIVE_GEAR_RATIO = 90.0;     // gear ratio of the drive motors (printed on the label)
const float DRILL_GEAR_RATIO = 30.0;     // gear ratio of the drill motor
const float ENCODER_PPR = 11.0;          // pulses per motor revolution (before the gearbox), channel A
const float DRILL_RPM_NOMINAL = 200.0;   // drill rpm at 100 % (used to detect jams)
const bool  USE_DRILL_ENCODER = true;    // set to false if the drill encoder is not wired
const float MANUAL_SPEED = 80.0;         // % used by the old Bluetooth commands
const int   PWM_FREQ = 20000;            // 20 kHz: no motor whine
const int   PWM_BITS = 10;               // 0..1023
const float PWM_MIN = 0.12;              // minimum duty to overcome friction
const float DRIVE_RAMP = 250.0;          // %/s
const float DRILL_RAMP = 70.0;           // %/s (gentle: protects the printed gears)
const int   OBST_MM = 260;               // braking distance measured from the chassis front
                                         // (the drill tip sticks out ~18 cm)
const float MAX_TILT_DEG = 35.0;         // degrees
const float VBAT_LOW = 10.5;             // 3S: 3.5 V per cell
const float VBAT_CRITICAL = 9.9;         // 3S: 3.3 V per cell
const float VBAT_DIVIDER = (100.0 + 27.0) / 27.0;
const uint32_t PI_WATCHDOG_MS = 600;     // if the Raspberry Pi is silent longer than this, stop
const uint32_t MANUAL_TIMEOUT_MS = 30000;
const int   NUM_LEDS = 26;               // collar ring (Ø140 mm, 60 LED/m strip)
const uint8_t BRIGHTNESS = 90;           // 0..255 (brighter = more current)

// -------------------------------------- FLAGS ----------------------------------------
enum {
  F_OBSTACLE = 1 << 0, F_TILTED = 1 << 1, F_DRILL_JAMMED = 1 << 2, F_BATT_LOW = 1 << 3,
  F_BATT_CRITICAL = 1 << 4, F_NO_PI = 1 << 5, F_ESTOP = 1 << 6, F_MANUAL = 1 << 7,
  F_TOF_LEFT_FAIL = 1 << 8, F_TOF_RIGHT_FAIL = 1 << 9, F_IMU_FAIL = 1 << 10, F_SHOW = 1 << 11
};

// ------------------------------------- OBJECTS ---------------------------------------
VL53L0X tofLeft, tofRight;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
BluetoothSerial SerialBT;

// -------------------------------------- STATE ----------------------------------------
float tgtLeft = 0, tgtRight = 0, tgtDrill = 0;   // what is requested (%)
float outLeft = 0, outRight = 0, outDrill = 0;   // what actually goes out, after ramps (%)
bool manualMode = false, estop = false, drillJammed = false;
bool tilted = false, battLow = false, battCritical = false, obstacle = false;
bool piSeen = false;
bool tofLeftOk = false, tofRightOk = false, imuOk = false;
uint32_t lastPiMs = 0, lastManualMs = 0;
int lightMode = 1;
int distLeft = 8190, distRight = 8190;           // mm (8190 = nothing in sight)
float pitch = 0, roll = 0, vbat = 12.0;
float rpmLeft = 0, rpmRight = 0, rpmDrill = 0;
float gyroBiasX = 0, gyroBiasY = 0;
uint32_t jamSince = 0, battLowSince = 0, battCritSince = 0;

volatile long cntLeft = 0, cntRight = 0, cntDrill = 0;

// ------------------------------------- ENCODERS --------------------------------------
void ARDUINO_ISR_ATTR isrLeft() { if (digitalRead(ENC_LEFT_B)) cntLeft--; else cntLeft++; }
void ARDUINO_ISR_ATTR isrRight() { if (digitalRead(ENC_RIGHT_B)) cntRight--; else cntRight++; }
void ARDUINO_ISR_ATTR isrDrill() { cntDrill++; }

// --------------------------------------- PWM -----------------------------------------
// Works with core 2.x (channels) and 3.x (pins)
const int PWM_PINS[6] = {LEFT_RPWM, LEFT_LPWM, RIGHT_RPWM, RIGHT_LPWM, DRILL_RPWM, DRILL_LPWM};

int channelOf(int pin) {
  for (int i = 0; i < 6; i++) if (PWM_PINS[i] == pin) return i;
  return 0;
}

void pwmInit(int pin) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(channelOf(pin), PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, channelOf(pin));
#endif
}

void pwmSetDuty(int pin, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channelOf(pin), duty);
#endif
}

// Writes a -100..100 percentage to one BTS7960
void setMotor(int rpwm, int lpwm, float pct, bool invert) {
  if (invert) pct = -pct;
  const uint32_t maxDuty = (1u << PWM_BITS) - 1;
  float a = fabsf(pct);
  uint32_t duty = 0;
  if (a > 0.5f) duty = (uint32_t)((PWM_MIN + (1.0f - PWM_MIN) * fminf(a, 100.0f) / 100.0f) * maxDuty);
  if (pct >= 0) { pwmSetDuty(lpwm, 0); pwmSetDuty(rpwm, duty); }
  else          { pwmSetDuty(rpwm, 0); pwmSetDuty(lpwm, duty); }
}

void enableDrivers(bool on) {
  digitalWrite(LEFT_EN, on); digitalWrite(RIGHT_EN, on); digitalWrite(DRILL_EN, on);
}

float ramp(float current, float target, float maxStep) {
  if (target > current) return fminf(target, current + maxStep);
  return fmaxf(target, current - maxStep);
}

void stopAll() { tgtLeft = tgtRight = tgtDrill = 0; }

// -------------------------------------- BUZZER ---------------------------------------
int pendingBeeps = 0;
uint32_t buzzerT = 0;
bool buzzerOn = false;

void beep(int n) { pendingBeeps += n; }

void updateBuzzer() {
  uint32_t now = millis();
  if (buzzerOn && now - buzzerT > 90) { digitalWrite(BUZZER_PIN, LOW); buzzerOn = false; buzzerT = now; }
  else if (!buzzerOn && pendingBeeps > 0 && now - buzzerT > 110) {
    digitalWrite(BUZZER_PIN, HIGH); buzzerOn = true; buzzerT = now; pendingBeeps--;
  }
}

// ----------------------------------- SHOW ("1") --------------------------------------
// The demo routine from the old code, but without blocking the program
struct Step { int8_t left, right, drill; uint16_t ms; };
const Step SHOW[] = {
  {60, -60, 0, 350}, {-60, 60, 0, 350}, {60, -60, 0, 350}, {-60, 60, 0, 350},
  {0, 0, 40, 800}, {0, 0, 80, 2200}, {0, 0, 0, 1200}, {-40, -40, 0, 600}, {0, 0, 0, 100}
};
const int N_SHOW = sizeof(SHOW) / sizeof(SHOW[0]);
int showStep = -1;
uint32_t showT = 0;

void startShow() { showStep = 0; showT = millis(); beep(3); }

void updateShow() {
  if (showStep < 0) return;
  if (millis() - showT > SHOW[showStep].ms) { showStep++; showT = millis(); }
  if (showStep >= N_SHOW) { showStep = -1; stopAll(); return; }
  tgtLeft = SHOW[showStep].left; tgtRight = SHOW[showStep].right; tgtDrill = SHOW[showStep].drill;
}

// --------------------------------- LEGACY COMMANDS -----------------------------------
// Same letters as the old Bluetooth app, so it keeps working unchanged
bool isLegacyKey(char c) { return c != 0 && strchr("AaiIdDSMm10", c) != nullptr; }

void legacyCommand(char c) {
  manualMode = true;
  lastManualMs = millis();
  if (c != '1') showStep = -1;
  switch (c) {
    case 'A': tgtLeft = tgtRight = MANUAL_SPEED; break;                   // forward
    case 'a': tgtLeft = tgtRight = -MANUAL_SPEED; break;                  // backward
    case 'i': tgtLeft = 0; tgtRight = MANUAL_SPEED; break;                // left (pivot)
    case 'I': tgtLeft = -MANUAL_SPEED; tgtRight = MANUAL_SPEED; break;    // crazy left (spin in place)
    case 'd': tgtLeft = MANUAL_SPEED; tgtRight = 0; break;                // right (pivot)
    case 'D': tgtLeft = MANUAL_SPEED; tgtRight = -MANUAL_SPEED; break;    // crazy right
    case 'S': tgtLeft = tgtRight = 0; break;                              // stop wheels
    case 'M': tgtDrill = 100; drillJammed = false; break;                 // drill at max
    case 'm': tgtDrill = 0; drillJammed = false; break;                   // drill off
    case '1': startShow(); break;                                         // show
    case '0': stopAll(); showStep = -1; manualMode = false; beep(1); break;  // all off, hand control back to the AI
  }
}

// ------------------------------------- PROTOCOL --------------------------------------
float clampPct(float v) { return fmaxf(-100.0f, fminf(100.0f, v)); }

void printInfo(Stream &s) {
  s.printf("I %s | ToF left:%s right:%s IMU:%s | vbat %.2f V\n", VERSION, tofLeftOk ? "ok" : "FAIL",
           tofRightOk ? "ok" : "FAIL", imuOk ? "ok" : "FAIL", vbat);
}

void handleLine(char *s) {
  for (char *p = s; *p; p++) if (*p == ',' || *p == ';') *p = ' ';
  while (*s == ' ') s++;
  size_t n = strlen(s);
  while (n > 0 && s[n - 1] == ' ') s[--n] = 0;
  if (n == 0) return;
  if (n == 1 && isLegacyKey(s[0])) { legacyCommand(s[0]); return; }
  char c = s[0];
  float a = 0, b = 0;
  switch (c) {
    case 'V':
      if (sscanf(s + 1, "%f %f", &a, &b) == 2) {
        lastPiMs = millis(); piSeen = true;
        if (!manualMode && showStep < 0) { tgtLeft = clampPct(a); tgtRight = clampPct(b); }
      }
      break;
    case 'T':
      if (sscanf(s + 1, "%f", &a) == 1) {
        lastPiMs = millis(); piSeen = true;
        if (!manualMode && showStep < 0) {
          if (fabsf(a) < 0.5f) drillJammed = false;   // asking for 0 clears the jam
          tgtDrill = clampPct(a);
        }
      }
      break;
    case 'L': if (sscanf(s + 1, "%f", &a) == 1) lightMode = (int)a; break;
    case 'B': if (sscanf(s + 1, "%f", &a) == 1) beep((int)a); break;
    case 'P': lastPiMs = millis(); piSeen = true; break;
    case 'X': estop = true; stopAll(); outLeft = outRight = outDrill = 0; beep(5); break;
    case 'R': estop = false; drillJammed = false; beep(1); break;
    case '?': printInfo(Serial); break;
  }
}

char usbBuf[64];
int usbLen = 0;

void readUSB() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { usbBuf[usbLen] = 0; handleLine(usbBuf); usbLen = 0; }
    else if (usbLen < (int)sizeof(usbBuf) - 1) usbBuf[usbLen++] = ch;
    else usbLen = 0;  // line too long: drop it
  }
}

void readBT() {
  while (SerialBT.available()) {
    char ch = (char)SerialBT.read();
    if (isLegacyKey(ch)) legacyCommand(ch);
    else if (ch == 'X') { estop = true; stopAll(); }
    else if (ch == 'R') { estop = false; drillJammed = false; }
  }
}

// -------------------------------------- SENSORS --------------------------------------
bool initToF(VL53L0X &s, int xshut, uint8_t address) {
  digitalWrite(xshut, HIGH);
  delay(10);
  s.setTimeout(60);
  if (!s.init()) return false;
  s.setAddress(address);
  s.setMeasurementTimingBudget(33000);
  s.startContinuous();
  return true;
}

// Non-blocking read: only reads when a new measurement is ready
void readToF(VL53L0X &s, bool ok, int &dist) {
  if (!ok) return;
  if ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return;
  uint16_t mm = s.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  s.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  dist = (mm == 0 || mm > 2000) ? 8190 : mm;
}

const uint8_t MPU = 0x68;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

bool mpuRead(int16_t v[7]) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU, 14) != 14) return false;
  for (int i = 0; i < 7; i++) { int16_t hi = Wire.read(); int16_t lo = Wire.read(); v[i] = (hi << 8) | lo; }
  return true;
}

bool initIMU() {
  Wire.beginTransmission(MPU);
  if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x00);   // wake up
  mpuWrite(0x1A, 0x03);   // 44 Hz filter
  mpuWrite(0x1B, 0x08);   // gyro ±500 °/s
  mpuWrite(0x1C, 0x08);   // accelerometer ±4 g
  delay(50);
  // calibrate the gyro (the rover must be still at power-up)
  int16_t v[7];
  long sx = 0, sy = 0; int n = 0;
  for (int i = 0; i < 200; i++) { if (mpuRead(v)) { sx += v[4]; sy += v[5]; n++; } delay(3); }
  if (n > 0) { gyroBiasX = (float)sx / n; gyroBiasY = (float)sy / n; }
  return n > 0;
}

// Angles from a complementary filter. The MPU6050 lies flat with its X axis pointing forward.
void updateIMU(float dt) {
  if (!imuOk) return;
  int16_t v[7];
  if (!mpuRead(v)) return;
  float ax = v[0] / 8192.0f, ay = v[1] / 8192.0f, az = v[2] / 8192.0f;
  float gx = (v[4] - gyroBiasX) / 65.5f, gy = (v[5] - gyroBiasY) / 65.5f;
  float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
  float rollAcc = atan2f(ay, az) * 57.2958f;
  pitch = 0.98f * (pitch + gy * dt) + 0.02f * pitchAcc;
  roll = 0.98f * (roll + gx * dt) + 0.02f * rollAcc;
}

void updateEncoders(float dt) {
  static long pLeft = 0, pRight = 0, pDrill = 0;
  noInterrupts();
  long l = cntLeft, r = cntRight, d = cntDrill;
  interrupts();
  float k = 60.0f / (ENCODER_PPR * DRIVE_GEAR_RATIO * dt);
  rpmLeft = (l - pLeft) * k * (INVERT_LEFT ? -1 : 1);
  rpmRight = (r - pRight) * k * (INVERT_RIGHT ? -1 : 1);
  rpmDrill = (d - pDrill) * 60.0f / (ENCODER_PPR * DRILL_GEAR_RATIO * dt);
  pLeft = l; pRight = r; pDrill = d;
}

void updateBattery() {
  float v = analogReadMilliVolts(VBAT_PIN) * VBAT_DIVIDER / 1000.0f;
  if (v < 3.0f) { vbat = v; battLow = battCritical = false; return; }  // no battery: USB power only
  vbat = 0.9f * vbat + 0.1f * v;
  uint32_t now = millis();
  if (vbat < VBAT_LOW) { if (!battLowSince) battLowSince = now; }
  else if (vbat > VBAT_LOW + 0.3f) { battLowSince = 0; battLow = false; }
  if (vbat < VBAT_CRITICAL) { if (!battCritSince) battCritSince = now; }
  else if (vbat > VBAT_CRITICAL + 0.5f) { battCritSince = 0; battCritical = false; }  // battery swapped
  else battCritSince = 0;
  if (battLowSince && now - battLowSince > 3000 && !battLow) { battLow = true; beep(2); }
  if (battCritSince && now - battCritSince > 3000 && !battCritical) { battCritical = true; beep(6); }
}

// -------------------------------------- REFLEXES -------------------------------------
void control(float dt) {
  uint32_t now = millis();
  updateShow();

  // Raspberry Pi watchdog (only while the Pi is in charge)
  bool noPi = !manualMode && showStep < 0 && (now - lastPiMs > PI_WATCHDOG_MS);
  if (noPi) stopAll();
  if (manualMode && now - lastManualMs > MANUAL_TIMEOUT_MS) { manualMode = false; stopAll(); }

  // tilt, with hysteresis
  if (fabsf(pitch) > MAX_TILT_DEG || fabsf(roll) > MAX_TILT_DEG) {
    if (!tilted) beep(4);
    tilted = true;
  } else if (fabsf(pitch) < MAX_TILT_DEG - 10 && fabsf(roll) < MAX_TILT_DEG - 10) tilted = false;

  obstacle = (distLeft < OBST_MM) || (distRight < OBST_MM);

  float left = tgtLeft, right = tgtRight, drill = tgtDrill;
  if (drillJammed) drill = 0;
  if (battLow) { drill = 0; left *= 0.5f; right *= 0.5f; }
  if (obstacle) {                        // remove forward motion, still allow turning and reversing
    float fwd = (left + right) / 2;
    if (fwd > 0) { left -= fwd; right -= fwd; }
  }
  if (estop || tilted || battCritical) { left = right = drill = 0; }

  outLeft = ramp(outLeft, left, DRIVE_RAMP * dt);
  outRight = ramp(outRight, right, DRIVE_RAMP * dt);
  outDrill = ramp(outDrill, drill, DRILL_RAMP * dt);
  if (estop) outLeft = outRight = outDrill = 0;

  // drill jam: lots of power but it barely turns
  if (USE_DRILL_ENCODER && fabsf(outDrill) > 40 && !drillJammed) {
    float expected = fabsf(outDrill) / 100.0f * DRILL_RPM_NOMINAL;
    if (fabsf(rpmDrill) < 0.2f * expected) {
      if (!jamSince) jamSince = now;
      if (now - jamSince > 600) { drillJammed = true; outDrill = 0; beep(3); }
    } else jamSince = 0;
  } else jamSince = 0;

  enableDrivers(!estop);
  setMotor(LEFT_RPWM, LEFT_LPWM, outLeft, INVERT_LEFT);
  setMotor(RIGHT_RPWM, RIGHT_LPWM, outRight, INVERT_RIGHT);
  setMotor(DRILL_RPWM, DRILL_LPWM, outDrill, INVERT_DRILL);
}

uint32_t statusFlags() {
  uint32_t f = 0;
  if (obstacle) f |= F_OBSTACLE;
  if (tilted) f |= F_TILTED;
  if (drillJammed) f |= F_DRILL_JAMMED;
  if (battLow) f |= F_BATT_LOW;
  if (battCritical) f |= F_BATT_CRITICAL;
  if (millis() - lastPiMs > PI_WATCHDOG_MS) f |= F_NO_PI;
  if (estop) f |= F_ESTOP;
  if (manualMode) f |= F_MANUAL;
  if (!tofLeftOk) f |= F_TOF_LEFT_FAIL;
  if (!tofRightOk) f |= F_TOF_RIGHT_FAIL;
  if (!imuOk) f |= F_IMU_FAIL;
  if (showStep >= 0) f |= F_SHOW;
  return f;
}

void sendTelemetry() {
  Serial.printf("E %.0f %.0f %.0f %d %d %.1f %.1f %.2f %lu %.0f %.0f %.0f\n", rpmLeft, rpmRight, rpmDrill,
                distLeft, distRight, pitch, roll, vbat, (unsigned long)statusFlags(), outLeft, outRight, outDrill);
}

// -------------------------------------- LIGHTS ---------------------------------------
uint32_t scaledColor(uint8_t r, uint8_t g, uint8_t b, float k) {
  k = fmaxf(0.0f, fminf(1.0f, k));
  return strip.Color((uint8_t)(r * k), (uint8_t)(g * k), (uint8_t)(b * k));
}

void updateLights() {
  static float phase = 0;
  uint32_t t = millis();
  bool blink = (t / 150) % 2;
  strip.clear();
  if (estop) {
    for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, blink ? strip.Color(255, 0, 0) : 0);
  } else if (tilted) {
    for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, blink ? strip.Color(160, 0, 255) : 0);
  } else if (battCritical || drillJammed) {
    for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, blink ? strip.Color(255, 90, 0) : 0);
  } else if (fabsf(outDrill) > 5) {
    // two comets running in opposite directions, like the counter-rotating drill
    phase += outDrill * 0.004f;
    for (int k = 0; k < 6; k++) {
      int a = ((int)floorf(phase) - k) % NUM_LEDS; if (a < 0) a += NUM_LEDS;
      int b = (NUM_LEDS - 1 - a + NUM_LEDS / 2) % NUM_LEDS;
      strip.setPixelColor(a, scaledColor(255, 90, 30, 1.0f - k / 6.0f));
      uint32_t prev = strip.getPixelColor(b);
      strip.setPixelColor(b, prev | scaledColor(60, 140, 255, 1.0f - k / 6.0f));
    }
  } else {
    float breathe = 0.5f + 0.5f * sinf(t / 600.0f);
    int m = manualMode ? 99 : lightMode;
    for (int i = 0; i < NUM_LEDS; i++) {
      uint32_t c = 0;
      switch (m) {
        case 0: c = 0; break;
        case 1: c = scaledColor(0, 200, 255, 0.15f + 0.6f * breathe); break;           // breathe
        case 2: {                                                                       // searching
          int pos = (t / 40) % NUM_LEDS;
          int dd = abs(i - pos); dd = min(dd, NUM_LEDS - dd);
          c = scaledColor(0, 80, 255, dd < 4 ? 1.0f - dd / 4.0f : 0.0f);
        } break;
        case 3: c = strip.Color(0, 255, 60); break;                                    // target
        case 4: c = strip.gamma32(strip.ColorHSV((uint16_t)(i * 65536L / NUM_LEDS + t * 40))); break;  // party
        case 5: c = strip.Color(255, 255, 255); break;                                 // headlights
        default: c = scaledColor(40, 255, 40, 0.2f + 0.5f * breathe); break;            // manual
      }
      strip.setPixelColor(i, c);
    }
    if (obstacle && blink) for (int i = 0; i < NUM_LEDS; i += 2) strip.setPixelColor(i, strip.Color(255, 0, 0));
  }
  strip.show();
}

// --------------------------------------- SETUP ---------------------------------------
void setup() {
  // drivers disabled while everything boots
  pinMode(LEFT_EN, OUTPUT); pinMode(RIGHT_EN, OUTPUT); pinMode(DRILL_EN, OUTPUT);
  enableDrivers(false);
  for (int i = 0; i < 6; i++) { pwmInit(PWM_PINS[i]); pwmSetDuty(PWM_PINS[i], 0); }
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(TOF_LEFT_XSHUT, OUTPUT); pinMode(TOF_RIGHT_XSHUT, OUTPUT);
  digitalWrite(TOF_LEFT_XSHUT, LOW); digitalWrite(TOF_RIGHT_XSHUT, LOW);
  // pins 34-39 have no internal pull-up: use 10k resistors to 3.3 V
  pinMode(ENC_LEFT_A, INPUT); pinMode(ENC_LEFT_B, INPUT);
  pinMode(ENC_RIGHT_A, INPUT); pinMode(ENC_RIGHT_B, INPUT);
  pinMode(ENC_DRILL_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRight, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DRILL_A), isrDrill, RISING);

  Serial.begin(115200);
  SerialBT.begin("Ktopo");
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.clear(); strip.show();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  tofLeftOk = initToF(tofLeft, TOF_LEFT_XSHUT, 0x30);
  tofRightOk = initToF(tofRight, TOF_RIGHT_XSHUT, 0x31);
  imuOk = initIMU();

  for (int i = 0; i < 20; i++) updateBattery();
  enableDrivers(true);
  printInfo(Serial);
  beep(2);
}

// --------------------------------------- LOOP ----------------------------------------
void loop() {
  static uint32_t tCtrl = 0, tEnc = 0, tBat = 0, tTel = 0, tLight = 0, tImu = 0;
  uint32_t now = millis();

  readUSB();
  readBT();
  readToF(tofLeft, tofLeftOk, distLeft);
  readToF(tofRight, tofRightOk, distRight);

  if (now - tImu >= 10) { updateIMU((now - tImu) / 1000.0f); tImu = now; }
  if (now - tCtrl >= 10) { control((now - tCtrl) / 1000.0f); tCtrl = now; }
  if (now - tEnc >= 100) { updateEncoders((now - tEnc) / 1000.0f); tEnc = now; }
  if (now - tBat >= 100) { updateBattery(); tBat = now; }
  if (now - tTel >= 50) { sendTelemetry(); tTel = now; }
  if (now - tLight >= 33) { updateLights(); tLight = now; }
  updateBuzzer();
  digitalWrite(STATUS_LED, (now / (manualMode ? 150 : 600)) % 2);
}
