/* =====================================================================================
 *  TOPO S14 — Firmware del ESP32  ("la médula espinal")
 *  -------------------------------------------------------------------------------------
 *  Placa:     ESP32 Dev Module (ESP32-WROOM-32, DevKit V1 de 30 o 38 pines)
 *  Núcleo:    Arduino-ESP32 3.x  (también compila con 2.x)
 *  Librerías: "VL53L0X" de Pololu  +  "Adafruit NeoPixel"   (Gestor de librerías)
 *
 *  Qué hace:
 *   - Maneja 3 drivers BTS7960: lado IZQUIERDO, lado DERECHO y TALADRO, con rampas suaves
 *     (el código viejo hacía un for sin delay: la "rampa" duraba microsegundos).
 *   - Lee 3 encoders, 2 sensores de distancia VL53L0X, el giroscopio MPU6050 y la batería.
 *   - REFLEJOS que funcionan aunque la Raspberry se cuelgue: no avanza contra obstáculos,
 *     apaga todo si se voltea, corta el taladro si se traba, cuida la batería y frena si
 *     la Raspberry deja de hablar (watchdog).
 *   - Recibe órdenes de la Raspberry Pi por USB y del celular por Bluetooth clásico.
 *     Por Bluetooth acepta LAS MISMAS LETRAS de tu código viejo: A a i I d D S M m 1 0
 *   - Anillo LED WS2812: cuando el taladro gira muestra dos cometas en sentidos opuestos.
 *
 *  Protocolo USB (115200 baudios, una orden por línea):
 *     V <izq> <der>    velocidad de cada lado, -100..100 (%)
 *     T <taladro>      velocidad del taladro, -100..100 (%)
 *     L <modo>         luces: 0 apagadas, 1 respirar, 2 buscando, 3 objetivo, 4 fiesta, 5 faros
 *     B <n>            pitar n veces
 *     P                ping (mantiene vivo el enlace)
 *     X                PARADA DE EMERGENCIA (queda bloqueado hasta recibir R)
 *     R                borra fallas (emergencia, taladro trabado)
 *     ?                imprime versión y configuración
 *     También acepta las letras viejas (A, a, i, ...) solas en una línea.
 *
 *  Telemetría hacia la Raspberry (20 veces por segundo):
 *     E <rpmIzq> <rpmDer> <rpmTal> <distIzq_mm> <distDer_mm> <pitch> <roll> <vbat> <banderas>
 *       <salidaIzq> <salidaDer> <salidaTal>
 * ===================================================================================== */
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#define VERSION "TOPO-S14 firmware 1.0"

// ------------------------------------- PINES -----------------------------------------
const int IZQ_RPWM = 25, IZQ_LPWM = 26, IZQ_EN = 27;   // BTS7960 lado izquierdo (R_EN+L_EN juntos)
const int DER_RPWM = 18, DER_LPWM = 19, DER_EN = 23;   // BTS7960 lado derecho
const int TAL_RPWM = 16, TAL_LPWM = 17, TAL_EN = 4;    // BTS7960 taladro
const int ENC_IZQ_A = 34, ENC_IZQ_B = 35;              // encoder motor trasero izquierdo
const int ENC_DER_A = 36, ENC_DER_B = 39;              // encoder motor trasero derecho (VP / VN)
const int ENC_TAL_A = 33;                              // encoder del motor del taladro (solo canal A)
const int I2C_SDA = 21, I2C_SCL = 22;                  // MPU6050 + 2x VL53L0X
const int TOF_IZQ_XSHUT = 13, TOF_DER_XSHUT = 15;
const int LED_PIN = 5;                                 // DIN de la tira WS2812 (con resistencia 330 ohm)
const int BUZZER_PIN = 14;                             // buzzer activo vía transistor NPN
const int VBAT_PIN = 32;                               // divisor 100k / 27k
const int LED_ESTADO = 2;                              // LED azul de la placa

// ------------------------------------ AJUSTES ----------------------------------------
// Si un lado avanza al revés, cambia su INVERTIR (los motores de un lado están en espejo).
const bool INVERTIR_IZQ = false;
const bool INVERTIR_DER = true;
const bool INVERTIR_TAL = false;
const float RELACION_TRACCION = 90.0;    // relación de la caja de los motores de tracción (etiqueta)
const float RELACION_TALADRO  = 30.0;    // relación de la caja del motor del taladro
const float PULSOS_MOTOR      = 11.0;    // pulsos por vuelta del motor (antes de la caja), canal A
const float RPM_TALADRO_NOMINAL = 200.0; // rpm del taladro al 100 % (para detectar trabas)
const bool  USAR_ENCODER_TALADRO = true; // ponlo en false si no conectaste el encoder del taladro
const float VEL_MANUAL = 80.0;           // % usado por los comandos Bluetooth viejos
const int   PWM_FREQ = 20000;            // 20 kHz: los motores no chillan
const int   PWM_BITS = 10;               // 0..1023
const float PWM_MIN = 0.12;              // fracción mínima para vencer la fricción
const float RAMPA_TRACCION = 250.0;      // %/s
const float RAMPA_TALADRO  = 70.0;       // %/s (suave: cuida los engranajes impresos)
const int   OBST_MM = 260;               // distancia de frenado medida desde el frente del chasis
                                         // (la punta del taladro sobresale ~18 cm)
const float INCLINACION_MAX = 35.0;      // grados
const float VBAT_BAJA = 10.5;            // 3S: 3.5 V por celda
const float VBAT_CRITICA = 9.9;          // 3S: 3.3 V por celda
const float DIVISOR_VBAT = (100.0 + 27.0) / 27.0;
const uint32_t WATCHDOG_PI_MS = 600;     // si la Raspberry calla más que esto, se frena
const uint32_t MANUAL_TIMEOUT_MS = 30000;
const int   NUM_LEDS = 26;               // anillo del collar (Ø140 mm, tira de 60 LED/m)
const uint8_t BRILLO = 90;               // 0..255 (más brillo = más consumo)

// ----------------------------------- BANDERAS ----------------------------------------
enum {
  F_OBSTACULO = 1 << 0, F_INCLINADO = 1 << 1, F_TALADRO_TRABADO = 1 << 2, F_BAT_BAJA = 1 << 3,
  F_BAT_CRITICA = 1 << 4, F_SIN_PI = 1 << 5, F_EMERGENCIA = 1 << 6, F_MANUAL = 1 << 7,
  F_TOF_IZQ_FALLA = 1 << 8, F_TOF_DER_FALLA = 1 << 9, F_IMU_FALLA = 1 << 10, F_SHOW = 1 << 11
};

// ------------------------------------ OBJETOS ----------------------------------------
VL53L0X tofIzq, tofDer;
Adafruit_NeoPixel tira(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
BluetoothSerial SerialBT;

// ------------------------------------ ESTADO -----------------------------------------
float objIzq = 0, objDer = 0, objTal = 0;   // lo que se pide (%)
float salIzq = 0, salDer = 0, salTal = 0;   // lo que realmente sale, con rampa (%)
bool modoManual = false, emergencia = false, taladroTrabado = false;
bool inclinado = false, batBaja = false, batCritica = false, obstaculo = false;
bool piVisto = false;
bool tofIzqOk = false, tofDerOk = false, imuOk = false;
uint32_t ultimoPi = 0, ultimoManual = 0;
int modoLuces = 1;
int distIzq = 8190, distDer = 8190;          // mm (8190 = nada a la vista)
float pitch = 0, roll = 0, vbat = 12.0;
float rpmIzq = 0, rpmDer = 0, rpmTal = 0;
float gyroBiasX = 0, gyroBiasY = 0;
uint32_t trabaDesde = 0, batBajaDesde = 0, batCritDesde = 0;

volatile long cntIzq = 0, cntDer = 0, cntTal = 0;

// ---------------------------------- ENCODERS -----------------------------------------
void ARDUINO_ISR_ATTR isrIzq() { if (digitalRead(ENC_IZQ_B)) cntIzq--; else cntIzq++; }
void ARDUINO_ISR_ATTR isrDer() { if (digitalRead(ENC_DER_B)) cntDer--; else cntDer++; }
void ARDUINO_ISR_ATTR isrTal() { cntTal++; }

// ------------------------------------- PWM -------------------------------------------
// Compatibilidad con el núcleo 2.x (canales) y 3.x (pines)
const int PINES_PWM[6] = {IZQ_RPWM, IZQ_LPWM, DER_RPWM, DER_LPWM, TAL_RPWM, TAL_LPWM};

int canalDe(int pin) {
  for (int i = 0; i < 6; i++) if (PINES_PWM[i] == pin) return i;
  return 0;
}

void pwmIniciar(int pin) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(canalDe(pin), PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, canalDe(pin));
#endif
}

void pwmEscribir(int pin, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(canalDe(pin), duty);
#endif
}

// Escribe un porcentaje -100..100 en un BTS7960
void motor(int rpwm, int lpwm, float pct, bool invertir) {
  if (invertir) pct = -pct;
  const uint32_t maxDuty = (1u << PWM_BITS) - 1;
  float a = fabsf(pct);
  uint32_t duty = 0;
  if (a > 0.5f) duty = (uint32_t)((PWM_MIN + (1.0f - PWM_MIN) * fminf(a, 100.0f) / 100.0f) * maxDuty);
  if (pct >= 0) { pwmEscribir(lpwm, 0); pwmEscribir(rpwm, duty); }
  else          { pwmEscribir(rpwm, 0); pwmEscribir(lpwm, duty); }
}

void habilitarDrivers(bool on) {
  digitalWrite(IZQ_EN, on); digitalWrite(DER_EN, on); digitalWrite(TAL_EN, on);
}

float rampa(float actual, float objetivo, float pasoMax) {
  if (objetivo > actual) return fminf(objetivo, actual + pasoMax);
  return fmaxf(objetivo, actual - pasoMax);
}

void todoQuieto() { objIzq = objDer = objTal = 0; }

// ------------------------------------ BUZZER -----------------------------------------
int pitidosPendientes = 0;
uint32_t buzzerT = 0;
bool buzzerOn = false;

void pitar(int n) { pitidosPendientes += n; }

void actualizarBuzzer() {
  uint32_t ahora = millis();
  if (buzzerOn && ahora - buzzerT > 90) { digitalWrite(BUZZER_PIN, LOW); buzzerOn = false; buzzerT = ahora; }
  else if (!buzzerOn && pitidosPendientes > 0 && ahora - buzzerT > 110) {
    digitalWrite(BUZZER_PIN, HIGH); buzzerOn = true; buzzerT = ahora; pitidosPendientes--;
  }
}

// --------------------------------- SHOW ("1") ----------------------------------------
// El "FUN-CIO-NA" del código viejo, pero sin bloquear el programa
struct Paso { int8_t izq, der, tal; uint16_t ms; };
const Paso SHOW[] = {
  {60, -60, 0, 350}, {-60, 60, 0, 350}, {60, -60, 0, 350}, {-60, 60, 0, 350},
  {0, 0, 40, 800}, {0, 0, 80, 2200}, {0, 0, 0, 1200}, {-40, -40, 0, 600}, {0, 0, 0, 100}
};
const int N_SHOW = sizeof(SHOW) / sizeof(SHOW[0]);
int showPaso = -1;
uint32_t showT = 0;

void iniciarShow() { showPaso = 0; showT = millis(); pitar(3); }

void actualizarShow() {
  if (showPaso < 0) return;
  if (millis() - showT > SHOW[showPaso].ms) { showPaso++; showT = millis(); }
  if (showPaso >= N_SHOW) { showPaso = -1; todoQuieto(); return; }
  objIzq = SHOW[showPaso].izq; objDer = SHOW[showPaso].der; objTal = SHOW[showPaso].tal;
}

// -------------------------------- COMANDOS VIEJOS ------------------------------------
bool esLetraVieja(char c) { return strchr("AaiIdDSMm10", c) != nullptr && c != 0; }

void comandoViejo(char c) {
  modoManual = true;
  ultimoManual = millis();
  if (c != '1') showPaso = -1;
  switch (c) {
    case 'A': objIzq = objDer = VEL_MANUAL; break;                 // adelante
    case 'a': objIzq = objDer = -VEL_MANUAL; break;                // atrás
    case 'i': objIzq = 0; objDer = VEL_MANUAL; break;              // izquierda (pivote)
    case 'I': objIzq = -VEL_MANUAL; objDer = VEL_MANUAL; break;    // izquierda loca (gira en su sitio)
    case 'd': objIzq = VEL_MANUAL; objDer = 0; break;              // derecha (pivote)
    case 'D': objIzq = VEL_MANUAL; objDer = -VEL_MANUAL; break;    // derecha loca
    case 'S': objIzq = objDer = 0; break;                          // stop ruedas
    case 'M': objTal = 100; taladroTrabado = false; break;         // taladro al máximo
    case 'm': objTal = 0; taladroTrabado = false; break;           // taladro apagado
    case '1': iniciarShow(); break;                                // show
    case '0': todoQuieto(); showPaso = -1; modoManual = false; pitar(1); break;  // apagar y devolver a la IA
  }
}

// ---------------------------------- PROTOCOLO ----------------------------------------
float limitar(float v) { return fmaxf(-100.0f, fminf(100.0f, v)); }

void imprimirInfo(Stream &s) {
  s.printf("I %s | ToF izq:%s der:%s IMU:%s | vbat %.2f V\n", VERSION, tofIzqOk ? "ok" : "FALLA",
           tofDerOk ? "ok" : "FALLA", imuOk ? "ok" : "FALLA", vbat);
}

void procesarLinea(char *s) {
  for (char *p = s; *p; p++) if (*p == ',' || *p == ';') *p = ' ';
  while (*s == ' ') s++;
  size_t n = strlen(s);
  while (n > 0 && s[n - 1] == ' ') s[--n] = 0;
  if (n == 0) return;
  if (n == 1 && esLetraVieja(s[0])) { comandoViejo(s[0]); return; }
  char c = s[0];
  float a = 0, b = 0;
  switch (c) {
    case 'V':
      if (sscanf(s + 1, "%f %f", &a, &b) == 2) {
        ultimoPi = millis(); piVisto = true;
        if (!modoManual && showPaso < 0) { objIzq = limitar(a); objDer = limitar(b); }
      }
      break;
    case 'T':
      if (sscanf(s + 1, "%f", &a) == 1) {
        ultimoPi = millis(); piVisto = true;
        if (!modoManual && showPaso < 0) {
          if (fabsf(a) < 0.5f) taladroTrabado = false;   // pedir 0 borra la traba
          objTal = limitar(a);
        }
      }
      break;
    case 'L': if (sscanf(s + 1, "%f", &a) == 1) modoLuces = (int)a; break;
    case 'B': if (sscanf(s + 1, "%f", &a) == 1) pitar((int)a); break;
    case 'P': ultimoPi = millis(); piVisto = true; break;
    case 'X': emergencia = true; todoQuieto(); salIzq = salDer = salTal = 0; pitar(5); break;
    case 'R': emergencia = false; taladroTrabado = false; pitar(1); break;
    case '?': imprimirInfo(Serial); break;
  }
}

char bufUSB[64];
int lenUSB = 0;

void leerUSB() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { bufUSB[lenUSB] = 0; procesarLinea(bufUSB); lenUSB = 0; }
    else if (lenUSB < (int)sizeof(bufUSB) - 1) bufUSB[lenUSB++] = ch;
    else lenUSB = 0;  // línea demasiado larga: se descarta
  }
}

void leerBT() {
  while (SerialBT.available()) {
    char ch = (char)SerialBT.read();
    if (esLetraVieja(ch)) comandoViejo(ch);
    else if (ch == 'X') { emergencia = true; todoQuieto(); }
    else if (ch == 'R') { emergencia = false; taladroTrabado = false; }
  }
}

// ---------------------------------- SENSORES -----------------------------------------
bool iniciarToF(VL53L0X &s, int xshut, uint8_t direccion) {
  digitalWrite(xshut, HIGH);
  delay(10);
  s.setTimeout(60);
  if (!s.init()) return false;
  s.setAddress(direccion);
  s.setMeasurementTimingBudget(33000);
  s.startContinuous();
  return true;
}

// Lectura sin bloquear: solo lee si hay una medición nueva lista
void leerToF(VL53L0X &s, bool ok, int &dist) {
  if (!ok) return;
  if ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return;
  uint16_t mm = s.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  s.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  dist = (mm == 0 || mm > 2000) ? 8190 : mm;
}

const uint8_t MPU = 0x68;

void mpuEscribir(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

bool mpuLeer(int16_t v[7]) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU, 14) != 14) return false;
  for (int i = 0; i < 7; i++) { int16_t hi = Wire.read(); int16_t lo = Wire.read(); v[i] = (hi << 8) | lo; }
  return true;
}

bool iniciarIMU() {
  Wire.beginTransmission(MPU);
  if (Wire.endTransmission() != 0) return false;
  mpuEscribir(0x6B, 0x00);   // despertar
  mpuEscribir(0x1A, 0x03);   // filtro 44 Hz
  mpuEscribir(0x1B, 0x08);   // giroscopio ±500 °/s
  mpuEscribir(0x1C, 0x08);   // acelerómetro ±4 g
  delay(50);
  // calibrar el giroscopio (el carro debe estar quieto al encender)
  int16_t v[7];
  long sx = 0, sy = 0; int n = 0;
  for (int i = 0; i < 200; i++) { if (mpuLeer(v)) { sx += v[4]; sy += v[5]; n++; } delay(3); }
  if (n > 0) { gyroBiasX = (float)sx / n; gyroBiasY = (float)sy / n; }
  return n > 0;
}

// Ángulos con filtro complementario. El MPU6050 va plano con su eje X hacia adelante.
void actualizarIMU(float dt) {
  if (!imuOk) return;
  int16_t v[7];
  if (!mpuLeer(v)) return;
  float ax = v[0] / 8192.0f, ay = v[1] / 8192.0f, az = v[2] / 8192.0f;
  float gx = (v[4] - gyroBiasX) / 65.5f, gy = (v[5] - gyroBiasY) / 65.5f;
  float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
  float rollAcc = atan2f(ay, az) * 57.2958f;
  pitch = 0.98f * (pitch + gy * dt) + 0.02f * pitchAcc;
  roll = 0.98f * (roll + gx * dt) + 0.02f * rollAcc;
}

void actualizarEncoders(float dt) {
  static long pIzq = 0, pDer = 0, pTal = 0;
  noInterrupts();
  long i = cntIzq, d = cntDer, t = cntTal;
  interrupts();
  float k = 60.0f / (PULSOS_MOTOR * RELACION_TRACCION * dt);
  rpmIzq = (i - pIzq) * k * (INVERTIR_IZQ ? -1 : 1);
  rpmDer = (d - pDer) * k * (INVERTIR_DER ? -1 : 1);
  rpmTal = (t - pTal) * 60.0f / (PULSOS_MOTOR * RELACION_TALADRO * dt);
  pIzq = i; pDer = d; pTal = t;
}

void actualizarBateria() {
  float v = analogReadMilliVolts(VBAT_PIN) * DIVISOR_VBAT / 1000.0f;
  if (v < 3.0f) { vbat = v; batBaja = batCritica = false; return; }  // sin batería: solo USB
  vbat = 0.9f * vbat + 0.1f * v;
  uint32_t ahora = millis();
  if (vbat < VBAT_BAJA) { if (!batBajaDesde) batBajaDesde = ahora; }
  else if (vbat > VBAT_BAJA + 0.3f) { batBajaDesde = 0; batBaja = false; }
  if (vbat < VBAT_CRITICA) { if (!batCritDesde) batCritDesde = ahora; }
  else if (vbat > VBAT_CRITICA + 0.5f) { batCritDesde = 0; batCritica = false; }  // batería cambiada
  else batCritDesde = 0;
  if (batBajaDesde && ahora - batBajaDesde > 3000 && !batBaja) { batBaja = true; pitar(2); }
  if (batCritDesde && ahora - batCritDesde > 3000 && !batCritica) { batCritica = true; pitar(6); }
}

// ----------------------------------- REFLEJOS ----------------------------------------
void controlar(float dt) {
  uint32_t ahora = millis();
  actualizarShow();

  // watchdog de la Raspberry (solo cuando ella manda)
  bool sinPi = !modoManual && showPaso < 0 && (ahora - ultimoPi > WATCHDOG_PI_MS);
  if (sinPi) todoQuieto();
  if (modoManual && ahora - ultimoManual > MANUAL_TIMEOUT_MS) { modoManual = false; todoQuieto(); }

  // inclinación con histéresis
  if (fabsf(pitch) > INCLINACION_MAX || fabsf(roll) > INCLINACION_MAX) {
    if (!inclinado) pitar(4);
    inclinado = true;
  } else if (fabsf(pitch) < INCLINACION_MAX - 10 && fabsf(roll) < INCLINACION_MAX - 10) inclinado = false;

  obstaculo = (distIzq < OBST_MM) || (distDer < OBST_MM);

  float izq = objIzq, der = objDer, tal = objTal;
  if (taladroTrabado) tal = 0;
  if (batBaja) { tal = 0; izq *= 0.5f; der *= 0.5f; }
  if (obstaculo) {                       // quita el avance, deja girar y retroceder
    float avance = (izq + der) / 2;
    if (avance > 0) { izq -= avance; der -= avance; }
  }
  if (emergencia || inclinado || batCritica) { izq = der = tal = 0; }

  salIzq = rampa(salIzq, izq, RAMPA_TRACCION * dt);
  salDer = rampa(salDer, der, RAMPA_TRACCION * dt);
  salTal = rampa(salTal, tal, RAMPA_TALADRO * dt);
  if (emergencia) salIzq = salDer = salTal = 0;

  // taladro trabado: mucha potencia pero casi no gira
  if (USAR_ENCODER_TALADRO && fabsf(salTal) > 40 && !taladroTrabado) {
    float esperado = fabsf(salTal) / 100.0f * RPM_TALADRO_NOMINAL;
    if (fabsf(rpmTal) < 0.2f * esperado) {
      if (!trabaDesde) trabaDesde = ahora;
      if (ahora - trabaDesde > 600) { taladroTrabado = true; salTal = 0; pitar(3); }
    } else trabaDesde = 0;
  } else trabaDesde = 0;

  habilitarDrivers(!emergencia);
  motor(IZQ_RPWM, IZQ_LPWM, salIzq, INVERTIR_IZQ);
  motor(DER_RPWM, DER_LPWM, salDer, INVERTIR_DER);
  motor(TAL_RPWM, TAL_LPWM, salTal, INVERTIR_TAL);
}

uint32_t banderas() {
  uint32_t f = 0;
  if (obstaculo) f |= F_OBSTACULO;
  if (inclinado) f |= F_INCLINADO;
  if (taladroTrabado) f |= F_TALADRO_TRABADO;
  if (batBaja) f |= F_BAT_BAJA;
  if (batCritica) f |= F_BAT_CRITICA;
  if (millis() - ultimoPi > WATCHDOG_PI_MS) f |= F_SIN_PI;
  if (emergencia) f |= F_EMERGENCIA;
  if (modoManual) f |= F_MANUAL;
  if (!tofIzqOk) f |= F_TOF_IZQ_FALLA;
  if (!tofDerOk) f |= F_TOF_DER_FALLA;
  if (!imuOk) f |= F_IMU_FALLA;
  if (showPaso >= 0) f |= F_SHOW;
  return f;
}

void enviarTelemetria() {
  Serial.printf("E %.0f %.0f %.0f %d %d %.1f %.1f %.2f %lu %.0f %.0f %.0f\n", rpmIzq, rpmDer, rpmTal, distIzq,
                distDer, pitch, roll, vbat, (unsigned long)banderas(), salIzq, salDer, salTal);
}

// ------------------------------------- LUCES -----------------------------------------
uint32_t colorEscala(uint8_t r, uint8_t g, uint8_t b, float k) {
  k = fmaxf(0.0f, fminf(1.0f, k));
  return tira.Color((uint8_t)(r * k), (uint8_t)(g * k), (uint8_t)(b * k));
}

void actualizarLuces() {
  static float fase = 0;
  uint32_t t = millis();
  bool parpadeo = (t / 150) % 2;
  tira.clear();
  if (emergencia) {
    for (int i = 0; i < NUM_LEDS; i++) tira.setPixelColor(i, parpadeo ? tira.Color(255, 0, 0) : 0);
  } else if (inclinado) {
    for (int i = 0; i < NUM_LEDS; i++) tira.setPixelColor(i, parpadeo ? tira.Color(160, 0, 255) : 0);
  } else if (batCritica || taladroTrabado) {
    for (int i = 0; i < NUM_LEDS; i++) tira.setPixelColor(i, parpadeo ? tira.Color(255, 90, 0) : 0);
  } else if (fabsf(salTal) > 5) {
    // dos cometas en sentidos opuestos, como el taladro contrarrotante
    fase += salTal * 0.004f;
    for (int k = 0; k < 6; k++) {
      int a = ((int)floorf(fase) - k) % NUM_LEDS; if (a < 0) a += NUM_LEDS;
      int b = (NUM_LEDS - 1 - a + NUM_LEDS / 2) % NUM_LEDS;
      tira.setPixelColor(a, colorEscala(255, 90, 30, 1.0f - k / 6.0f));
      uint32_t prev = tira.getPixelColor(b);
      tira.setPixelColor(b, prev | colorEscala(60, 140, 255, 1.0f - k / 6.0f));
    }
  } else {
    float resp = 0.5f + 0.5f * sinf(t / 600.0f);
    int m = modoManual ? 99 : modoLuces;
    for (int i = 0; i < NUM_LEDS; i++) {
      uint32_t c = 0;
      switch (m) {
        case 0: c = 0; break;
        case 1: c = colorEscala(0, 200, 255, 0.15f + 0.6f * resp); break;               // respirar
        case 2: {                                                                       // buscando
          int pos = (t / 40) % NUM_LEDS;
          int dd = abs(i - pos); dd = min(dd, NUM_LEDS - dd);
          c = colorEscala(0, 80, 255, dd < 4 ? 1.0f - dd / 4.0f : 0.0f);
        } break;
        case 3: c = tira.Color(0, 255, 60); break;                                     // objetivo
        case 4: c = tira.gamma32(tira.ColorHSV((uint16_t)(i * 65536L / NUM_LEDS + t * 40))); break;
        case 5: c = tira.Color(255, 255, 255); break;                                  // faros
        default: c = colorEscala(40, 255, 40, 0.2f + 0.5f * resp); break;               // manual
      }
      tira.setPixelColor(i, c);
    }
    if (obstaculo && parpadeo) for (int i = 0; i < NUM_LEDS; i += 2) tira.setPixelColor(i, tira.Color(255, 0, 0));
  }
  tira.show();
}

// ------------------------------------- SETUP -----------------------------------------
void setup() {
  // drivers apagados mientras arranca todo
  pinMode(IZQ_EN, OUTPUT); pinMode(DER_EN, OUTPUT); pinMode(TAL_EN, OUTPUT);
  habilitarDrivers(false);
  for (int i = 0; i < 6; i++) { pwmIniciar(PINES_PWM[i]); pwmEscribir(PINES_PWM[i], 0); }
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_ESTADO, OUTPUT);
  pinMode(TOF_IZQ_XSHUT, OUTPUT); pinMode(TOF_DER_XSHUT, OUTPUT);
  digitalWrite(TOF_IZQ_XSHUT, LOW); digitalWrite(TOF_DER_XSHUT, LOW);
  // los pines 34-39 no tienen pull-up interno: usa resistencias de 10k a 3.3 V
  pinMode(ENC_IZQ_A, INPUT); pinMode(ENC_IZQ_B, INPUT);
  pinMode(ENC_DER_A, INPUT); pinMode(ENC_DER_B, INPUT);
  pinMode(ENC_TAL_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ_A), isrIzq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DER_A), isrDer, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_TAL_A), isrTal, RISING);

  Serial.begin(115200);
  SerialBT.begin("TOPO-S14");
  tira.begin();
  tira.setBrightness(BRILLO);
  tira.clear(); tira.show();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  tofIzqOk = iniciarToF(tofIzq, TOF_IZQ_XSHUT, 0x30);
  tofDerOk = iniciarToF(tofDer, TOF_DER_XSHUT, 0x31);
  imuOk = iniciarIMU();

  for (int i = 0; i < 20; i++) actualizarBateria();
  habilitarDrivers(true);
  imprimirInfo(Serial);
  pitar(2);
}

// ------------------------------------- LOOP ------------------------------------------
void loop() {
  static uint32_t tCtrl = 0, tEnc = 0, tBat = 0, tTel = 0, tLuz = 0, tImu = 0;
  uint32_t ahora = millis();

  leerUSB();
  leerBT();
  leerToF(tofIzq, tofIzqOk, distIzq);
  leerToF(tofDer, tofDerOk, distDer);

  if (ahora - tImu >= 10) { actualizarIMU((ahora - tImu) / 1000.0f); tImu = ahora; }
  if (ahora - tCtrl >= 10) { controlar((ahora - tCtrl) / 1000.0f); tCtrl = ahora; }
  if (ahora - tEnc >= 100) { actualizarEncoders((ahora - tEnc) / 1000.0f); tEnc = ahora; }
  if (ahora - tBat >= 100) { actualizarBateria(); tBat = ahora; }
  if (ahora - tTel >= 50) { enviarTelemetria(); tTel = ahora; }
  if (ahora - tLuz >= 33) { actualizarLuces(); tLuz = ahora; }
  actualizarBuzzer();
  digitalWrite(LED_ESTADO, (ahora / (modoManual ? 150 : 600)) % 2);
}
