// ============================================================
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// Arduino Mega - Receptor MAVLink ATTITUDE -> traductor GM3
// ============================================================
// PC (oculusmonitor) -> Arduino por USB, MAVLink v2 ATTITUDE
// Arduino -> Gimbal CADDX GM3 por Serial2, protocolo propietario
// ============================================================
//
// PROTOCOLO GM3 RECONSTRUIDO EMPIRICAMENTE (v5)
// ---------------------------------------------
// Trama de 10 bytes:
//
//   A5 5A 03 [RollL][RollH] [TiltL][TiltH] [Pan] [CRC_H][CRC_L]
//    b0 b1 b2   b3     b4      b5     b6     b7     b8     b9
//
//   Roll -> b3-b4, int16 little-endian, 182.04 u/grado  (16 bits: 65536/360)
//   Tilt -> b5-b6, int16 little-endian,  11.38 u/grado  (12 bits:  4096/360)
//   Pan  -> b7,    int8,                  0.711 u/grado ( 8 bits:   256/360)
//
//   CRC16-CCITT (poli 0x1021, init 0x0000) sobre b0..b7, transmitido big-endian.
//
// Cada eje usa una codificacion angular binaria de distinta profundidad.
// El signo del tilt esta invertido respecto de MAVLink: en el gimbal,
// valores positivos bajan la camara; en MAVLink, pitch positivo es
// "nariz arriba". De ahi el SIGN_TILT = -1.
// ============================================================

extern "C" {
#include "mavlink/common/mavlink.h"
}

// ---------------------------
// Puertos
// ---------------------------
#define PC_SERIAL     Serial
#define GIMBAL_SERIAL Serial2
constexpr uint32_t PC_BAUD     = 115200;
constexpr uint32_t GIMBAL_BAUD = 115200;

// ---------------------------
// Escalas (unidades por grado)
// ---------------------------
constexpr float ROLL_UNITS_PER_DEG = 182.04f;   // 65536 / 360
constexpr float TILT_UNITS_PER_DEG =  11.38f;   //  4096 / 360
constexpr float PAN_UNITS_PER_DEG  =   0.711f;  //   256 / 360

// ---------------------------
// Signos respecto de MAVLink
// ---------------------------
constexpr float SIGN_ROLL = +1.0f;
constexpr float SIGN_TILT = +1.0f;   // era -1.0f
constexpr float SIGN_PAN  = +1.0f;

// ---------------------------
// Limites fisicos del GM3 (manual del fabricante)
// ---------------------------
constexpr float ROLL_MIN_DEG =  -60.0f;
constexpr float ROLL_MAX_DEG =   60.0f;
constexpr float TILT_MIN_DEG = -120.0f;
constexpr float TILT_MAX_DEG =  120.0f;
constexpr float PAN_MIN_DEG  = -160.0f;
constexpr float PAN_MAX_DEG  =  160.0f;

// ---------------------------
// Limitador de tasa de envio al gimbal
// ---------------------------
constexpr uint32_t SEND_INTERVAL_MS = 20;   // 50 Hz
uint32_t lastSend = 0;

// Ultima actitud recibida
float lastRollDeg  = 0.0f;
float lastPitchDeg = 0.0f;
float lastYawDeg   = 0.0f;
bool  haveAttitude = false;

// ---------------------------
// MAVLink parser
// ---------------------------
mavlink_message_t mavMsg;
mavlink_status_t  mavStatus;

// ---------------------------
// Utilidades
// ---------------------------
float radToDeg(float rad) {
  return rad * 180.0f / 3.14159265359f;
}

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int16_t degreesToInt16(float deg, float minDeg, float maxDeg,
                       float unitsPerDeg, float sign) {
  deg = clampFloat(deg, minDeg, maxDeg);
  float raw = sign * deg * unitsPerDeg;
  raw = clampFloat(raw, -32768.0f, 32767.0f);
  return (int16_t)lroundf(raw);
}

int8_t degreesToInt8(float deg, float minDeg, float maxDeg,
                     float unitsPerDeg, float sign) {
  deg = clampFloat(deg, minDeg, maxDeg);
  float raw = sign * deg * unitsPerDeg;
  raw = clampFloat(raw, -128.0f, 127.0f);
  return (int8_t)lroundf(raw);
}

// ---------------------------
// CRC16-CCITT (0x1021, init 0x0000)
// ---------------------------
uint16_t crc16_ccitt(const uint8_t* data, size_t length) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc <<= 1;
    }
  }
  return crc;
}

// ---------------------------
// Trama GM3
// ---------------------------
void sendGimbalFrame(int16_t rollRaw, int16_t tiltRaw, int8_t panRaw) {
  uint8_t f[10];
  f[0] = 0xA5;
  f[1] = 0x5A;
  f[2] = 0x03;

  f[3] = (uint8_t)(rollRaw & 0xFF);          // Roll low
  f[4] = (uint8_t)((rollRaw >> 8) & 0xFF);   // Roll high

  f[5] = (uint8_t)(tiltRaw & 0xFF);          // Tilt low
  f[6] = (uint8_t)((tiltRaw >> 8) & 0xFF);   // Tilt high

  f[7] = (uint8_t)panRaw;                    // Pan (int8)

  uint16_t crc = crc16_ccitt(f, 8);
  f[8] = (uint8_t)((crc >> 8) & 0xFF);       // CRC high
  f[9] = (uint8_t)(crc & 0xFF);              // CRC low

  GIMBAL_SERIAL.write(f, 10);
}

// ---------------------------
// ATTITUDE trae roll, pitch, yaw en radianes
// ---------------------------
void handleAttitude(const mavlink_attitude_t& att) {
  lastRollDeg  = radToDeg(att.roll);
  lastPitchDeg = radToDeg(att.pitch);
  lastYawDeg   = radToDeg(att.yaw);
  haveAttitude = true;
}

void pushToGimbal() {
  if (!haveAttitude) return;

  int16_t rollRaw = degreesToInt16(lastRollDeg,  ROLL_MIN_DEG, ROLL_MAX_DEG,
                                   ROLL_UNITS_PER_DEG, SIGN_ROLL);
  int16_t tiltRaw = degreesToInt16(lastPitchDeg, TILT_MIN_DEG, TILT_MAX_DEG,
                                   TILT_UNITS_PER_DEG, SIGN_TILT);
  int8_t  panRaw  = degreesToInt8 (lastYawDeg,   PAN_MIN_DEG,  PAN_MAX_DEG,
                                   PAN_UNITS_PER_DEG,  SIGN_PAN);

  sendGimbalFrame(rollRaw, tiltRaw, panRaw);

  // Debug por USB (lo lee el panel de oculusmonitor)
  PC_SERIAL.print("ROLL=");
  PC_SERIAL.print(lastRollDeg, 1);
  PC_SERIAL.print("  PITCH=");
  PC_SERIAL.print(lastPitchDeg, 1);
  PC_SERIAL.print("  YAW=");
  PC_SERIAL.print(lastYawDeg, 1);
  PC_SERIAL.print("  | RollRaw=");
  PC_SERIAL.print(rollRaw);
  PC_SERIAL.print("  TiltRaw=");
  PC_SERIAL.print(tiltRaw);
  PC_SERIAL.print("  PanRaw=");
  PC_SERIAL.println(panRaw);
}

void setup() {
  PC_SERIAL.begin(PC_BAUD);
  GIMBAL_SERIAL.begin(GIMBAL_BAUD);
  PC_SERIAL.println("GM3 MAVLink receiver ready - v5 protocolo corregido");
}

void loop() {
  // Parseo de MAVLink desde la PC
  while (PC_SERIAL.available() > 0) {
    uint8_t c = (uint8_t)PC_SERIAL.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &mavMsg, &mavStatus)) {
      if (mavMsg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(&mavMsg, &att);
        handleAttitude(att);
      }
    }
  }

  // Envio al gimbal a tasa fija (desacoplado de la tasa de llegada)
  uint32_t now = millis();
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;
    pushToGimbal();
  }
}