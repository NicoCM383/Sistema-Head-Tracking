// ============================================================
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// Seccion 6.3 - Ingenieria Inversa (validacion del protocolo reconstruido)
// Fuente: "Codigos para la ingenieria inversa.docx"
// Extraccion fiel (sin refactor).
// ============================================================
// Arduino Mega 2560
// Serial2 = UART hacia Gimbal GM3
//
// Validacion de protocolo reconstruido:
// [A5][5A][03][B3][B4][B5][B6][B7][CRC_H][CRC_L]
//
// ------------------------------------------------------------
// FACTOR DE ESCALA DE TILT:
// 321.0f es el factor de calibracion de Tilt usado por este sketch
// de validacion (ver tiltDegToRaw). El firmware final de runtime
// utiliza el mismo factor de calibracion de Tilt, en:
//   firmware/arduino/gm3_mavlink_receiver/gm3_mavlink_receiver.ino
// ------------------------------------------------------------
#include <Arduino.h>

uint16_t crc16_ccitt_0(const uint8_t *data, size_t len) {
  uint16_t crc = 0x0000;

  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;

    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

void sendFrameToGimbal(uint8_t b3,
                       uint8_t b4,
                       uint8_t b5,
                       uint8_t b6,
                       uint8_t b7) {
  uint8_t frame[10];

  frame[0] = 0xA5;
  frame[1] = 0x5A;
  frame[2] = 0x03;
  frame[3] = b3;
  frame[4] = b4;
  frame[5] = b5;
  frame[6] = b6;
  frame[7] = b7;

  uint16_t crc = crc16_ccitt_0(frame, 8);

  frame[8] = (crc >> 8) & 0xFF;
  frame[9] = crc & 0xFF;

  Serial2.write(frame, 10);
}

int16_t tiltDegToRaw(float tilt_deg) {
  if (tilt_deg > 90.0f) tilt_deg = 90.0f;
  if (tilt_deg < -90.0f) tilt_deg = -90.0f;

  // 321.0f: factor de calibracion de Tilt (mismo factor que el firmware final).
  return (int16_t)lroundf(tilt_deg * 321.0f);
}

int16_t panDegToRaw(float pan_deg) {
  if (pan_deg > 170.0f) pan_deg = 170.0f;
  if (pan_deg < -170.0f) pan_deg = -170.0f;

  int32_t raw = (int32_t)lroundf(pan_deg * 182.07f);

  // Cuantizacion observada en Pan
  raw = (int32_t)lroundf(raw / 16.0f) * 16;
  if (raw > 30944) raw = 30944;
  if (raw < -30960) raw = -30960;

  return (int16_t)raw;
}

void sendTiltPanDeg(float tilt_deg, float pan_deg) {
  int16_t tiltRaw = tiltDegToRaw(tilt_deg);
  int16_t panRaw  = panDegToRaw(pan_deg);

  uint8_t TL = tiltRaw & 0xFF;
  uint8_t TH = (tiltRaw >> 8) & 0xFF;

  uint8_t PL = panRaw & 0xFF;
  uint8_t PH = (panRaw >> 8) & 0xFF;

  sendFrameToGimbal(0x00, TL, TH, PL, PH);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  Serial.println("Validacion control directo GM3 iniciada");
}

void loop() {
  static uint32_t last = 0;

  if (millis() - last >= 20) {
    last = millis();

    // Prueba de validacion
    float tilt = 20.0f;
    float pan  = -45.0f;

    sendTiltPanDeg(tilt, pan);
  }
}
