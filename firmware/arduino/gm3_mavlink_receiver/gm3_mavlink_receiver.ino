// ============================================================
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// Seccion 6.6 - Desarrollo del programa en Arduino Mega
// Fuente: "Codigo Arduino Protocolo MavLink y Protocolo propietario.docx"
// Extraccion fiel (sin refactor). Programa final del prototipo.
// ============================================================
// Arduino Mega - Receptor MAVLink ATTITUDE + traductor GM3
// PC -> Arduino por USB (Serial)
// Arduino -> Gimbal por Serial2
// ============================================================
extern "C" {
  #include <mavlink/common/mavlink.h>
}
// ---------------------------
// Puertos
// ---------------------------
#define PC_SERIAL     Serial
#define GIMBAL_SERIAL Serial2
constexpr uint32_t PC_BAUD     = 115200;
constexpr uint32_t GIMBAL_BAUD = 115200;
// ---------------------------
// Calibración obtenida de tus logs
// ---------------------------
constexpr float TILT_UNITS_PER_DEG = 2900.0f;
constexpr float PAN_UNITS_PER_DEG  = 182.0f;
// Si más adelante detectás offset distinto de cero, ajustalo aquí.
constexpr int16_t TILT_ZERO_RAW = 0;
constexpr int16_t PAN_ZERO_RAW  = 0;
// ---------------------------
// Límites útiles
// ---------------------------
constexpr float TILT_MIN_DEG = -90.0f;
constexpr float TILT_MAX_DEG =  90.0f;
constexpr float PAN_MIN_DEG  = -180.0f;
constexpr float PAN_MAX_DEG  =  180.0f;
constexpr float ROLL_MIN_DEG = -30.0f;
constexpr float ROLL_MAX_DEG =  30.0f;
// ---------------------------
// LUT de Roll identificada
// Roll -> b3-b4
// ---------------------------
struct RollLutEntry {
  float deg;
  uint8_t b3;
  uint8_t b4;
};
const RollLutEntry ROLL_LUT[] = {
  { -30.0f, 0xA0, 0xEA },
  { -10.0f, 0xE0, 0xF8 },
  {   0.0f, 0x00, 0x00 },
  {  10.0f, 0x10, 0x07 },
  {  30.0f, 0x50, 0x15 }
};
constexpr size_t ROLL_LUT_SIZE = sizeof(ROLL_LUT) / sizeof(ROLL_LUT[0]);
// ---------------------------
// MAVLink parser
// ---------------------------
mavlink_message_t mavMsg;
mavlink_status_t mavStatus;
// ---------------------------
// Utilidades
// ---------------------------
float radToDeg(float rad) {
  return rad * 180.0f / 3.14159265359f;
}
float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}
int16_t degreesToTiltRaw(float tiltDeg) {
  tiltDeg = clampFloat(tiltDeg, TILT_MIN_DEG, TILT_MAX_DEG);
  float raw = TILT_ZERO_RAW + tiltDeg * TILT_UNITS_PER_DEG;
  return static_cast<int16_t>(raw);
}
int16_t degreesToPanRaw(float panDeg) {
  panDeg = clampFloat(panDeg, PAN_MIN_DEG, PAN_MAX_DEG);
  float raw = PAN_ZERO_RAW + panDeg * PAN_UNITS_PER_DEG;
  return static_cast<int16_t>(raw);
}
// Selección por vecino más cercano dentro de la LUT.
// Si luego completás más puntos intermedios, esta parte mejora automáticamente.
void rollDegreesToBytes(float rollDeg, uint8_t &b3, uint8_t &b4) {
  rollDeg = clampFloat(rollDeg, ROLL_MIN_DEG, ROLL_MAX_DEG);
  size_t bestIndex = 0;
  float bestError = 1e9f;
  for (size_t i = 0; i < ROLL_LUT_SIZE; ++i) {
    float err = fabs(rollDeg - ROLL_LUT[i].deg);
    if (err < bestError) {
      bestError = err;
      bestIndex = i;
    }
  }
  b3 = ROLL_LUT[bestIndex].b3;
  b4 = ROLL_LUT[bestIndex].b4;
}
// ---------------------------
// CRC16-CCITT
// Polinomio 0x1021
// Init 0x0000
// Sobre b0..b7
// ---------------------------
uint16_t crc16_ccitt(const uint8_t* data, size_t length) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}
// ---------------------------
// Trama GM3: modo Tilt/Pan
// [A5][5A][03][00][TL][TH][PL][PH][CRC_H][CRC_L]
// ---------------------------
void sendTiltPanFrame(int16_t tiltRaw, int16_t panRaw) {
  uint8_t frame[10];
  frame[0] = 0xA5;
  frame[1] = 0x5A;
  frame[2] = 0x03;
  frame[3] = 0x00;
  // Tilt -> b4-b5 little-endian
  frame[4] = (uint8_t)(tiltRaw & 0xFF);
  frame[5] = (uint8_t)((tiltRaw >> 8) & 0xFF);
  // Pan -> b6-b7 little-endian
  frame[6] = (uint8_t)(panRaw & 0xFF);
  frame[7] = (uint8_t)((panRaw >> 8) & 0xFF);
  uint16_t crc = crc16_ccitt(frame, 8);
  frame[8] = (uint8_t)((crc >> 8) & 0xFF); // CRC_H
  frame[9] = (uint8_t)(crc & 0xFF);        // CRC_L
  GIMBAL_SERIAL.write(frame, sizeof(frame));
}
// ---------------------------
// Trama GM3: modo Roll
// [A5][5A][03][R1][R0][00][00][00][CRC_H][CRC_L]
// ---------------------------
void sendRollFrame(uint8_t r1, uint8_t r0) {
  uint8_t frame[10];
  frame[0] = 0xA5;
  frame[1] = 0x5A;
  frame[2] = 0x03;
  frame[3] = r1;
  frame[4] = r0;
  frame[5] = 0x00;
  frame[6] = 0x00;
  frame[7] = 0x00;
  uint16_t crc = crc16_ccitt(frame, 8);
  frame[8] = (uint8_t)((crc >> 8) & 0xFF); // CRC_H
  frame[9] = (uint8_t)(crc & 0xFF);        // CRC_L
  GIMBAL_SERIAL.write(frame, sizeof(frame));
}
// ---------------------------
// Procesamiento del mensaje ATTITUDE
// ATTITUDE trae roll, pitch, yaw en radianes
// ---------------------------
void handleAttitude(const mavlink_attitude_t& att) {
  float rollDeg  = radToDeg(att.roll);
  float pitchDeg = radToDeg(att.pitch);
  float yawDeg   = radToDeg(att.yaw);
  int16_t tiltRaw = degreesToTiltRaw(pitchDeg);
  int16_t panRaw  = degreesToPanRaw(yawDeg);
  uint8_t rollB3 = 0x00;
  uint8_t rollB4 = 0x00;
  rollDegreesToBytes(rollDeg, rollB3, rollB4);
  // El protocolo reconstruido mostró dos modos funcionales:
  // 1) Tilt/Pan conjunto
  // 2) Roll por LUT
  sendTiltPanFrame(tiltRaw, panRaw);
  sendRollFrame(rollB3, rollB4);
  // Debug opcional por USB
  PC_SERIAL.print("ROLL=");
  PC_SERIAL.print(rollDeg, 2);
  PC_SERIAL.print("  PITCH=");
  PC_SERIAL.print(pitchDeg, 2);
  PC_SERIAL.print("  YAW=");
  PC_SERIAL.print(yawDeg, 2);
  PC_SERIAL.print("  | TiltRaw=");
  PC_SERIAL.print(tiltRaw);
  PC_SERIAL.print("  PanRaw=");
  PC_SERIAL.print(panRaw);
  PC_SERIAL.print("  RollBytes=");
  PC_SERIAL.print(rollB3, HEX);
  PC_SERIAL.print(" ");
  PC_SERIAL.println(rollB4, HEX);
}
void setup() {
  PC_SERIAL.begin(PC_BAUD);
  GIMBAL_SERIAL.begin(GIMBAL_BAUD);
  PC_SERIAL.println("GM3 MAVLink receiver ready");
}
void loop() {
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
}
