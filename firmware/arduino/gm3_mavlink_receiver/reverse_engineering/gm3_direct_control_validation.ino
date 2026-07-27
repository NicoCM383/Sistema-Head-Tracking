// ============================================================================
// gm3_direct_control_validation.ino
// ----------------------------------------------------------------------------
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// ETAPA: Primera validacion (vease seccion 6.3.4)
//
// Genera tramas propias con la estructura reconstruida en la primera
// aproximacion, calcula el CRC correspondiente y las envia directamente al
// gimbal, prescindiendo de Mission Planner y de la Pixhawk.
//
// ALCANCE DE ESTA VALIDACION (limitacion documentada en 6.3.4):
// Este programa confirma unicamente que el gimbal ACEPTA las tramas generadas
// -es decir, que el encabezado, la longitud y el CRC son correctos- y que el
// dispositivo responde fisicamente a ellas. NO verifica que la respuesta sea
// la CORRECTA: el actuador se mueve, pero no necesariamente en el eje ni en la
// magnitud comandados. Esta insuficiencia permitio que una interpretacion
// erronea de los campos permaneciera sin detectar.
//
// La validacion definitiva, con aislamiento de variable y controles positivo
// y negativo, se implementa en gm3_protocol_validation.ino.
//
// CONEXIONES
//   Serial2 -> gimbal GM3  (pin 16 TX2 -> RX gimbal, pin 17 RX2 -> TX gimbal,
//                           GND comun obligatorio)
// ============================================================================

#define PC_SERIAL     Serial
#define GIMBAL_SERIAL Serial2
constexpr uint32_t PC_BAUD     = 115200;
constexpr uint32_t GIMBAL_BAUD = 115200;
constexpr uint32_t SEND_INTERVAL_MS = 50;   // 20 Hz

// --- Escalas segun la reconstruccion de la PRIMERA aproximacion ---
// (offsets y factores posteriormente corregidos)
constexpr float TILT_UNITS_PER_DEG = 321.0f;   // estimacion de la etapa
constexpr float PAN_UNITS_PER_DEG  = 182.0f;   // estimacion de la etapa

uint32_t lastSend = 0;
float testAngle = 0.0f;
float dir = 1.0f;

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

// Trama segun la hipotesis de la primera aproximacion:
// [A5][5A][03][00][TiltL][TiltH][PanL][PanH][CRC_H][CRC_L]
void sendFramePrimeraAprox(int16_t tiltRaw, int16_t panRaw) {
  uint8_t f[10];
  f[0] = 0xA5;
  f[1] = 0x5A;
  f[2] = 0x03;
  f[3] = 0x00;
  f[4] = (uint8_t)(tiltRaw & 0xFF);         // Tilt en b4-b5 (hipotesis vieja)
  f[5] = (uint8_t)((tiltRaw >> 8) & 0xFF);
  f[6] = (uint8_t)(panRaw & 0xFF);          // Pan en b6-b7 (hipotesis vieja)
  f[7] = (uint8_t)((panRaw >> 8) & 0xFF);
  uint16_t crc = crc16_ccitt(f, 8);
  f[8] = (uint8_t)((crc >> 8) & 0xFF);
  f[9] = (uint8_t)(crc & 0xFF);
  GIMBAL_SERIAL.write(f, 10);
}

void setup() {
  PC_SERIAL.begin(PC_BAUD);
  GIMBAL_SERIAL.begin(GIMBAL_BAUD);
  PC_SERIAL.println("gm3_direct_control_validation - primera validacion");
  PC_SERIAL.println("Envia tramas propias con la estructura reconstruida.");
}

void loop() {
  uint32_t now = millis();
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

    // Barrido lento de un angulo de prueba
    testAngle += dir * 1.0f;
    if (testAngle > 45.0f)  dir = -1.0f;
    if (testAngle < -45.0f) dir = 1.0f;

    int16_t tiltRaw = (int16_t)(testAngle * TILT_UNITS_PER_DEG);
    int16_t panRaw  = (int16_t)(testAngle * PAN_UNITS_PER_DEG);

    sendFramePrimeraAprox(tiltRaw, panRaw);

    PC_SERIAL.print("angulo="); PC_SERIAL.print(testAngle, 1);
    PC_SERIAL.print("  tiltRaw="); PC_SERIAL.print(tiltRaw);
    PC_SERIAL.print("  panRaw="); PC_SERIAL.println(panRaw);
  }
}
