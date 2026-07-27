// ============================================================================
// gm3_protocol_validation.ino
// ----------------------------------------------------------------------------
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// ETAPA: Validacion final (vease seccion 6.3.8)
//
// Banco de ensayos que verifica la reconstruccion definitiva del protocolo
// mediante aislamiento de la variable independiente y controles experimentales.
//
// PROTOCOLO RECONSTRUIDO (validado):
//   A5 5A 03 [RollL][RollH] [TiltL][TiltH] [Pan] [CRC_H][CRC_L]
//    b0 b1 b2   b3     b4      b5     b6     b7     b8     b9
//   Roll -> b3-b4, int16 LE, 182,04 u/grado (16 bits)
//   Tilt -> b5-b6, int16 LE,  11,38 u/grado (12 bits)
//   Pan  -> b7,    int8,       0,711 u/grado ( 8 bits)
//   CRC16-CCITT (0x1021, init 0x0000) sobre b0..b7, big-endian.
//
// DISENO EXPERIMENTAL
//   - Aislamiento: cada ensayo transmite un valor conocido en un unico eje,
//     manteniendo los demas en cero. Se prescinde del visor de realidad virtual
//     para evitar la correlacion entre ejes de una senal humana.
//   - Control positivo: un comando de eje previamente verificado como funcional,
//     que confirma la integridad del cableado, el baudrate y el CRC.
//   - Control negativo: reproduce el layout defectuoso conocido, confirmando
//     que el fallo es reproducible.
//
// USO
//   Monitor Serie a 115200. Enviar el numero del ensayo. Volver a 0 (neutro)
//   entre ensayo y ensayo. Mano en la alimentacion del gimbal.
// ============================================================================

#define PC_SERIAL     Serial
#define GIMBAL_SERIAL Serial2
constexpr uint32_t PC_BAUD     = 115200;
constexpr uint32_t GIMBAL_BAUD = 115200;
constexpr uint32_t SEND_INTERVAL_MS = 50;   // 20 Hz

// Escalas validadas (unidades por grado)
constexpr float ROLL_UNITS_PER_DEG = 182.04f;
constexpr float TILT_UNITS_PER_DEG =  11.38f;
constexpr float PAN_UNITS_PER_DEG  =   0.711f;

int      currentTest = 0;
uint32_t lastSend    = 0;

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

// Trama unica de 10 bytes con los tres ejes
void sendFrame(int16_t rollRaw, int16_t tiltRaw, int8_t panRaw, bool verbose) {
  uint8_t f[10];
  f[0] = 0xA5;
  f[1] = 0x5A;
  f[2] = 0x03;
  f[3] = (uint8_t)(rollRaw & 0xFF);
  f[4] = (uint8_t)((rollRaw >> 8) & 0xFF);
  f[5] = (uint8_t)(tiltRaw & 0xFF);
  f[6] = (uint8_t)((tiltRaw >> 8) & 0xFF);
  f[7] = (uint8_t)panRaw;
  uint16_t crc = crc16_ccitt(f, 8);
  f[8] = (uint8_t)((crc >> 8) & 0xFF);
  f[9] = (uint8_t)(crc & 0xFF);
  GIMBAL_SERIAL.write(f, 10);

  if (verbose) {
    PC_SERIAL.print("  TX: ");
    for (uint8_t i = 0; i < 10; ++i) {
      if (f[i] < 0x10) PC_SERIAL.print('0');
      PC_SERIAL.print(f[i], HEX);
      PC_SERIAL.print(' ');
    }
    PC_SERIAL.println();
  }
}

int16_t deg16(float deg, float upd) { return (int16_t)lroundf(deg * upd); }
int8_t  deg8 (float deg, float upd) { return (int8_t) lroundf(deg * upd); }

void sendCurrentTest(bool verbose) {
  switch (currentTest) {

    case 0:  // NEUTRO - todo cero
      sendFrame(0, 0, 0, verbose);
      break;

    case 1:  // AISLAMIENTO Roll +30
      sendFrame(deg16(30.0f, ROLL_UNITS_PER_DEG), 0, 0, verbose);
      break;

    case 2:  // AISLAMIENTO Tilt +30
      sendFrame(0, deg16(30.0f, TILT_UNITS_PER_DEG), 0, verbose);
      break;

    case 3:  // AISLAMIENTO Pan +30
      sendFrame(0, 0, deg8(30.0f, PAN_UNITS_PER_DEG), verbose);
      break;

    case 4:  // CONTROL POSITIVO - Pan +30 (comando funcional conocido)
      sendFrame(0, 0, deg8(30.0f, PAN_UNITS_PER_DEG), verbose);
      break;

    case 5:  // CONTROL NEGATIVO - layout defectuoso (Tilt en b4-b5)
      {
        // Reproduce el error de la primera aproximacion: Tilt en b4-b5
        int16_t tiltRaw = deg16(30.0f, 321.0f);   // escala vieja
        uint8_t f[10];
        f[0]=0xA5; f[1]=0x5A; f[2]=0x03; f[3]=0x00;
        f[4]=(uint8_t)(tiltRaw & 0xFF);
        f[5]=(uint8_t)((tiltRaw>>8)&0xFF);
        f[6]=0x00; f[7]=0x00;
        uint16_t crc=crc16_ccitt(f,8);
        f[8]=(uint8_t)((crc>>8)&0xFF); f[9]=(uint8_t)(crc&0xFF);
        GIMBAL_SERIAL.write(f,10);
        if (verbose) PC_SERIAL.println("  (control negativo: layout b4-b5)");
      }
      break;

    case 6:  // ENSAYO INTEGRADO - los tres ejes a +30
      sendFrame(deg16(30.0f, ROLL_UNITS_PER_DEG),
                deg16(30.0f, TILT_UNITS_PER_DEG),
                deg8 (30.0f, PAN_UNITS_PER_DEG), verbose);
      break;
  }
}

void printMenu() {
  PC_SERIAL.println();
  PC_SERIAL.println("=== gm3_protocol_validation - banco de ensayos ===");
  PC_SERIAL.println(" 0 = NEUTRO (volver aca entre ensayos)");
  PC_SERIAL.println(" 1 = aislamiento Roll +30");
  PC_SERIAL.println(" 2 = aislamiento Tilt +30");
  PC_SERIAL.println(" 3 = aislamiento Pan  +30");
  PC_SERIAL.println(" 4 = CONTROL POSITIVO (Pan +30 conocido)");
  PC_SERIAL.println(" 5 = CONTROL NEGATIVO (layout defectuoso b4-b5)");
  PC_SERIAL.println(" 6 = ensayo integrado (los tres ejes +30)");
  PC_SERIAL.println(" m = mostrar este menu");
  PC_SERIAL.print("Ensayo actual: ");
  PC_SERIAL.println(currentTest);
}

void setup() {
  PC_SERIAL.begin(PC_BAUD);
  GIMBAL_SERIAL.begin(GIMBAL_BAUD);
  delay(500);
  printMenu();
}

void loop() {
  while (PC_SERIAL.available() > 0) {
    char c = (char)PC_SERIAL.read();
    if (c >= '0' && c <= '6') {
      currentTest = c - '0';
      PC_SERIAL.print(">>> ENSAYO "); PC_SERIAL.println(currentTest);
      sendCurrentTest(true);
    } else if (c == 'm' || c == 'M') {
      printMenu();
    }
  }

  uint32_t now = millis();
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;
    sendCurrentTest(false);
  }
}
