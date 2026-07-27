// ============================================================================
// gm3_bridge_csv_logger.ino
// ----------------------------------------------------------------------------
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// ETAPA: Captura
//
// Puente transparente entre la controladora de vuelo Pixhawk y el gimbal
// Caddx GM3. Reenvia la comunicacion serial en ambos sentidos y, ademas,
// imprime por USB cada trama observada en formato CSV para su posterior
// analisis. La escritura del archivo la realiza PuTTY del lado de la netbook.
//
// Formato de salida CSV (una linea por trama):
//     <tiempo_ms>,<dir>,<len>,<b0> <b1> ... <bN>
//
//   dir = '<'  trama con destino al gimbal   (Pixhawk -> gimbal)
//   dir = '>'  trama con destino a la Pixhawk (gimbal -> Pixhawk)
//
// CONEXIONES
//   Serial  (USB)  -> netbook (captura con PuTTY)
//   Serial1        -> Pixhawk
//   Serial2        -> gimbal GM3
//
// El encabezado A5 5A se utiliza para delimitar el inicio de cada trama
// con destino al gimbal.
// ============================================================================

#define PC_SERIAL      Serial
#define PIXHAWK_SERIAL Serial1
#define GIMBAL_SERIAL  Serial2

constexpr uint32_t PC_BAUD      = 115200;
constexpr uint32_t PIXHAWK_BAUD = 115200;
constexpr uint32_t GIMBAL_BAUD  = 115200;

constexpr uint8_t  FRAME_LEN    = 10;     // longitud fija observada
constexpr uint8_t  HDR0         = 0xA5;
constexpr uint8_t  HDR1         = 0x5A;

// Buffer de ensamblado de la trama con destino al gimbal
uint8_t frameBuf[FRAME_LEN];
uint8_t frameIdx = 0;
bool    inFrame  = false;

uint32_t startMs = 0;

void printFrameCSV(char dir, const uint8_t* buf, uint8_t len) {
  PC_SERIAL.print(millis() - startMs);
  PC_SERIAL.print(',');
  PC_SERIAL.print(dir);
  PC_SERIAL.print(',');
  PC_SERIAL.print(len);
  PC_SERIAL.print(',');
  for (uint8_t i = 0; i < len; ++i) {
    if (buf[i] < 0x10) PC_SERIAL.print('0');
    PC_SERIAL.print(buf[i], HEX);
    if (i < len - 1) PC_SERIAL.print(' ');
  }
  PC_SERIAL.println();
}

void setup() {
  PC_SERIAL.begin(PC_BAUD);
  PIXHAWK_SERIAL.begin(PIXHAWK_BAUD);
  GIMBAL_SERIAL.begin(GIMBAL_BAUD);
  startMs = millis();
  PC_SERIAL.println("# gm3_bridge_csv_logger - captura iniciada");
  PC_SERIAL.println("# formato: tiempo_ms,dir,len,bytes");
}

void loop() {
  // ---- Pixhawk -> gimbal (con deteccion y registro de tramas) ----
  while (PIXHAWK_SERIAL.available() > 0) {
    uint8_t b = (uint8_t)PIXHAWK_SERIAL.read();

    // Reenvio transparente inmediato hacia el gimbal
    GIMBAL_SERIAL.write(b);

    // Ensamblado en paralelo para el registro
    if (!inFrame) {
      // Buscar el encabezado A5 5A
      if (frameIdx == 0 && b == HDR0) {
        frameBuf[frameIdx++] = b;
      } else if (frameIdx == 1 && b == HDR1) {
        frameBuf[frameIdx++] = b;
        inFrame = true;
      } else {
        frameIdx = 0;
      }
    } else {
      frameBuf[frameIdx++] = b;
      if (frameIdx >= FRAME_LEN) {
        printFrameCSV('<', frameBuf, FRAME_LEN);
        frameIdx = 0;
        inFrame  = false;
      }
    }
  }

  // ---- gimbal -> Pixhawk (reenvio transparente) ----
  while (GIMBAL_SERIAL.available() > 0) {
    uint8_t b = (uint8_t)GIMBAL_SERIAL.read();
    PIXHAWK_SERIAL.write(b);
  }
}
