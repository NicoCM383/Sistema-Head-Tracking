// ============================================================
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// Seccion 6.3 - Ingenieria Inversa
// Fuente: "Codigos para la ingenieria inversa.docx"
// Extraccion fiel (sin refactor).
// Puente transparente Pixhawk <-> Gimbal + captura estructurada CSV por USB.
// ============================================================
// Arduino Mega 2560
// Serial  = USB hacia netbook
// Serial1 = Pixhawk  (RX1=19, TX1=18)
// Serial2 = Gimbal   (RX2=17, TX2=16)
//
// Formato CSV generado:
// t_ms,dir,len,b0,b1,b2,b3,b4,b5,b6,b7,b8,b9
//
// dir:
// < = Pixhawk -> Gimbal
// > = Gimbal  -> Pixhawk

const unsigned long BAUD_PC      = 230400;
const unsigned long BAUD_PIXHAWK = 115200;
const unsigned long BAUD_GIMBAL  = 115200;

const int FRAME_LEN = 10;

struct FrameParser {
  uint8_t buffer[FRAME_LEN];
  int state = 0;
  int index = 0;

  bool feed(uint8_t b) {
    if (state == 0) {
      if (b == 0xA5) {
        buffer[0] = b;
        state = 1;
      }
    }
    else if (state == 1) {
      if (b == 0x5A) {
        buffer[1] = b;
        index = 2;
        state = 2;
      } else {
        state = 0;
      }
    }
    else {
      buffer[index++] = b;

      if (index >= FRAME_LEN) {
        state = 0;
        return true;
      }
    }

    return false;
  }
};

FrameParser parserPixhawkToGimbal;
FrameParser parserGimbalToPixhawk;

void printByteHex(uint8_t b) {
  if (b < 16) Serial.print('0');
  Serial.print(b, HEX);
}

void printFrameCSV(char dir, const uint8_t *frame, int len) {
  Serial.print(millis());
  Serial.print(',');
  Serial.print(dir);
  Serial.print(',');
  Serial.print(len);

  for (int i = 0; i < len; i++) {
    Serial.print(',');
    printByteHex(frame[i]);
  }

  Serial.println();
}

void forwardAndLog(HardwareSerial &input,
                   HardwareSerial &output,
                   FrameParser &parser,
                   char dir) {
  while (input.available()) {
    uint8_t b = input.read();

    // Puente transparente
    output.write(b);
    // Captura estructurada
    if (parser.feed(b)) {
      printFrameCSV(dir, parser.buffer, FRAME_LEN);
    }
  }
}

void setup() {
  Serial.begin(BAUD_PC);
  Serial1.begin(BAUD_PIXHAWK);
  Serial2.begin(BAUD_GIMBAL);

  Serial.println("t_ms,dir,len,b0,b1,b2,b3,b4,b5,b6,b7,b8,b9");
}

void loop() {
  forwardAndLog(Serial1, Serial2, parserPixhawkToGimbal, '<');
  forwardAndLog(Serial2, Serial1, parserGimbalToPixhawk, '>');
}
