// ===========================================================================
// AS5047P SPI bring-up diagnostic  (built only by [env:diag], -D ENCODER_DIAG)
//
// Purpose: turn the "angle pinned at all-ones" mystery into direct pin
// measurements. It bit-bangs the encoder and prints the RAW 16-bit word, and
// lets you statically drive each output pin so you can meter it -- both at the
// STM32 pad AND at the encoder pin (which also proves continuity + that the
// pin really toggles, e.g. whether PA15/JTDI works as GPIO).
//
// Wiring under test: SCK=PB6  MOSI=PB7  MISO=PB8  CS=PA15 (J3 pin 4)
// ===========================================================================
#ifdef ENCODER_DIAG

#include <Arduino.h>

// Mapping matches the confirmed wiring: CLK->PB6, MOSI->PB7, MISO->PB8, CSn->PA15.
#define PIN_SCK  PB6    // encoder CLK
#define PIN_MOSI PB7
#define PIN_MISO PB8
#define PIN_CS   PA15   // encoder CSn (J3 pin 4)

static bool autoread = true;

// VERY slow clock (~2 kHz) so it survives any RC noise-filter on the Hall pads.
static const int T = 250;  // us per phase

static uint16_t transfer16(uint16_t out) {
  uint16_t in = 0;
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(T);
  for (int i = 15; i >= 0; --i) {
    digitalWrite(PIN_MOSI, (out >> i) & 1);
    delayMicroseconds(T);
    digitalWrite(PIN_SCK, HIGH);   // rising edge: AS5047P updates MISO
    delayMicroseconds(T);
    digitalWrite(PIN_SCK, LOW);    // falling edge
    delayMicroseconds(T);          // let MISO settle through any filter
    in = (in << 1) | (digitalRead(PIN_MISO) & 1);
  }
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(T);
  return in;
}

static void menu() {
  Serial.println();
  Serial.println("=== AS5047P SPI DIAG ===");
  Serial.println("SCK=PB6  MOSI=PB7  MISO=PB8  CS=PA15(J3 pin4)");
  Serial.println("Auto-reads RAW word 2x/s. A real encoder changes RAW as you turn the shaft;");
  Serial.println("0xFFFF = MISO stuck high (not driven). Commands pause auto-read:");
  Serial.println("  c=CS LOW   C=CS HIGH    <- meter PA15 AND encoder CSn (should follow)");
  Serial.println("  k=SCK LOW  K=SCK HIGH   <- meter PB6 AND encoder CLK");
  Serial.println("  o=MOSI LOW O=MOSI HIGH  <- meter PB7 AND encoder MOSI");
  Serial.println("  i=read MISO(PB8) level   r=resume auto-read   ?=menu");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_SCK, LOW);
  digitalWrite(PIN_MOSI, LOW);
  delay(300);
  menu();
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    switch (ch) {
      case 'c': autoread = false; digitalWrite(PIN_CS, LOW);   Serial.println("CS=LOW  (PA15 + encoder CSn should read ~0V)"); break;
      case 'C': autoread = false; digitalWrite(PIN_CS, HIGH);  Serial.println("CS=HIGH (PA15 + encoder CSn should read ~3.3V)"); break;
      case 'k': autoread = false; digitalWrite(PIN_SCK, LOW);  Serial.println("SCK=LOW"); break;
      case 'K': autoread = false; digitalWrite(PIN_SCK, HIGH); Serial.println("SCK=HIGH (PB6 + encoder CLK ~3.3V)"); break;
      case 'o': autoread = false; digitalWrite(PIN_MOSI, LOW); Serial.println("MOSI=LOW"); break;
      case 'O': autoread = false; digitalWrite(PIN_MOSI, HIGH);Serial.println("MOSI=HIGH (PB7 + encoder MOSI ~3.3V)"); break;
      case 'i': Serial.print("MISO(PB8) reads "); Serial.println(digitalRead(PIN_MISO)); break;
      case 't': {  // self-test: can the MISO pin be read as a GPIO at all?
        autoread = false;
        pinMode(PIN_MISO, OUTPUT);
        digitalWrite(PIN_MISO, LOW);  delayMicroseconds(10); int lo = digitalRead(PIN_MISO);
        digitalWrite(PIN_MISO, HIGH); delayMicroseconds(10); int hi = digitalRead(PIN_MISO);
        pinMode(PIN_MISO, INPUT);
        Serial.print("PB8 self-test: drive LOW->read "); Serial.print(lo);
        Serial.print(", drive HIGH->read "); Serial.print(hi);
        Serial.println((lo==0 && hi==1) ? "  => PB8 GPIO OK" : "  => PB8 GPIO BROKEN");
        break;
      }
      case 'L': {  // loopback: jumper PB7(MOSI)->PB8(MISO), expect to read back the sent word
        autoread = false;
        uint16_t sent = 0xA55A;
        uint16_t got = transfer16(sent);
        char b[64];
        snprintf(b, sizeof(b), "LOOPBACK sent=0x%04X got=0x%04X %s",
                 sent, got, (got == sent) ? "=> MATCH (G431 SPI read path OK)" : "=> MISMATCH");
        Serial.println(b);
        break;
      }
      case 'r': autoread = true; Serial.println("auto-read resumed"); break;
      case '?': menu(); break;
      default: break;
    }
  }

  static uint32_t last = 0;
  if (autoread && (millis() - last) >= 500) {
    last = millis();
    uint16_t raw = transfer16(0xFFFF);
    char buf[48];
    uint16_t a14 = raw & 0x3FFF;
    snprintf(buf, sizeof(buf), "RAW=0x%04X  angle14=%u  deg=%.1f",
             raw, a14, a14 * 360.0f / 16384.0f);
    Serial.println(buf);
  }
}

#endif  // ENCODER_DIAG
