#ifndef AS5047P_SOFTSPI_H
#define AS5047P_SOFTSPI_H

#include <Arduino.h>
#include <SimpleFOC.h>

// ---------------------------------------------------------------------------
// AS5047P absolute magnetic encoder, read over BIT-BANGED (software) SPI.
//
// Why software SPI: on the B-G431B-ESC1 the only free GPIO are the Hall
// connector (PB6/PB7/PB8) + the spare throttle pin (PA15). None of those map
// to a single hardware SPI peripheral, and the hardware-SPI pins (PB3/PB4/PB5)
// collide with the USART2 link that the ST-LINK Virtual COM Port uses to talk
// to the PC. So we bit-bang. It's slow but more than fast enough for the
// AS5047P (14-bit absolute) at centrifuge speeds.
//
// SPI mode 1 (CPOL=0, CPHA=1), MSB first, CS active low.
// Returns the 14-bit absolute mechanical angle as radians in [0, 2*PI).
// ---------------------------------------------------------------------------
class AS5047P_SoftSPI : public Sensor {
 public:
  AS5047P_SoftSPI(int sck, int miso, int mosi, int cs)
      : sck_(sck), miso_(miso), mosi_(mosi), cs_(cs) {}

  void init() {
    pinMode(cs_, OUTPUT);
    pinMode(sck_, OUTPUT);
    pinMode(mosi_, OUTPUT);
    pinMode(miso_, INPUT);
    digitalWrite(cs_, HIGH);   // deselect
    digitalWrite(sck_, LOW);   // CPOL=0 idle low
    digitalWrite(mosi_, LOW);
    // Establish the SimpleFOC baseline (full-rotation tracking starts here).
    this->Sensor::init();
  }

  // SimpleFOC calls this every loop; return the raw shaft angle in radians.
  float getSensorAngle() override {
    // Command: read ANGLECOM (0x3FFF) with read bit (0x4000) + even parity bit
    // in the MSB => 0xFFFF. The AS5047P returns the result of the *previous*
    // command, so reading ANGLECOM continuously yields a fresh angle each call.
    uint16_t raw = transfer16(0xFFFF) & 0x3FFF;
    return ((float)raw) * (_2PI / 16384.0f);
  }

 private:
  uint16_t transfer16(uint16_t out) {
    uint16_t in = 0;
    digitalWrite(cs_, LOW);
    delayMicroseconds(1);  // CS-to-clock setup
    for (int i = 15; i >= 0; --i) {
      digitalWrite(mosi_, (out >> i) & 0x1);
      delayMicroseconds(1);
      digitalWrite(sck_, HIGH);   // leading edge: slave latches MOSI / drives MISO
      delayMicroseconds(1);
      digitalWrite(sck_, LOW);    // trailing edge: sample MISO (CPHA=1)
      in = (in << 1) | (digitalRead(miso_) & 0x1);
    }
    digitalWrite(cs_, HIGH);
    delayMicroseconds(1);
    return in;
  }

  int sck_, miso_, mosi_, cs_;
};

#endif  // AS5047P_SOFTSPI_H
