// Simple ATmega328p responder for RS485 PING-PONG test.
// Listens on hardware Serial for "PING" and replies "PONG".

#include <Arduino.h>
#include <SPI.h>

#define RS485_DIR 2   // DE + /RE tied together (TX enable)
// Debug output pin to indicate bytes seen by the MCU (probe this pin)
#define DEBUG_RX_PIN 4

static constexpr uint32_t RS485_BAUD = 57600;

void rs485Transmit() {
  digitalWrite(RS485_DIR, HIGH);
  delayMicroseconds(20);
}

void rs485Receive() {
  delayMicroseconds(20);
  digitalWrite(RS485_DIR, LOW);
}

void setup() {
  pinMode(RS485_DIR, OUTPUT);
  rs485Receive();

  // Hardware Serial is connected to the RS485 transceiver on this board
  Serial.begin(RS485_BAUD);
  // LED for visual receive indication
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DEBUG_RX_PIN, OUTPUT);
  digitalWrite(DEBUG_RX_PIN, LOW);
}

void loop() {
  static String line;
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;
    // Indicate at hardware level that a byte was observed
    digitalWrite(DEBUG_RX_PIN, HIGH);
    // Echo received byte back immediately (raw) so master can detect reception
    rs485Transmit();
    Serial.write((uint8_t)c);
    Serial.flush();
    rs485Receive();
    delay(5);
    digitalWrite(DEBUG_RX_PIN, LOW);

    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line == "PING") {
        // send PONG twice to increase chance of seeing it on the master
        rs485Transmit();
        Serial.print("PONG\n");
        Serial.flush();
        rs485Receive();

        delay(10);

        rs485Transmit();
        Serial.print("PONG\n");
        Serial.flush();
        rs485Receive();
      }
      line = "";
    } else {
      line += (char)c;
      // keep string bounded
      if (line.length() > 64) line = "";
    }
  }
}
