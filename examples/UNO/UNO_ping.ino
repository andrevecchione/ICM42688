#include <AltSoftSerial.h>

#define RS485_DIR 2
// Optional debug pin you can probe with a meter/oscilloscope
#define DEBUG_TX_PIN 3

AltSoftSerial rs485;

static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t RS485_BAUD = 9600;

// How long (ms) to listen for replies after sending a PING
static const unsigned long PING_REPLY_WINDOW_MS = 500;

void rs485Transmit() {
  digitalWrite(RS485_DIR, HIGH);
  // give transceiver time to enable driver
  delayMicroseconds(200);
}

void rs485Receive() {
  // deassert driver then wait for line to settle
  digitalWrite(RS485_DIR, LOW);
  delayMicroseconds(200);
}

void setup() {
  pinMode(RS485_DIR, OUTPUT);
  pinMode(DEBUG_TX_PIN, OUTPUT);
  rs485Receive();

  Serial.begin(USB_BAUD);
  rs485.begin(RS485_BAUD);

  Serial.println("PING-PONG master ready");
  Serial.println("Type 'p' then Enter on the USB serial to send one poll");
}

void loop() {
  // Manual poll trigger: send one poll when user types 'p' on USB serial
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch == 'p' || ch == 'P') {
      const uint8_t POLL_MARKER = 0x51; // single-byte poll marker

      digitalWrite(DEBUG_TX_PIN, HIGH);
      rs485Transmit();
      delay(5);
      unsigned long sendStart = millis();
      // send the single-byte poll marker repeatedly for 200 ms
      while ((millis() - sendStart) < 200) {
        rs485.write(POLL_MARKER);
        delay(2);
      }
      rs485.flush();
      delay(100);
      rs485Receive();
      digitalWrite(DEBUG_TX_PIN, LOW);

      // debug print of single-byte poll marker
      Serial.print("-> POLL (1-byte) (hex): ");
      if (POLL_MARKER < 0x10) Serial.print('0');
      Serial.print(POLL_MARKER, HEX);
      Serial.println();
    }
  }

  // read any response from the RS485 line and print per-byte with timestamp
  unsigned long start = millis();
  while ((millis() - start) < PING_REPLY_WINDOW_MS) {
    while (rs485.available()) {
      int c = rs485.read();
      if (c < 0) break;
      // print timestamped hex for each byte
      Serial.print("<- t="); Serial.print(millis()); Serial.print(" ms HEX: ");
      if (c < 0x10) Serial.print('0');
      Serial.print((uint8_t)c, HEX);
      Serial.println();
    }
    delay(5);
  }

  // Also check hardware Serial RX in case the RS485 transceiver is wired
  // to hardware UART (D0/D1) instead of AltSoftSerial.
  static char hwbuf[128];
  static size_t hwidx = 0;
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      hwbuf[hwidx] = '\0';
      Serial.print("<- HW: ");
      Serial.println(hwbuf);
      Serial.print("<- HW HEX: ");
      for (size_t j = 0; j < hwidx; ++j) {
        uint8_t b = (uint8_t)hwbuf[j];
        if (b < 0x10) Serial.print('0');
        Serial.print(b, HEX);
        Serial.print(' ');
      }
      Serial.println();
      hwidx = 0;
    } else if (hwidx + 1 < sizeof(hwbuf)) {
      hwidx++; // store but don't duplicate into main buf (we're already printing)
      hwbuf[hwidx-1] = (char)c;
    } else {
      hwidx = 0;
    }
  }
}
