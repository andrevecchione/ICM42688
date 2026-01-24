// Diagnostic sketch: periodically report Serial.available() over RS485.
// Helps determine whether the MCU is receiving bytes from the transceiver.

#include <Arduino.h>

#define RS485_DIR 2   // DE + /RE tied together (TX enable)
// Node id for polled responses
#define NODE_ID 1

static constexpr uint32_t RS485_BAUD = 9600;

// Give the transceiver a bit more time to switch direction reliably.
// These values are conservative but inexpensive; we can reduce later.
void rs485Transmit() { digitalWrite(RS485_DIR, HIGH); delayMicroseconds(200); }
// Deassert DE first, then wait for the line to settle.
void rs485Receive() { digitalWrite(RS485_DIR, LOW); delayMicroseconds(200); }

void setup() {
  pinMode(RS485_DIR, OUTPUT);
  rs485Receive();
  Serial.begin(RS485_BAUD);
}

void loop() {
  // Measure how many bytes are pending in the hardware UART RX
  int avail = Serial.available();

  // Send diagnostic line with timestamp and DE toggling
  char buf[80];
  int n = snprintf(buf, sizeof(buf), "%lu RXCOUNT: %d\n", (unsigned long)millis(), avail);
  if (n > 0) {
    // Do NOT assert DE — keep this node receive-only to avoid collisions
    Serial.write((const uint8_t*)buf, (size_t)n);
  }

  // If any bytes available, read them into a buffer and echo up to 64 bytes as hex
  if (avail > 0) {
    size_t toread = avail > 64 ? 64 : avail;
    uint8_t rb[64];
    size_t got = 0;
    unsigned long start = millis();
    // Read up to `toread` bytes, with a longer timeout to accumulate what's already arrived.
    while (got < toread && (millis() - start) < 100) {
      int c = Serial.read();
      if (c < 0) { delay(1); continue; }
      rb[got++] = (uint8_t)c;
      // Immediate sliding-window poll detection: if the last 4 bytes match a poll
      if (got >= 4) {
        size_t p = got - 4;
        if (rb[p] == 0xA5 && rb[p+1] == 0x5A && rb[p+2] == 0x51 && (rb[p+3] == NODE_ID || rb[p+3] == 0xFF)) {
          // reply immediately (duplicate reply to improve capture chance)
          const char *reply = "PONG\n";
          rs485Transmit();
          Serial.write((const uint8_t*)reply, strlen(reply));
          Serial.write((const uint8_t*)reply, strlen(reply));
          Serial.flush();
          delay(100);
          rs485Receive();
          // keep reading remaining available bytes
        }
      }
    }

    if (got > 0) {
      // Check for poll frame: 0xA5 0x5A 0x51 <node_id>
      for (size_t i = 0; i + 3 < got; ++i) {
        if (rb[i] == 0xA5 && rb[i+1] == 0x5A && rb[i+2] == 0x51 && (rb[i+3] == NODE_ID || rb[i+3] == 0xFF)) {
          // reply briefly (send duplicate reply to improve chance UNO captures it)
          const char *reply = "PONG\n";
          rs485Transmit();
          Serial.write((const uint8_t*)reply, strlen(reply));
          // duplicate reply
          Serial.write((const uint8_t*)reply, strlen(reply));
          Serial.flush();
          delay(100);
          rs485Receive();
          break;
        }
      }

      // Echo received bytes to the serial monitor as hex (no DE assert)
      // detect either the older 4-byte poll or the new single-byte poll (0x51)
      for (size_t i = 0; i < got; ++i) {
        // single-byte poll marker
        if (rb[i] == 0x51) {
          const char *reply = "PONG\n";
          rs485Transmit();
          Serial.write((const uint8_t*)reply, strlen(reply));
          Serial.write((const uint8_t*)reply, strlen(reply));
          Serial.flush();
          delay(100);
          rs485Receive();
          break;
        }
        // legacy 4-byte poll detection (if present)
        if (i + 3 < got && rb[i] == 0xA5 && rb[i+1] == 0x5A && rb[i+2] == 0x51 && (rb[i+3] == NODE_ID || rb[i+3] == 0xFF)) {
          const char *reply = "PONG\n";
          rs485Transmit();
          Serial.write((const uint8_t*)reply, strlen(reply));
          Serial.write((const uint8_t*)reply, strlen(reply));
          Serial.flush();
          delay(100);
          rs485Receive();
          break;
        }
      }
      }
    }

    // Periodic unsolicited self-test transmit so the master can observe this node
}

// Periodic unsolicited self-test transmit so the master can observe this node
// even if polling fails. Sends a duplicated ASCII `SELFTEST\n` every 8s.
// This asserts DE only for the transmit window.
// Useful for debugging; remove when done.
// NOTE: keep low frequency to avoid bus collisions in real deployments.
void __attribute__((weak)) __send_selftest_if_time() {
  static unsigned long last = 0;
  const unsigned long now = millis();
  if (now - last < 8000) return;
  last = now;

  const char *msg = "SELFTEST\n";
  rs485Transmit();
  // send duplicate to improve capture chance
  Serial.write((const uint8_t*)msg, strlen(msg));
  Serial.write((const uint8_t*)msg, strlen(msg));
  Serial.flush();
  delay(100);
  rs485Receive();
}

// Call the self-test sender from loop via weak symbol to avoid breaking
// existing control flow (call manually after flashing or integrate into loop).
