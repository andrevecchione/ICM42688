#include <AltSoftSerial.h>

#define RS485_DIR 2

// Set to 1 for high-rate binary forwarding (recommended for 500 Hz+).
// Set to 0 for the older human-readable line mode.
#define BINARY_BRIDGE_MODE 1

AltSoftSerial rs485;

#if !BINARY_BRIDGE_MODE
// CSV IMU frames are much longer than "HELLO".
// Keep this comfortably above your longest expected line.
char rxBuf[128];
uint16_t rxIndex = 0;
bool droppingLine = false;
bool overflowNotified = false;

uint32_t lastHeartbeatMs = 0;
#endif

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

  // USB serial to notebook
  Serial.begin(115200);
  rs485.begin(115200);

  Serial.println("RS485 MASTER READY");
}

void loop() {
#if BINARY_BRIDGE_MODE
  // Forward RS485 bytes directly to USB serial.
  while (rs485.available()) {
    const int b = rs485.read();
    if (b >= 0) {
      Serial.write((uint8_t)b);
    }
  }
#else
  // Heartbeat so you can confirm USB serial is alive (useful when debugging WSL/USBIP).
  if (millis() - lastHeartbeatMs >= 1000) {
    lastHeartbeatMs = millis();
    Serial.println("#HB");
  }

  // Read all available bytes
  while (rs485.available()) {
    char c = rs485.read();

    if (c == '\r') continue;  // ignore carriage return

    if (droppingLine) {
      // Discard bytes until the end-of-line so we re-sync to the next frame.
      if (c == '\n') {
        droppingLine = false;
        rxIndex = 0;
      }
      continue;
    }

    if (c == '\n') {
      rxBuf[rxIndex] = '\0';  // terminate string
      Serial.println(rxBuf);   // print received message
      rxIndex = 0;             // reset buffer for next message
      overflowNotified = false;
    } else if (rxIndex < sizeof(rxBuf) - 1) {
      rxBuf[rxIndex++] = c;    // store incoming byte
    } else {
      // Line too long; drop the rest until newline.
      rxIndex = 0;
      droppingLine = true;
      if (!overflowNotified) {
        Serial.println("[WARN] RS485 line overflow; dropping until newline");
        overflowNotified = true;
      }
    }
  }

#endif
}
