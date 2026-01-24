#include <AltSoftSerial.h>

#define RS485_DIR 2

// Poll constants (must match sender)
static constexpr uint8_t SYNC0 = 0xA5;
static constexpr uint8_t SYNC1 = 0x5A;
#define POLL_MARKER 0x51 // 'Q'

AltSoftSerial rs485;

static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t RS485_BAUD = 57600;

// Configure which node IDs to poll on the bus
const uint8_t node_ids[] = {1, 2, 3};
const size_t node_count = sizeof(node_ids) / sizeof(node_ids[0]);

// Timing
static constexpr uint16_t REPLY_TIMEOUT_MS = 80; // wait for node reply
static constexpr uint16_t POLL_PERIOD_MS = 200; // between rounds

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

  Serial.begin(USB_BAUD);
  rs485.begin(RS485_BAUD);

  Serial.println("RS485 MASTER POLLER READY");
}

void poll_node(uint8_t nid) {
  uint8_t poll[4] = {SYNC0, SYNC1, POLL_MARKER, nid};

  rs485Transmit();
  rs485.write(poll, sizeof(poll));
  rs485.flush();
  rs485Receive();

  unsigned long start = millis();
  bool any = false;
  while ((millis() - start) < REPLY_TIMEOUT_MS) {
    while (rs485.available()) {
      const int b = rs485.read();
      if (b < 0) break;
      Serial.write((uint8_t)b);
      any = true;
    }
    // small yield to avoid busy-looping
    delay(1);
  }

  // Optionally indicate missing node (commented out to preserve binary stream)
  // if (!any) Serial.print("[NO_REPLY]");
}

void loop() {
  const unsigned long roundStart = millis();
  for (size_t i = 0; i < node_count; ++i) {
    poll_node(node_ids[i]);
    // small gap between polls to allow remote driver turnaround
    delay(2);
  }

  // keep overall poll frequency bounded
  const unsigned long elapsed = millis() - roundStart;
  if (elapsed < POLL_PERIOD_MS) delay(POLL_PERIOD_MS - elapsed);
}
