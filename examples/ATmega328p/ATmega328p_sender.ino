// ATmega328p (standalone) -> MAX485 -> RS485 bus -> UNO (AltSoftSerial)
// High-rate streaming using ICM42688 FIFO + a compact binary packet format.

#include "ICM42688.h"

#include <SPI.h>

#define RS485_DIR 2   // DE + /RE tied together (TX enable)

// SPI wiring notes (ATmega328p / Arduino UNO pin mapping):
// - SCK  -> D13
// - MISO -> D12
// - MOSI -> D11
// - CS   -> PB2 (SS). In Arduino pin numbering this is typically D10.
// Using SS keeps this portable across bare-chip board packages.
static constexpr uint8_t ICM_CS_PIN = SS;

// Use SPI mode 3; the library configures SPISettings internally.
// Keep HS clock conservative on long wires; 1-8 MHz is typical.
ICM42688_FIFO IMU(SPI, ICM_CS_PIN, 1000000);

static constexpr uint32_t RS485_BAUD = 115200;

// Binary frame: 2 sync bytes + 16-byte payload + 1-byte XOR checksum (fixed 19 bytes)
// Payload (little-endian):
// - t_ms: uint32
// - ax,ay,az,gx,gy,gz: int16 each
static constexpr uint8_t SYNC0 = 0xA5;
static constexpr uint8_t SYNC1 = 0x5A;
static constexpr size_t PAYLOAD_LEN = 16;
static constexpr size_t FRAME_LEN = 2 + PAYLOAD_LEN + 1;

static inline void _write_i16_le(uint8_t* dst, int16_t v);

void setup() {
  pinMode(RS485_DIR, OUTPUT);
  digitalWrite(RS485_DIR, HIGH); // always transmit for this test

  Serial.begin(RS485_BAUD); // hardware serial connected to MAX485

  // Initialize IMU
  int status = IMU.begin();
  if (status < 0) {
    // Send diagnostics over RS485 so the UNO can show them.
    Serial.println("IMU init failed");
    Serial.print("Status:");
    Serial.println(status);
    while (1) {
      delay(1000);
    }
  }

  // Configure IMU for FIFO streaming at 500 Hz.
  IMU.setAccelFS(ICM42688::gpm8);
  IMU.setGyroFS(ICM42688::dps1000);
  IMU.setAccelODR(ICM42688::odr500);
  IMU.setGyroODR(ICM42688::odr500);
  IMU.setFilters(false, false);

  // Enable FIFO for accel+gyro (temp is always enabled by this library's FIFO helper).
  status = IMU.enableFifo(true, true, false);
  if (status < 0) {
    Serial.println("FIFO enable failed");
    Serial.print("Status:");
    Serial.println(status);
    while (1) {
      delay(1000);
    }
  }
  status = IMU.streamToFifo();
  if (status < 0) {
    Serial.println("FIFO stream start failed");
    Serial.print("Status:");
    Serial.println(status);
    while (1) {
      delay(1000);
    }
  }
}

void loop() {
  // Pull frames from FIFO and stream them as binary packets.
  // Keep per-loop work bounded to reduce jitter.
  static uint8_t fifoFrame[32];
  const size_t frameSize = IMU.fifoFrameSize();
  if (frameSize == 0 || frameSize > sizeof(fifoFrame)) {
    return;
  }

  if (IMU.updateFifoByteCount() < 0) {
    return;
  }

  const size_t fifoBytes = IMU.fifoByteCount();
  const size_t availableFrames = fifoBytes / frameSize;
  if (availableFrames == 0) {
    return;
  }

  const size_t framesThisLoop = availableFrames > 8 ? 8 : availableFrames;

  for (size_t n = 0; n < framesThisLoop; n++) {
    if (IMU.readFifoFrame(fifoFrame, frameSize) < 0) {
      break;
    }

    // FIFO frame layout (structure 3/4 typical when accel+gyro enabled):
    // [0]=header, [1..6]=acc xyz (big-endian int16), [7..12]=gyro xyz (big-endian int16),
    // [13]=temp (int8), [14..15]=timestamp (uint16) (ignored here)
    const int16_t ax = (int16_t)((fifoFrame[1] << 8) | fifoFrame[2]);
    const int16_t ay = (int16_t)((fifoFrame[3] << 8) | fifoFrame[4]);
    const int16_t az = (int16_t)((fifoFrame[5] << 8) | fifoFrame[6]);
    const int16_t gx = (int16_t)((fifoFrame[7] << 8) | fifoFrame[8]);
    const int16_t gy = (int16_t)((fifoFrame[9] << 8) | fifoFrame[10]);
    const int16_t gz = (int16_t)((fifoFrame[11] << 8) | fifoFrame[12]);
    const uint32_t t_ms = millis();

    uint8_t payload[PAYLOAD_LEN];
    payload[0] = (uint8_t)(t_ms & 0xFF);
    payload[1] = (uint8_t)((t_ms >> 8) & 0xFF);
    payload[2] = (uint8_t)((t_ms >> 16) & 0xFF);
    payload[3] = (uint8_t)((t_ms >> 24) & 0xFF);

    _write_i16_le(payload + 4, ax);
    _write_i16_le(payload + 6, ay);
    _write_i16_le(payload + 8, az);
    _write_i16_le(payload + 10, gx);
    _write_i16_le(payload + 12, gy);
    _write_i16_le(payload + 14, gz);

    uint8_t cksum = 0;
    for (size_t i = 0; i < PAYLOAD_LEN; i++) {
      cksum ^= payload[i];
    }

    Serial.write(SYNC0);
    Serial.write(SYNC1);
    Serial.write(payload, PAYLOAD_LEN);
    Serial.write(cksum);
  }
}

static inline void _write_i16_le(uint8_t* dst, int16_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
}
