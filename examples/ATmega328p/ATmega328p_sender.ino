// ATmega328p (standalone) -> MAX485 -> RS485 bus -> UNO (AltSoftSerial)
// High-rate streaming using ICM42688 FIFO + a compact binary packet format.
// Note: TX is rate-limited (see TX_HZ) to improve RS485 reliability.

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

// RS485 throughput is limited (especially through UNO+AltSoftSerial).
// We run RS485 at a conservative baud that has proven reliable.
static constexpr uint32_t RS485_BAUD = 57600;

// IMU runs at 500 Hz; transmit ~400 Hz by dropping 1 out of every 5 FIFO frames.
static constexpr uint8_t DOWNSAMPLE_DEN = 5;
static constexpr uint8_t DOWNSAMPLE_NUM = 4; // keep 4/5

// Batched packet format (uses IMU FIFO timestamp, not host timing):
//   SYNC3 (A5 5A 42) + count(u8) + samples + CRC16(u16 LE)
// Each sample is 14 bytes:
//   imu_ts (uint16 LE) + ax,ay,az,gx,gy,gz (int16 LE)
static constexpr uint8_t SYNC2 = 0x42; // 'B' for batch
static constexpr uint8_t BATCH_SAMPLES = 20; // larger batch reduces overhead at 57600

// Binary frame: 2 sync bytes + 16-byte payload + 1-byte XOR checksum (fixed 19 bytes)
// Payload (little-endian):
// - t_ms: uint32
// - ax,ay,az,gx,gy,gz: int16 each
static constexpr uint8_t SYNC0 = 0xA5;
static constexpr uint8_t SYNC1 = 0x5A;
static constexpr size_t PAYLOAD_LEN = 16;

static inline void _write_i16_le(uint8_t* dst, int16_t v);
static inline void _write_u16_le(uint8_t* dst, uint16_t v);
static inline void _write_u32_le(uint8_t* dst, uint32_t v);
static inline uint16_t _crc16_ccitt(const uint8_t* data, size_t len);

void setup() {
  pinMode(RS485_DIR, OUTPUT);
  digitalWrite(RS485_DIR, HIGH); // always transmit for this test

  Serial.begin(RS485_BAUD); // hardware serial connected to MAX485

  // Initialize IMU
  int status = IMU.begin();
  if (status < 0) {
    // Send diagnostics over RS485 so the UNO can show them.
    Serial.println(F("IMU init failed"));
    Serial.print(F("Status:"));
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
    Serial.println(F("FIFO enable failed"));
    Serial.print(F("Status:"));
    Serial.println(status);
    while (1) {
      delay(1000);
    }
  }
  status = IMU.streamToFifo();
  if (status < 0) {
    Serial.println(F("FIFO stream start failed"));
    Serial.print(F("Status:"));
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
  static uint32_t fifoSeq = 0; // counts every FIFO frame drained
  static uint8_t packet[3 + 1 + (14 * BATCH_SAMPLES) + 2];
  static uint8_t batchCount = 0;
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

  const size_t framesThisLoop = availableFrames > 16 ? 16 : availableFrames;

  for (size_t n = 0; n < framesThisLoop; n++) {
    if (IMU.readFifoFrame(fifoFrame, frameSize) < 0) {
      break;
    }

    // Downsample transmitted frames (but still drain FIFO to avoid overflow).
    fifoSeq++;
    if ((fifoSeq % DOWNSAMPLE_DEN) >= DOWNSAMPLE_NUM) {
      continue;
    }

    // FIFO frame layout (structure 3/4 typical when accel+gyro enabled):
    // [0]=header, [1..6]=acc xyz (big-endian int16), [7..12]=gyro xyz (big-endian int16),
    // [13]=temp (int8), [14..15]=timestamp (uint16)
    const int16_t ax = (int16_t)((fifoFrame[1] << 8) | fifoFrame[2]);
    const int16_t ay = (int16_t)((fifoFrame[3] << 8) | fifoFrame[4]);
    const int16_t az = (int16_t)((fifoFrame[5] << 8) | fifoFrame[6]);
    const int16_t gx = (int16_t)((fifoFrame[7] << 8) | fifoFrame[8]);
    const int16_t gy = (int16_t)((fifoFrame[9] << 8) | fifoFrame[10]);
    const int16_t gz = (int16_t)((fifoFrame[11] << 8) | fifoFrame[12]);
    const uint16_t imu_ts = (uint16_t)((fifoFrame[14] << 8) | fifoFrame[15]);

    if (batchCount == 0) {
      packet[0] = SYNC0;
      packet[1] = SYNC1;
      packet[2] = SYNC2;
      packet[3] = 0; // count placeholder
    }

    const size_t sampleOff = 3 + 1 + (size_t)batchCount * 14;
    _write_u16_le(packet + sampleOff + 0, imu_ts);
    _write_i16_le(packet + sampleOff + 2, ax);
    _write_i16_le(packet + sampleOff + 4, ay);
    _write_i16_le(packet + sampleOff + 6, az);
    _write_i16_le(packet + sampleOff + 8, gx);
    _write_i16_le(packet + sampleOff + 10, gy);
    _write_i16_le(packet + sampleOff + 12, gz);
    batchCount++;

    if (batchCount >= BATCH_SAMPLES) {
      packet[3] = batchCount;
      const size_t bodyLen = 1 + (size_t)batchCount * 14;
      const uint16_t crc = _crc16_ccitt(packet + 3, bodyLen);
      const size_t crcOff = 3 + bodyLen;
      _write_u16_le(packet + crcOff, crc);
      const size_t pktLen = crcOff + 2;
      Serial.write(packet, pktLen);
      batchCount = 0;
    }
  }

  // Flush a partially-filled batch so latency stays bounded.
  if (batchCount > 0) {
    packet[3] = batchCount;
    const size_t bodyLen = 1 + (size_t)batchCount * 14;
    const uint16_t crc = _crc16_ccitt(packet + 3, bodyLen);
    const size_t crcOff = 3 + bodyLen;
    _write_u16_le(packet + crcOff, crc);
    const size_t pktLen = crcOff + 2;
    Serial.write(packet, pktLen);
    batchCount = 0;
  }
}

static inline void _write_i16_le(uint8_t* dst, int16_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void _write_u16_le(uint8_t* dst, uint16_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void _write_u32_le(uint8_t* dst, uint32_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
  dst[2] = (uint8_t)((v >> 16) & 0xFF);
  dst[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline uint16_t _crc16_ccitt(const uint8_t* data, size_t len) {
  // CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no xorout
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}
