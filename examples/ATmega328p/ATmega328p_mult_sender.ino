// ATmega328p (standalone) polled sender for RS485 multi-drop bus.
// Listens for a master poll and responds with a single FIFO batch packet.

#include "ICM42688.h"
#include <SPI.h>

#define RS485_DIR 2   // DE + /RE tied together (TX enable)

// SPI wiring notes: use SS for CS as in other examples
static constexpr uint8_t ICM_CS_PIN = SS;
ICM42688_FIFO IMU(SPI, ICM_CS_PIN, 1000000);

static constexpr uint32_t RS485_BAUD = 57600;

// Batch parameters (same format as original sender)
static constexpr uint8_t SYNC2 = 0x42; // 'B' for batch
static constexpr uint8_t SYNC0 = 0xA5;
static constexpr uint8_t SYNC1 = 0x5A;
static constexpr size_t BATCH_SAMPLES = 20;

// Poll marker and node ID
#define POLL_MARKER 0x51 // 'Q'
#ifndef NODE_ID
#define NODE_ID 1
#endif

// Default FSYNC config value (written to UB0_REG_FSYNC_CONFIG).
// Adjust per datasheet if needed; set to non-zero to enable basic FSYNC sampling.
#ifndef FSYNC_CONFIG_VAL
#define FSYNC_CONFIG_VAL 0x01
#endif

static inline void _write_i16_le(uint8_t* dst, int16_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void _write_u16_le(uint8_t* dst, uint16_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline uint16_t _crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
      else crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

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

  // Serial is wired to MAX485 on this board.
  Serial.begin(RS485_BAUD);

  // Initialize IMU
  int status = IMU.begin();
  if (status < 0) {
    // block; nothing to do without IMU
    while (1) delay(1000);
  }

  IMU.setAccelFS(ICM42688::gpm8);
  IMU.setGyroFS(ICM42688::dps1000);
  IMU.setAccelODR(ICM42688::odr500);
  IMU.setGyroODR(ICM42688::odr500);
  IMU.setFilters(false, false);
  status = IMU.enableFifo(true, true, false);
  if (status < 0) while (1) delay(1000);
  status = IMU.streamToFifo();
  if (status < 0) while (1) delay(1000);

  // Enable FSYNC sampling in the IMU (value depends on datasheet settings).
  IMU.setFsyncConfig(FSYNC_CONFIG_VAL);
}

// Wait for a poll addressed to this node. Simple byte matcher.
bool check_for_poll() {
  static uint8_t state = 0;
  while (Serial.available()) {
    const int b = Serial.read();
    if (b < 0) return false;
    const uint8_t ub = (uint8_t)b;
    if (state == 0) {
      if (ub == SYNC0) state = 1;
    } else if (state == 1) {
      if (ub == SYNC1) state = 2;
      else state = (ub == SYNC0) ? 1 : 0;
    } else if (state == 2) {
      if (ub == POLL_MARKER) state = 3; else state = 0;
    } else if (state == 3) {
      // ub is node id
      const uint8_t nid = ub;
      state = 0;
      return nid == (uint8_t)NODE_ID;
    }
  }
  return false;
}

void send_one_batch() {
  // Determine available frames
  if (IMU.updateFifoByteCount() < 0) return;
  const size_t frameSize = IMU.fifoFrameSize();
  if (frameSize == 0 || frameSize > 64) return;
  const size_t fifoBytes = IMU.fifoByteCount();
  size_t availableFrames = fifoBytes / frameSize;
  if (availableFrames == 0) return; // nothing to send

  const size_t samplesToSend = availableFrames > BATCH_SAMPLES ? BATCH_SAMPLES : availableFrames;

  // allocate packet: 3 sync + 1 count + samples*14 + 2 crc
  const size_t pktMax = 3 + 1 + samplesToSend * 14 + 2;
  uint8_t packet[3 + 1 + BATCH_SAMPLES * 14 + 2];
  uint8_t fifoFrame[64];

  // Packet layout now:
  // [SYNC0 SYNC1 SYNC2] [count u8] [fsync_ts u16 LE] [samples...] [crc16 LE]
  packet[0] = SYNC0;
  packet[1] = SYNC1;
  packet[2] = SYNC2;
  packet[3] = 0; // count placeholder
  // reserve two bytes for fsync timestamp (filled just before transmit)
  packet[4] = 0;
  packet[5] = 0;
  uint8_t batchCount = 0;

  for (size_t n = 0; n < samplesToSend; n++) {
    if (IMU.readFifoFrame(fifoFrame, frameSize) < 0) break;
    const int16_t ax = (int16_t)((fifoFrame[1] << 8) | fifoFrame[2]);
    const int16_t ay = (int16_t)((fifoFrame[3] << 8) | fifoFrame[4]);
    const int16_t az = (int16_t)((fifoFrame[5] << 8) | fifoFrame[6]);
    const int16_t gx = (int16_t)((fifoFrame[7] << 8) | fifoFrame[8]);
    const int16_t gy = (int16_t)((fifoFrame[9] << 8) | fifoFrame[10]);
    const int16_t gz = (int16_t)((fifoFrame[11] << 8) | fifoFrame[12]);
    const uint16_t imu_ts = (uint16_t)((fifoFrame[14] << 8) | fifoFrame[15]);

    const size_t sampleOff = 3 + 1 + 2 + (size_t)batchCount * 14;
    _write_u16_le(packet + sampleOff + 0, imu_ts);
    _write_i16_le(packet + sampleOff + 2, ax);
    _write_i16_le(packet + sampleOff + 4, ay);
    _write_i16_le(packet + sampleOff + 6, az);
    _write_i16_le(packet + sampleOff + 8, gx);
    _write_i16_le(packet + sampleOff + 10, gy);
    _write_i16_le(packet + sampleOff + 12, gz);
    batchCount++;
  }

  if (batchCount == 0) return;

  packet[3] = batchCount;
  // read FSYNC timestamp register and write little-endian
  uint16_t fsync_ts = 0;
  if (IMU.getFsyncTimestamp(fsync_ts) > 0) {
    packet[4] = (uint8_t)(fsync_ts & 0xFF);
    packet[5] = (uint8_t)((fsync_ts >> 8) & 0xFF);
  } else {
    packet[4] = 0;
    packet[5] = 0;
  }

  const size_t bodyLen = 1 + 2 + (size_t)batchCount * 14; // count + fsync_ts + samples
  const uint16_t crc = _crc16_ccitt(packet + 3, bodyLen);
  const size_t crcOff = 3 + bodyLen;
  _write_u16_le(packet + crcOff, crc);
  const size_t pktLen = crcOff + 2;

  // Transmit on RS485
  rs485Transmit();
  Serial.write(packet, pktLen);
  Serial.flush();
  rs485Receive();
}

void loop() {
  if (check_for_poll()) {
    send_one_batch();
  }
}
