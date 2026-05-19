/*
  comms.h — HX1A Custom Binary Protocol
  Pico 2 (RP2350) — PlatformIO / Arduino framework

  ── Frame Structure ─────────────────────────────────────────────────
  All frames (both directions) share the same envelope:

    [0xAA] [0x55] [MSG_TYPE : 1B] [LEN : 1B] [...payload : LEN bytes...] [CRC16_CCITT : 2B]

  Header:  0xAA 0x55  — sync bytes, fixed
  Type:    one of CMD_* or TEL_STATUS
  Len:     payload length in bytes (does NOT include header, type, len, or CRC)
  CRC16:   CRC-16/CCITT-FALSE over bytes [type, len, payload...] (excludes sync header)

  ── Inbound Command Types ────────────────────────────────────────────
  CMD_VEL          0x10   Velocity walk command       float[3]: X, Y, R  (-127..127)
  CMD_PITCH_ROLL   0x11   Manual pitch/roll setpoint  float[2]: pitch, roll  (degrees)
  CMD_GAIT         0x12   Select gait                 float[1]: 0=tripod 1=wave 2=ripple 3=tetrapod
  CMD_MODE         0x13   Select mode                 float[1]: 0=idle 1=walk 2=translate 3=rotate 99=all90
  CMD_LEG_POS      0x14   XYZ leg override            float[4]: leg, X, Y, Z
  CMD_LEG_DEG      0x15   Direct servo angles         float[4]: leg, coxa, femur, tibia
  CMD_RESET_OFFSETS 0x16  Reset all offsets + overrides  (no payload)
  CMD_STABILIZE    0x17   Enable/disable stabilization   float[1]: 1=on 0=off
  CMD_HIGH_STEP    0x18   High-step mode              float[1]: 0=off 1=high 2=highest
  CMD_GAIT_SPEED   0x19   Gait speed                  float[1]: 0=fast 1=slow

  ── Outbound Telemetry ───────────────────────────────────────────────
  TEL_STATUS       0x50   Full status frame  (see HexTelemetry struct)

  ── Payload Encoding ────────────────────────────────────────────────
  All floats are packed IEEE-754 little-endian (native for RP2350).
  Integer flags are packed as uint8_t.
*/

#pragma once
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════
//  MESSAGE TYPE IDs
// ═══════════════════════════════════════════════════════════════════
#define CMD_VEL            0x10
#define CMD_PITCH_ROLL     0x11
#define CMD_GAIT           0x12
#define CMD_MODE           0x13
#define CMD_LEG_POS        0x14
#define CMD_LEG_DEG        0x15
#define CMD_RESET_OFFSETS  0x16
#define CMD_STABILIZE      0x17
#define CMD_HIGH_STEP      0x18
#define CMD_GAIT_SPEED     0x19

#define TEL_STATUS         0x50

// ── Frame constants ──────────────────────────────────────────────
#define COMMS_SYNC0        0xAA
#define COMMS_SYNC1        0x55
#define COMMS_MAX_PAYLOAD  256   // max payload bytes per frame
#define COMMS_HEADER_SIZE  4     // sync0 sync1 type len
#define COMMS_CRC_SIZE     2

// ═══════════════════════════════════════════════════════════════════
//  INBOUND COMMAND STRUCT
// ═══════════════════════════════════════════════════════════════════
struct HexCmd {
  uint8_t type;       // CMD_* value
  uint8_t nFloats;    // how many floats are valid in f[]
  float   f[8];       // decoded float payload (max 8 floats per cmd)
};

// ═══════════════════════════════════════════════════════════════════
//  OUTBOUND TELEMETRY STRUCT
// ═══════════════════════════════════════════════════════════════════
struct HexTelemetry {
  // Flags
  uint8_t mode;
  uint8_t gait;
  uint8_t safeMode;
  uint8_t stabilize;
  uint8_t pcaStatus;   // bit0=leftOK bit1=rightOK

  // IMU
  float imu_pitch;
  float imu_roll;
  float imu_yaw;
  float imu_gx;
  float imu_gy;
  float imu_gz;
  float imu_ax;
  float imu_ay;
  float imu_az;

  // Tibia tips — bitmask + XYZ per leg
  uint8_t tipContact;  // bit per leg, bit0=leg0
  float   tipX[6];
  float   tipY[6];
  float   tipZ[6];

  // All 18 servo current angles (leg*3+seg order)
  float servoAngle[18];
};

// Telemetry payload size:
//   5 uint8  = 5
//   9 floats (IMU) = 36
//   1 uint8 (tipContact) = 1
//   18 floats (tipXYZ) = 72
//   18 floats (servo)  = 72
//   Total payload = 186 bytes
#define TEL_PAYLOAD_SIZE  186

// ═══════════════════════════════════════════════════════════════════
//  CRC-16/CCITT-FALSE
//  Poly=0x1021  Init=0xFFFF  RefIn=false  RefOut=false  XorOut=0x0000
// ═══════════════════════════════════════════════════════════════════
inline uint16_t crc16_ccitt(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc <<= 1;
    }
  }
  return crc;
}

// ═══════════════════════════════════════════════════════════════════
//  PARSER STATE MACHINE  (used for reading inbound CMD frames)
// ═══════════════════════════════════════════════════════════════════
namespace _comms {
  enum ParseState : uint8_t {
    WAIT_SYNC0,
    WAIT_SYNC1,
    WAIT_TYPE,
    WAIT_LEN,
    READ_PAYLOAD,
    WAIT_CRC0,
    WAIT_CRC1,
  };

  inline ParseState rxState   = WAIT_SYNC0;
  inline uint8_t    rxType    = 0;
  inline uint8_t    rxLen     = 0;
  inline uint8_t    rxCount   = 0;
  inline uint8_t    rxBuf[COMMS_MAX_PAYLOAD];
  inline uint8_t    rxCrc0    = 0;

  // Scratch buffer for CRC: [type, len, payload...]
  inline uint8_t    crcBuf[COMMS_MAX_PAYLOAD + 2];
}

// ── Public init (call once in setup) ────────────────────────────
inline void comms_init() {
  _comms::rxState = _comms::WAIT_SYNC0;
}

// ─────────────────────────────────────────────────────────────────
//  comms_readCommand()
//
//  Non-blocking. Call in loop — pass the Stream you want to drain.
//  Returns true (and fills cmd) each time a valid frame is decoded.
//  Call repeatedly until it returns false to drain the RX buffer.
// ─────────────────────────────────────────────────────────────────
inline bool comms_readCommand(Stream& s, HexCmd& cmd) {
  using namespace _comms;

  while (s.available()) {
    uint8_t b = (uint8_t)s.read();

    switch (rxState) {
      case WAIT_SYNC0:
        if (b == COMMS_SYNC0) rxState = WAIT_SYNC1;
        break;

      case WAIT_SYNC1:
        rxState = (b == COMMS_SYNC1) ? WAIT_TYPE : WAIT_SYNC0;
        break;

      case WAIT_TYPE:
        rxType  = b;
        rxState = WAIT_LEN;
        break;

      case WAIT_LEN:
        rxLen   = b;
        rxCount = 0;
        rxState = (rxLen == 0) ? WAIT_CRC0 : READ_PAYLOAD;
        // Start CRC buffer with [type, len]
        crcBuf[0] = rxType;
        crcBuf[1] = rxLen;
        break;

      case READ_PAYLOAD:
        if (rxCount < COMMS_MAX_PAYLOAD) {
          rxBuf[rxCount]         = b;
          crcBuf[2 + rxCount]    = b;
        }
        rxCount++;
        if (rxCount >= rxLen) rxState = WAIT_CRC0;
        break;

      case WAIT_CRC0:
        rxCrc0  = b;
        rxState = WAIT_CRC1;
        break;

      case WAIT_CRC1: {
        uint16_t received = ((uint16_t)rxCrc0 << 8) | b;
        uint16_t expected = crc16_ccitt(crcBuf, 2 + rxLen);
        rxState = WAIT_SYNC0;

        if (received != expected) break; // CRC mismatch, discard

        // ── Decode payload floats ─────────────────────────────
        cmd.type    = rxType;
        cmd.nFloats = rxLen / 4;
        for (uint8_t i = 0; i < cmd.nFloats && i < 8; i++) {
          uint32_t raw;
          memcpy(&raw, &rxBuf[i * 4], 4);
          memcpy(&cmd.f[i], &raw, 4);
        }
        return true;
      }
    }
  }
  return false;
}

// ─────────────────────────────────────────────────────────────────
//  comms_sendTelemetry()
//
//  Serialises HexTelemetry into a TEL_STATUS frame and writes it
//  to the given Stream (Serial2 / GP9 TX on the Pico).
// ─────────────────────────────────────────────────────────────────
inline void comms_sendTelemetry(Stream& s, const HexTelemetry& tel) {
  // Build payload buffer
  static uint8_t payload[TEL_PAYLOAD_SIZE];
  uint16_t idx = 0;

  // ── Flags (5 bytes) ────────────────────────────────────────────
  payload[idx++] = tel.mode;
  payload[idx++] = tel.gait;
  payload[idx++] = tel.safeMode;
  payload[idx++] = tel.stabilize;
  payload[idx++] = tel.pcaStatus;

  // ── IMU floats (9 × 4 = 36 bytes) ─────────────────────────────
  auto packFloat = [&](float v) {
    memcpy(&payload[idx], &v, 4);
    idx += 4;
  };
  packFloat(tel.imu_pitch);
  packFloat(tel.imu_roll);
  packFloat(tel.imu_yaw);
  packFloat(tel.imu_gx);
  packFloat(tel.imu_gy);
  packFloat(tel.imu_gz);
  packFloat(tel.imu_ax);
  packFloat(tel.imu_ay);
  packFloat(tel.imu_az);

  // ── Tibia tips (1 + 18×4 = 73 bytes) ──────────────────────────
  payload[idx++] = tel.tipContact;
  for (int i = 0; i < 6; i++) { packFloat(tel.tipX[i]); }
  for (int i = 0; i < 6; i++) { packFloat(tel.tipY[i]); }
  for (int i = 0; i < 6; i++) { packFloat(tel.tipZ[i]); }

  // ── Servo angles (18 × 4 = 72 bytes) ──────────────────────────
  for (int i = 0; i < 18; i++) { packFloat(tel.servoAngle[i]); }

  // Sanity check (static assert equivalent at runtime, only fires once)
  // idx should always equal TEL_PAYLOAD_SIZE here.

  // ── Compute CRC over [type, len, payload] ─────────────────────
  static uint8_t crcInput[TEL_PAYLOAD_SIZE + 2];
  crcInput[0] = TEL_STATUS;
  crcInput[1] = (uint8_t)TEL_PAYLOAD_SIZE;
  memcpy(&crcInput[2], payload, TEL_PAYLOAD_SIZE);
  uint16_t crc = crc16_ccitt(crcInput, TEL_PAYLOAD_SIZE + 2);

  // ── Write frame ───────────────────────────────────────────────
  s.write(COMMS_SYNC0);
  s.write(COMMS_SYNC1);
  s.write((uint8_t)TEL_STATUS);
  s.write((uint8_t)TEL_PAYLOAD_SIZE);
  s.write(payload, TEL_PAYLOAD_SIZE);
  s.write((uint8_t)(crc >> 8));
  s.write((uint8_t)(crc & 0xFF));
}

// ─────────────────────────────────────────────────────────────────
//  comms_sendAck()  (optional — send a 1-byte ACK for a command)
// ─────────────────────────────────────────────────────────────────
inline void comms_sendAck(Stream& s, uint8_t cmdType, uint8_t result) {
  uint8_t payload[2] = { cmdType, result };
  uint8_t crcBuf[4]  = { 0xA1, 2, cmdType, result }; // type=0xA1 for ACK
  uint16_t crc = crc16_ccitt(crcBuf, 4);

  s.write(COMMS_SYNC0);
  s.write(COMMS_SYNC1);
  s.write((uint8_t)0xA1);  // ACK type
  s.write((uint8_t)2);
  s.write(payload, 2);
  s.write((uint8_t)(crc >> 8));
  s.write((uint8_t)(crc & 0xFF));
}