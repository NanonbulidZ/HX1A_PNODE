/*
  HX1A_V100.ino
  Hexapod Firmware — Raspberry Pi Pico 2 (RP2350)
  Platform: PlatformIO + Arduino framework

  ── Architecture ────────────────────────────────────────────────────────────
  Tier 3 Nervous System node. Receives packed command frames from Brain Pi
  over Serial1 (GP0 RX / GP1 TX). Sends telemetry frames over Serial2
  (GP9 TX / GP8 RX). Both use the custom HEX binary protocol defined in
  comms.h / comms.cpp.

  ── Hardware Map ────────────────────────────────────────────────────────────
  I2C0   SDA=GP4  SCL=GP5   → Left PCA9685 (0x40) + Right PCA9685 (0x41)
  SPI1   MISO=GP12 MOSI=GP13 SCK=GP14 CS=GP15 → BMI160 IMU
  GP16–GP21                  → Tibia tip limit switches (active LOW)
  GP25                       → Onboard LED (fault codes)
  Serial1 GP0(TX) GP1(RX)   → Command input from Brain Pi
  Serial2 GP8(RX) GP9(TX)   → Telemetry output to Brain Pi

  ── PCA9685 Channel Map ─────────────────────────────────────────────────────
  Left  (0x40): Leg1 ch0-2 | Leg2 ch5-7 | Leg3 ch13-15
  Right (0x41): Leg0 ch0-2 | Leg4 ch5-7 | Leg5 ch13-15

  ── Boot Sequence ───────────────────────────────────────────────────────────
  1. Init LED fault system
  2. Init I2C + probe both PCA9685 with timeout → fault LED if missing
  3. Init SPI + probe BMI160                   → safe mode if missing
  4. Init tibia buttons
  5. Init UART command + telemetry ports
  6. Calibrate IMU baseline (if not safe mode)
  7. Enter main loop
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "pico/multicore.h"  // multicore_launch_core1, multicore_fifo_*
#include <Adafruit_PWMServoDriver.h>
#include "comms.h"   // Custom HEX protocol framer/parser
// BMI160 SPI driver implemented inline below (initBMI160, updateIMU, calibrateIMUBaseline)
#include "fault.h"   // LED fault blink patterns

// ═══════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════
// PIN_LED = 25 — defined in pins_arduino.h for rpipico2, no redefinition needed
#define PIN_BMI_CS       15
#define PIN_BMI_MISO     12
#define PIN_BMI_MOSI     13
#define PIN_BMI_SCK      14
#define PIN_I2C_SDA       4
#define PIN_I2C_SCL       5
#define PIN_CMD_RX        1   // Serial1 RX
#define PIN_CMD_TX        0   // Serial1 TX
#define PIN_TEL_RX        8   // Serial2 RX
#define PIN_TEL_TX        9   // Serial2 TX

// Tibia tip switches — GP16 to GP21 → legs 0–5
const uint8_t TIBIA_PINS[6] = {16, 17, 18, 19, 20, 21};

// ═══════════════════════════════════════════════════════════════════
//  PCA9685 SETTINGS
// ═══════════════════════════════════════════════════════════════════
Adafruit_PWMServoDriver pwmLeft  = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwmRight = Adafruit_PWMServoDriver(0x41);
const uint16_t SERVOMIN  = 150;
const uint16_t SERVOMAX  = 600;
const float    PWM_FREQ  = 50.0f;
#define I2C_PROBE_TIMEOUT_MS 200   // Max wait for I2C ACK before declaring fault

// ═══════════════════════════════════════════════════════════════════
//  SERVO SMOOTHING
// ═══════════════════════════════════════════════════════════════════
const float PWM_ACCEL_RATE   = 0.85f;
const float PWM_DECEL_RATE   = 0.75f;
const int   PWM_SMOOTH_SAMPLES = 4;

struct ServoSmoothing {
  float targetAngle;
  float currentAngle;
  float prevAngles[PWM_SMOOTH_SAMPLES];
  int   sampleIndex;
};
ServoSmoothing servoState[18]; // 6 legs × 3 joints

// ═══════════════════════════════════════════════════════════════════
//  ARC WALKING
// ═══════════════════════════════════════════════════════════════════
const float ARC_HEIGHT_MULTIPLIER = 0.15f;
const float ARC_TRAJECTORY_SMOOTH = 0.80f;

// ═══════════════════════════════════════════════════════════════════
//  SYSTEM FLAGS
// ═══════════════════════════════════════════════════════════════════
bool safeMode       = false;  // True when BMI160 not detected
bool pcaLeftOk      = false;
bool pcaRightOk     = false;
bool stabilizationOn = false;

// ═══════════════════════════════════════════════════════════════════
//  IMU STATE  (BMI160 via SPI)
// ═══════════════════════════════════════════════════════════════════
float imu_pitch = 0.0f, imu_roll = 0.0f, imu_yaw = 0.0f;
float imu_gx   = 0.0f, imu_gy   = 0.0f, imu_gz  = 0.0f;
float imu_ax   = 0.0f, imu_ay   = 0.0f, imu_az  = 0.0f;
float pitch_offset = 0.0f, roll_offset = 0.0f;
const float Z_STABILIZE_GAIN = 0.75f;

// Complementary filter state
float cf_pitch = 0.0f, cf_roll = 0.0f;
unsigned long imu_last_us = 0;

// ═══════════════════════════════════════════════════════════════════
//  TIBIA TIP STATE
// ═══════════════════════════════════════════════════════════════════
bool tibiaTip[6]       = {false};   // Current contact state
bool tibiaTipPrev[6]   = {false};   // Previous state for edge detection
bool tibiaTipEdge[6]   = {false};   // Rising edge (foot just touched down)
// Estimated XYZ contact position per leg (updated by IK when tip fires)
float tipX[6] = {0}, tipY[6] = {0}, tipZ[6] = {0};

// ═══════════════════════════════════════════════════════════════════
//  HEXAPOD KINEMATICS CONSTANTS
// ═══════════════════════════════════════════════════════════════════
const int   COXA_LENGTH  = 51;
const int   FEMUR_LENGTH = 65;
const int   TIBIA_LENGTH = 121;
const int   TRAVEL       = 30;
const long  A12DEG       = 209440;
const long  A30DEG       = 523599;
const int   FRAME_TIME_MS = 20;

const float HOME_X[6] = {  82.0f,   0.0f, -82.0f, -82.0f,   0.0f,  82.0f};
const float HOME_Y[6] = {  82.0f, 116.0f,  82.0f, -82.0f,-116.0f, -82.0f};
const float HOME_Z[6] = { -80.0f, -80.0f, -80.0f, -80.0f, -80.0f, -80.0f};

const float BODY_X[6] = { 110.4f,  0.0f,-110.4f,-110.4f,   0.0f, 110.4f};
const float BODY_Y[6] = {  58.4f, 90.8f,  58.4f, -58.4f, -90.8f, -58.4f};
const float BODY_Z[6] = {   0.0f,  0.0f,   0.0f,   0.0f,   0.0f,   0.0f};

const int   COXA_CAL[6]  = {0,0,0,0,0,0};
const int   FEMUR_CAL[6] = {0,0,0,0,0,0};
const int   TIBIA_CAL[6] = {0,0,0,0,0,0};

const float LEG_X_DIR[6]        = {1,1,1,1,1,1};
const float LEG_Y_DIR[6]        = {1,1,1,1,1,1};
float       LEG_STEP_OFFSET[6]  = {0,0,0,0,0,0};
float       LEG_ROT_OFFSET_DEG[6] = {0,135,225,0,135,225};

// ═══════════════════════════════════════════════════════════════════
//  MOTION STATE
// ═══════════════════════════════════════════════════════════════════
int   mode          = 0;
int   gait          = 0;   // 0=tripod 1=wave 2=ripple 3=tetrapod
int   gait_speed    = 0;
int   reset_position = true;
bool  capture_offsets = false;

float current_X[6], current_Y[6], current_Z[6];
float offset_X[6],  offset_Y[6],  offset_Z[6];
float step_height_multiplier = 1.5f;

// Gait phase arrays
int tripod_case[6]   = {1,2,1,2,1,2};
int ripple_case[6]   = {2,6,4,1,3,5};
int wave_case[6]     = {1,2,3,4,5,6};
int tetrapod_case[6] = {1,3,2,1,2,3};

int tick = 0, duration, numTicks;
int commandedX = 0, commandedY = 0, commandedR = 0;
int translateX, translateY, translateZ;
float strideX, strideY, strideR;
float sinRotX,sinRotY,sinRotZ,cosRotX,cosRotY,cosRotZ;
float rotOffsetX,rotOffsetY,rotOffsetZ;
float amplitudeX,amplitudeY,amplitudeZ;
float totalX_f, totalY_f, totalZ_f;
int   leg_num;

// IK workspace vars
float L0, L3;
float gamma_femur, phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

// Timing
unsigned long currentTime, previousTime;

// High-step mode
bool  highStepMode = false;
float saved_step_height_multiplier = 1.0f;
float saved_offset_Z[6] = {0};
const float HIGH_STEP_MULT    = 2.5f;
const float HIGH_BODY_LIFT    = 30.0f;
const float HIGHEST_STEP_MULT = 3.5f;
const float HIGHEST_BODY_LIFT = 50.0f;

// Leg overrides from UART commands
bool  legPosOverride[6]   = {false};
float legOverrideX[6]     = {0};
float legOverrideY[6]     = {0};
float legOverrideZ[6]     = {0};
bool  legDegOverride[6]   = {false};
float legOverrideCoxa[6]  = {0};
float legOverrideFemur[6] = {0};
float legOverrideTibia[6] = {0};

// ─────────────────────────────────────────────────────────────────
//  FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────────
bool     probeI2CDevice(uint8_t addr);
void     startI2CProbeCore1();
bool     initPCA();
void     attachServos();
uint16_t angleToPCA(float angle);
void     setPCAChannelAngle(uint8_t legNum, uint8_t segment, float angle);
void     updateServoSmoothing(int idx, float target);
void     initializeServoSmoothing();
void     writeLegServo(int leg, int seg, float angle);
float    computeArcZ(float phase, float baseAmp, bool swing);
void     leg_IK(int leg_number, float X, float Y, float Z);
void     compute_strides();
void     compute_amplitudes();
void     tripod_gait();
void     wave_gait();
void     ripple_gait();
void     tetrapod_gait();
void     translate_control();
void     rotate_control();
void     set_all_90();
void     updateTibiaTips();
void     applyAdaptiveGait();
void     updateIMU();
void     applyStabilization();
void     processCommand(const HexCmd& cmd);
void     sendTelemetry();
void     print_debug();

// ═══════════════════════════════════════════════════════════════════
//  I2C PROBE WITH TIMEOUT
// ═══════════════════════════════════════════════════════════════════
// Core1 worker — plain function required by multicore_launch_core1
static void i2cProbeCore1Entry() {
  while (true) {
    if (multicore_fifo_rvalid()) {
      uint8_t addr = (uint8_t)multicore_fifo_pop_blocking();
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      multicore_fifo_push_blocking((uint32_t)err);
    }
    tight_loop_contents();
  }
}

void startI2CProbeCore1() {
  multicore_launch_core1(i2cProbeCore1Entry);
  delay(10); // let core1 initialise
}

bool probeI2CDevice(uint8_t addr) {
  // Push address to core1, wait with millis() watchdog so fault_update()
  // keeps running even if endTransmission stalls on a floating bus.
  multicore_fifo_push_blocking((uint32_t)addr);
  uint32_t start = millis();
  while (!multicore_fifo_rvalid()) {
    fault_update();
    if (millis() - start > I2C_PROBE_TIMEOUT_MS) return false;
  }
  return (multicore_fifo_pop_blocking() == 0);
}

// ═══════════════════════════════════════════════════════════════════
//  PCA9685 INIT WITH FAULT DETECTION
// ═══════════════════════════════════════════════════════════════════
bool initPCA() {
  fault_update(); pcaLeftOk  = probeI2CDevice(0x40); fault_update();
  fault_update(); pcaRightOk = probeI2CDevice(0x41); fault_update();

  if (pcaLeftOk) {
    pwmLeft.begin();
    pwmLeft.setPWMFreq(PWM_FREQ);
  }
  if (pcaRightOk) {
    pwmRight.begin();
    pwmRight.setPWMFreq(PWM_FREQ);
  }

  Serial.printf("[PCA] Left(0x40)=%s  Right(0x41)=%s\n",
    pcaLeftOk?"OK":"MISSING", pcaRightOk?"OK":"MISSING");

  return pcaLeftOk && pcaRightOk;
}

// ═══════════════════════════════════════════════════════════════════
//  SERVO HELPERS
// ═══════════════════════════════════════════════════════════════════
void attachServos() {
  initializeServoSmoothing();
}

uint16_t angleToPCA(float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  return (uint16_t)(SERVOMIN + (angle / 180.0f) * (SERVOMAX - SERVOMIN));
}

void setPCAChannelAngle(uint8_t legNum, uint8_t segment, float angle) {
  Adafruit_PWMServoDriver* pca;
  uint8_t channel;
  bool    isRight = (legNum == 0 || legNum == 4 || legNum == 5);

  if (isRight) {
    if (!pcaRightOk) return;
    pca = &pwmRight;
    if      (legNum == 0) channel = segment;
    else if (legNum == 4) channel = 5 + segment;
    else                  channel = 13 + segment;
  } else {
    if (!pcaLeftOk) return;
    pca = &pwmLeft;
    if      (legNum == 1) channel = segment;
    else if (legNum == 2) channel = 5 + segment;
    else                  channel = 13 + segment;
  }

  pca->setPWM(channel, 0, angleToPCA(angle));
}

void updateServoSmoothing(int idx, float target) {
  ServoSmoothing* s = &servoState[idx];
  s->targetAngle = constrain(target, 0.0f, 180.0f);
  float diff = s->targetAngle - s->currentAngle;
  float rate = (diff > 0) ? PWM_ACCEL_RATE : PWM_DECEL_RATE;
  s->currentAngle += diff * rate;
  s->prevAngles[s->sampleIndex] = s->currentAngle;
  s->sampleIndex = (s->sampleIndex + 1) % PWM_SMOOTH_SAMPLES;
  float avg = 0;
  for (int i = 0; i < PWM_SMOOTH_SAMPLES; i++) avg += s->prevAngles[i];
  avg /= PWM_SMOOTH_SAMPLES;
  setPCAChannelAngle(idx / 3, idx % 3, avg);
}

void initializeServoSmoothing() {
  for (int i = 0; i < 18; i++) {
    servoState[i].targetAngle  = 90.0f;
    servoState[i].currentAngle = 90.0f;
    servoState[i].sampleIndex  = 0;
    for (int j = 0; j < PWM_SMOOTH_SAMPLES; j++)
      servoState[i].prevAngles[j] = 90.0f;
  }
}

void writeLegServo(int leg, int seg, float angle) {
  updateServoSmoothing(leg * 3 + seg, angle);
}

// ═══════════════════════════════════════════════════════════════════
//  TIBIA TIP SWITCHES
// ═══════════════════════════════════════════════════════════════════
void updateTibiaTips() {
  for (int i = 0; i < 6; i++) {
    tibiaTipPrev[i] = tibiaTip[i];
    tibiaTip[i]     = (digitalRead(TIBIA_PINS[i]) == LOW); // active LOW
    tibiaTipEdge[i] = (!tibiaTipPrev[i] && tibiaTip[i]);   // rising edge

    // When foot touches down, record the current IK target as contact position
    if (tibiaTipEdge[i]) {
      tipX[i] = current_X[i] + offset_X[i];
      tipY[i] = current_Y[i] + offset_Y[i];
      tipZ[i] = current_Z[i] + offset_Z[i];
    }
  }
}

/*
  Adaptive gait logic:
  If a leg's tibia tip fires during its swing phase (it shouldn't be touching
  the ground yet), it means the terrain is higher than expected at that point.
  We abort the swing early, freeze that leg at contact position, and
  compensate body Z to maintain level stance.
  Only active when NOT in safe mode.
*/
void applyAdaptiveGait() {
  if (safeMode) return;
  for (int i = 0; i < 6; i++) {
    // If leg is in swing (case 1) but tip already touching — abort swing
    bool inSwing = false;
    if (gait == 0) inSwing = (tripod_case[i]   == 1);
    if (gait == 1) inSwing = (wave_case[i]      == 1);
    if (gait == 2) inSwing = (ripple_case[i]    == 1 || ripple_case[i] == 2);
    if (gait == 3) inSwing = (tetrapod_case[i]  == 1);

    if (inSwing && tibiaTip[i]) {
      // Freeze leg at current position — transition directly to stance
      current_X[i] = tipX[i] - offset_X[i];
      current_Y[i] = tipY[i] - offset_Y[i];
      current_Z[i] = tipZ[i] - offset_Z[i];
      // Force transition to stance case
      if (gait == 0) tripod_case[i]   = 2;
      if (gait == 1) wave_case[i]     = 6;
      if (gait == 2) ripple_case[i]   = 3;
      if (gait == 3) tetrapod_case[i] = 2;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════
//  BMI160 VIA SPI  (bare-metal driver, no external lib)
// ═══════════════════════════════════════════════════════════════════
static SPISettings bmiSPISettings(10000000, MSBFIRST, SPI_MODE0);

static uint8_t bmiRead(uint8_t reg) {
  SPI1.beginTransaction(bmiSPISettings);
  digitalWrite(PIN_BMI_CS, LOW);
  SPI1.transfer(reg | 0x80);
  uint8_t val = SPI1.transfer(0x00);
  digitalWrite(PIN_BMI_CS, HIGH);
  SPI1.endTransaction();
  return val;
}

static void bmiWrite(uint8_t reg, uint8_t val) {
  SPI1.beginTransaction(bmiSPISettings);
  digitalWrite(PIN_BMI_CS, LOW);
  SPI1.transfer(reg & 0x7F);
  SPI1.transfer(val);
  digitalWrite(PIN_BMI_CS, HIGH);
  SPI1.endTransaction();
}

static void bmiReadBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
  SPI1.beginTransaction(bmiSPISettings);
  digitalWrite(PIN_BMI_CS, LOW);
  SPI1.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI1.transfer(0x00);
  digitalWrite(PIN_BMI_CS, HIGH);
  SPI1.endTransaction();
}

bool initBMI160() {
  pinMode(PIN_BMI_CS, OUTPUT);
  digitalWrite(PIN_BMI_CS, HIGH);

  // SPI1 on RP2350: MISO=GP12 MOSI=GP13 SCK=GP14
  SPI1.setRX(PIN_BMI_MISO);
  SPI1.setTX(PIN_BMI_MOSI);
  SPI1.setSCK(PIN_BMI_SCK);
  SPI1.begin();

  delay(10);

  // Chip ID check — BMI160 = 0xD1
  uint8_t chipId = bmiRead(0x00);
  Serial.printf("[BMI160] Chip ID: 0x%02X (expect 0xD1)\n", chipId);
  if (chipId != 0xD1) return false;

  // Soft reset
  bmiWrite(0x7E, 0xB6); delay(10);

  // Accel: normal power, 100Hz, OSR4
  bmiWrite(0x40, 0x28); // ACC_CONF: 100Hz, OSR4
  bmiWrite(0x41, 0x03); // ACC_RANGE: ±2g
  bmiWrite(0x7E, 0x11); // ACC normal mode
  delay(10);

  // Gyro: normal power, 100Hz
  bmiWrite(0x42, 0x28); // GYR_CONF: 100Hz, OSR4
  bmiWrite(0x43, 0x00); // GYR_RANGE: ±2000 dps
  bmiWrite(0x7E, 0x15); // GYR normal mode
  delay(80);

  return true;
}

void updateIMU() {
  if (safeMode) return;

  uint8_t buf[12];
  bmiReadBurst(0x0C, buf, 12); // GYR_X_L through ACC_Z_H

  int16_t gx_raw = (int16_t)((buf[1]  << 8) | buf[0]);
  int16_t gy_raw = (int16_t)((buf[3]  << 8) | buf[2]);
  int16_t gz_raw = (int16_t)((buf[5]  << 8) | buf[4]);
  int16_t ax_raw = (int16_t)((buf[7]  << 8) | buf[6]);
  int16_t ay_raw = (int16_t)((buf[9]  << 8) | buf[8]);
  int16_t az_raw = (int16_t)((buf[11] << 8) | buf[10]);

  // ±2000 dps → deg/s
  imu_gx = gx_raw / 16.384f;
  imu_gy = gy_raw / 16.384f;
  imu_gz = gz_raw / 16.384f;

  // ±2g → g
  imu_ax = ax_raw / 16384.0f;
  imu_ay = ay_raw / 16384.0f;
  imu_az = az_raw / 16384.0f;

  // Complementary filter — 95% gyro, 5% accel
  unsigned long now_us = micros();
  float dt = (now_us - imu_last_us) / 1e6f;
  imu_last_us = now_us;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;

  float accel_pitch = atan2f(imu_ay, imu_az) * RAD_TO_DEG;
  float accel_roll  = atan2f(-imu_ax, sqrtf(imu_ay*imu_ay + imu_az*imu_az)) * RAD_TO_DEG;

  cf_pitch = 0.95f * (cf_pitch + imu_gy * dt) + 0.05f * accel_pitch;
  cf_roll  = 0.95f * (cf_roll  + imu_gx * dt) + 0.05f * accel_roll;

  imu_pitch = cf_pitch - pitch_offset;
  imu_roll  = cf_roll  - roll_offset;
  imu_yaw  += imu_gz * dt; // Gyro-only yaw (no mag)
}

void calibrateIMUBaseline() {
  float sp = 0, sr = 0;
  const int N = 200;
  for (int i = 0; i < N; i++) {
    updateIMU();
    sp += cf_pitch;
    sr += cf_roll;
    delay(5);
  }
  pitch_offset = sp / N;
  roll_offset  = sr / N;
  Serial.printf("[IMU] Baseline — pitch=%.2f roll=%.2f\n", pitch_offset, roll_offset);
}

// ═══════════════════════════════════════════════════════════════════
//  ACTIVE STABILIZATION
// ═══════════════════════════════════════════════════════════════════
void applyStabilization() {
  if (safeMode || !stabilizationOn) return;

  // For each leg compute a Z offset that counters measured body tilt.
  // Legs on the "low" side get lifted, legs on "high" side get pushed down.
  // Uses small-angle approximation: dZ ≈ gain × pitch/roll × leg_Y/X position.
  for (int i = 0; i < 6; i++) {
    float dZ_pitch = Z_STABILIZE_GAIN * imu_pitch * (HOME_Y[i] / 116.0f);
    float dZ_roll  = Z_STABILIZE_GAIN * imu_roll  * (HOME_X[i] / 82.0f);
    current_Z[i] = HOME_Z[i] + offset_Z[i] - dZ_pitch - dZ_roll;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  UART COMMAND PROCESSING  (comms.h / HexCmd struct)
// ═══════════════════════════════════════════════════════════════════
void processCommand(const HexCmd& cmd) {
  switch (cmd.type) {
    case CMD_VEL:
      // Velocity X/Y/R — maps directly to commandedX/Y/R
      commandedX = (int)cmd.f[0];
      commandedY = (int)cmd.f[1];
      commandedR = (int)cmd.f[2];
      if (mode != 1) { mode = 1; reset_position = true; }
      break;

    case CMD_PITCH_ROLL:
      // Manual pitch/roll setpoint — only used if IMU in safe mode
      if (safeMode) {
        imu_pitch = cmd.f[0];
        imu_roll  = cmd.f[1];
      }
      break;

    case CMD_GAIT:
      gait = constrain((int)cmd.f[0], 0, 3);
      reset_position = true;
      Serial.printf("[CMD] Gait → %d\n", gait);
      break;

    case CMD_MODE:
      mode = (int)cmd.f[0];
      reset_position = true;
      Serial.printf("[CMD] Mode → %d\n", mode);
      break;

    case CMD_LEG_POS: {
      int leg = constrain((int)cmd.f[0], 0, 5);
      legPosOverride[leg]  = true;
      legDegOverride[leg]  = false; // Mutually exclusive
      legOverrideX[leg]    = cmd.f[1];
      legOverrideY[leg]    = cmd.f[2];
      legOverrideZ[leg]    = cmd.f[3];
      Serial.printf("[CMD] LegPos %d → X=%.1f Y=%.1f Z=%.1f\n",
        leg, cmd.f[1], cmd.f[2], cmd.f[3]);
      break;
    }

    case CMD_LEG_DEG: {
      int leg = constrain((int)cmd.f[0], 0, 5);
      legDegOverride[leg]   = true;
      legPosOverride[leg]   = false;
      legOverrideCoxa[leg]  = cmd.f[1];
      legOverrideFemur[leg] = cmd.f[2];
      legOverrideTibia[leg] = cmd.f[3];
      Serial.printf("[CMD] LegDeg %d → C=%.1f F=%.1f T=%.1f\n",
        leg, cmd.f[1], cmd.f[2], cmd.f[3]);
      break;
    }

    case CMD_RESET_OFFSETS:
      for (int i = 0; i < 6; i++) {
        offset_X[i] = offset_Y[i] = offset_Z[i] = 0;
        legPosOverride[i] = legDegOverride[i] = false;
      }
      step_height_multiplier = 1.0f;
      reset_position = true;
      break;

    case CMD_STABILIZE:
      stabilizationOn = (cmd.f[0] > 0.5f);
      Serial.printf("[CMD] Stabilization → %s\n", stabilizationOn?"ON":"OFF");
      break;

    case CMD_HIGH_STEP: {
      int level = (int)cmd.f[0]; // 0=off 1=high 2=highest
      if (level == 0) {
        highStepMode = false;
        step_height_multiplier = saved_step_height_multiplier;
        for (int i = 0; i < 6; i++) offset_Z[i] = saved_offset_Z[i];
      } else {
        if (!highStepMode) {
          highStepMode = true;
          saved_step_height_multiplier = step_height_multiplier;
          for (int i = 0; i < 6; i++) saved_offset_Z[i] = offset_Z[i];
        }
        float mult = (level == 1) ? HIGH_STEP_MULT : HIGHEST_STEP_MULT;
        float lift = (level == 1) ? HIGH_BODY_LIFT  : HIGHEST_BODY_LIFT;
        step_height_multiplier = mult;
        for (int i = 0; i < 6; i++) offset_Z[i] = saved_offset_Z[i] - lift;
      }
      break;
    }

    case CMD_GAIT_SPEED:
      gait_speed = (int)cmd.f[0];
      break;

    default:
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  TELEMETRY OUTPUT
// ═══════════════════════════════════════════════════════════════════
void sendTelemetry() {
  HexTelemetry tel;
  tel.mode         = (uint8_t)mode;
  tel.gait         = (uint8_t)gait;
  tel.safeMode     = (uint8_t)safeMode;
  tel.stabilize    = (uint8_t)stabilizationOn;
  tel.pcaStatus    = (uint8_t)((pcaLeftOk ? 0x01 : 0) | (pcaRightOk ? 0x02 : 0));
  tel.imu_pitch    = imu_pitch;
  tel.imu_roll     = imu_roll;
  tel.imu_yaw      = imu_yaw;
  tel.imu_gx       = imu_gx;
  tel.imu_gy       = imu_gy;
  tel.imu_gz       = imu_gz;
  tel.imu_ax       = imu_ax;
  tel.imu_ay       = imu_ay;
  tel.imu_az       = imu_az;
  tel.tipContact   = 0;
  for (int i = 0; i < 6; i++) {
    tel.tipContact |= (tibiaTip[i] ? (1 << i) : 0);
    tel.tipX[i]    = tipX[i];
    tel.tipY[i]    = tipY[i];
    tel.tipZ[i]    = tipZ[i];
    for (int j = 0; j < 3; j++) {
      tel.servoAngle[i*3+j] = servoState[i*3+j].currentAngle;
    }
  }
  comms_sendTelemetry(Serial2, tel);
}

// ═══════════════════════════════════════════════════════════════════
//  ARC WALKING HELPER
// ═══════════════════════════════════════════════════════════════════
float computeArcZ(float phase, float baseAmp, bool swing) {
  if (!swing) return 0.0f;
  float arcH = fabsf(baseAmp) * ARC_HEIGHT_MULTIPLIER;
  return fabsf(sinf(M_PI * phase)) * arcH * ARC_TRAJECTORY_SMOOTH;
}

// ═══════════════════════════════════════════════════════════════════
//  LEG INVERSE KINEMATICS
// ═══════════════════════════════════════════════════════════════════
void leg_IK(int leg_number, float X, float Y, float Z) {
  // If this leg has a direct degree override, bypass IK entirely
  if (legDegOverride[leg_number]) {
    writeLegServo(leg_number, 0, legOverrideCoxa[leg_number]);
    writeLegServo(leg_number, 1, legOverrideFemur[leg_number]);
    writeLegServo(leg_number, 2, legOverrideTibia[leg_number]);
    return;
  }

  L0 = sqrtf(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrtf(sq(L0) + sq(Z));

  if ((L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) && (L3 > fabsf(TIBIA_LENGTH - FEMUR_LENGTH))) {
    phi_tibia   = acosf((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3))
                        / (2.0f * FEMUR_LENGTH * TIBIA_LENGTH));
    theta_tibia = phi_tibia * RAD_TO_DEG - 23.0f + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia, 0.0f, 180.0f);

    gamma_femur = atan2f(Z, L0);
    phi_femur   = acosf((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH))
                        / (2.0f * FEMUR_LENGTH * L3));
    theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0f + 90.0f + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur, 0.0f, 180.0f);

    theta_coxa  = atan2f(X, Y) * RAD_TO_DEG + COXA_CAL[leg_number];

    // Per-leg coxa offset based on physical mounting angle
    const float coxaOffset[6] = {45.0f, 90.0f, 135.0f, -135.0f, -90.0f, -45.0f};
    // Handle wrap-around for rear legs
    if (leg_number >= 3 && theta_coxa < 0) theta_coxa += 360.0f;
    theta_coxa += coxaOffset[leg_number];
    if (leg_number >= 3) theta_coxa -= 180.0f; // Mirror right side
    theta_coxa = constrain(theta_coxa, 0.0f, 180.0f);

    writeLegServo(leg_number, 0, theta_coxa);
    writeLegServo(leg_number, 1, theta_femur);
    writeLegServo(leg_number, 2, theta_tibia);
  }
}

// ═══════════════════════════════════════════════════════════════════
//  STRIDE & AMPLITUDE COMPUTATION
// ═══════════════════════════════════════════════════════════════════
void compute_strides() {
  strideX = 90.0f * commandedX / 127.0f;
  strideY = 90.0f * commandedY / 127.0f;
  strideR = 35.0f * commandedR / 127.0f;

  sinRotZ = sinf(radians(strideR));
  cosRotZ = cosf(radians(strideR));

  duration = (gait_speed == 0) ? 1080 : 3240;
}

void compute_amplitudes() {
  totalX_f = HOME_X[leg_num] + BODY_X[leg_num];
  totalY_f = HOME_Y[leg_num] + BODY_Y[leg_num];

  rotOffsetX = totalY_f * sinRotZ + totalX_f * cosRotZ - totalX_f;
  rotOffsetY = totalY_f * cosRotZ - totalX_f * sinRotZ - totalY_f;

  float ang = radians(LEG_ROT_OFFSET_DEG[leg_num]);
  float sXr = strideX * cosf(ang) - strideY * sinf(ang);
  float sYr = strideX * sinf(ang) + strideY * cosf(ang);

  amplitudeX = constrain((sXr + rotOffsetX) / 2.0f, -50.0f, 50.0f);
  amplitudeY = constrain((sYr + rotOffsetY) / 2.0f, -50.0f, 50.0f);

  float totalStride = sqrtf(sq(sXr + rotOffsetX) + sq(sYr + rotOffsetY));
  amplitudeZ = (step_height_multiplier * totalStride / 4.0f) + LEG_STEP_OFFSET[leg_num];
  if (totalStride > 2.0f && amplitudeZ < 15.0f) amplitudeZ = 15.0f;
  amplitudeZ = constrain(amplitudeZ, -70.0f, 100.0f);
}

// ═══════════════════════════════════════════════════════════════════
//  GAITS
// ═══════════════════════════════════════════════════════════════════
void tripod_gait() {
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || tick > 0) {
    compute_strides();
    numTicks = round(duration / (float)FRAME_TIME_MS / 2.0f);
    for (leg_num = 0; leg_num < 6; leg_num++) {
      compute_amplitudes();
      float phase = (float)tick / numTicks;
      switch (tripod_case[leg_num]) {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cosf(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cosf(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + fabsf(amplitudeZ) * sinf(M_PI * tick / numTicks)
                               + computeArcZ(phase, amplitudeZ, true);
          if (tick >= numTicks - 1) tripod_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = HOME_X[leg_num] + amplitudeX * cosf(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY * cosf(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + computeArcZ(phase, amplitudeZ, false);
          if (tick >= numTicks - 1) tripod_case[leg_num] = 1;
          break;
      }
    }
    if (tick < numTicks - 1) tick++; else tick = 0;
  }
}

void wave_gait() {
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || tick > 0) {
    compute_strides();
    numTicks = round(duration / (float)FRAME_TIME_MS / 6.0f);
    for (leg_num = 0; leg_num < 6; leg_num++) {
      compute_amplitudes();
      float phase = (float)tick / numTicks;
      switch (wave_case[leg_num]) {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cosf(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cosf(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + fabsf(amplitudeZ) * sinf(M_PI * tick / numTicks)
                               + computeArcZ(phase, amplitudeZ, true);
          if (tick >= numTicks - 1) wave_case[leg_num] = 6;
          break;
        default:
          current_X[leg_num] -= amplitudeX / numTicks / 2.5f;
          current_Y[leg_num] -= amplitudeY / numTicks / 2.5f;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if (tick >= numTicks - 1)
            wave_case[leg_num] = (wave_case[leg_num] == 6) ? 1 : wave_case[leg_num] - 1;
          break;
      }
    }
    if (tick < numTicks - 1) tick++; else tick = 0;
  }
}

void ripple_gait() {
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || tick > 0) {
    compute_strides();
    numTicks = round(duration / (float)FRAME_TIME_MS / 6.0f);
    for (leg_num = 0; leg_num < 6; leg_num++) {
      compute_amplitudes();
      float phase;
      switch (ripple_case[leg_num]) {
        case 1:
          phase = (float)tick / (numTicks * 2);
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cosf(M_PI * tick / (numTicks * 2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cosf(M_PI * tick / (numTicks * 2));
          current_Z[leg_num] = HOME_Z[leg_num] + fabsf(amplitudeZ) * sinf(M_PI * tick / (numTicks * 2))
                               + computeArcZ(phase, amplitudeZ, true);
          if (tick >= numTicks - 1) ripple_case[leg_num] = 2;
          break;
        case 2:
          phase = (float)(numTicks + tick) / (numTicks * 2);
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cosf(M_PI * (numTicks + tick) / (numTicks * 2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cosf(M_PI * (numTicks + tick) / (numTicks * 2));
          current_Z[leg_num] = HOME_Z[leg_num] + fabsf(amplitudeZ) * sinf(M_PI * (numTicks + tick) / (numTicks * 2))
                               + computeArcZ(phase, amplitudeZ, true);
          if (tick >= numTicks - 1) ripple_case[leg_num] = 3;
          break;
        default:
          current_X[leg_num] -= amplitudeX / numTicks / 2.0f;
          current_Y[leg_num] -= amplitudeY / numTicks / 2.0f;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if (tick >= numTicks - 1)
            ripple_case[leg_num] = (ripple_case[leg_num] == 6) ? 1 : ripple_case[leg_num] + 1;
          break;
      }
    }
    if (tick < numTicks - 1) tick++; else tick = 0;
  }
}

void tetrapod_gait() {
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || tick > 0) {
    compute_strides();
    numTicks = round(duration / (float)FRAME_TIME_MS / 3.0f);
    for (leg_num = 0; leg_num < 6; leg_num++) {
      compute_amplitudes();
      float phase = (float)tick / numTicks;
      switch (tetrapod_case[leg_num]) {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cosf(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cosf(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + fabsf(amplitudeZ) * sinf(M_PI * tick / numTicks)
                               + computeArcZ(phase, amplitudeZ, true);
          if (tick >= numTicks - 1) tetrapod_case[leg_num] = 2;
          break;
        default:
          current_X[leg_num] -= amplitudeX / numTicks;
          current_Y[leg_num] -= amplitudeY / numTicks;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if (tick >= numTicks - 1)
            tetrapod_case[leg_num] = (tetrapod_case[leg_num] == 3) ? 1 : tetrapod_case[leg_num] + 1;
          break;
      }
    }
    if (tick < numTicks - 1) tick++; else tick = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  BODY TRANSLATE / ROTATE  (called via CMD_MODE 2 / 3)
// ═══════════════════════════════════════════════════════════════════
void translate_control() {
  for (int i = 0; i < 6; i++) {
    current_X[i] = HOME_X[i] + ((float)commandedX / 127.0f) * 2.0f * TRAVEL * LEG_X_DIR[i];
    current_Y[i] = HOME_Y[i] + ((float)commandedY / 127.0f) * 2.0f * TRAVEL * LEG_Y_DIR[i];
    current_Z[i] = HOME_Z[i];
  }
}

void rotate_control() {
  sinRotX = sinf(((float)commandedR / 127.0f) * radians(12.0f));
  cosRotX = cosf(((float)commandedR / 127.0f) * radians(12.0f));
  sinRotY = sinf(((float)commandedY / 127.0f) * radians(12.0f));
  cosRotY = cosf(((float)commandedY / 127.0f) * radians(12.0f));
  sinRotZ = sinf(((float)commandedX / 127.0f) * radians(30.0f));
  cosRotZ = cosf(((float)commandedX / 127.0f) * radians(30.0f));

  for (int i = 0; i < 6; i++) {
    float tx = HOME_X[i] + BODY_X[i];
    float ty = HOME_Y[i] + BODY_Y[i];
    float tz = HOME_Z[i] + BODY_Z[i];

    float rX = tx*cosRotY*cosRotZ + ty*sinRotX*sinRotY*cosRotZ + ty*cosRotX*sinRotZ
             - tz*cosRotX*sinRotY*cosRotZ + tz*sinRotX*sinRotZ - tx;
    float rY =-tx*cosRotY*sinRotZ - ty*sinRotX*sinRotY*sinRotZ + ty*cosRotX*cosRotZ
             + tz*cosRotX*sinRotY*sinRotZ + tz*sinRotX*cosRotZ - ty;
    float rZ = tx*sinRotY - ty*sinRotX*cosRotY + tz*cosRotX*cosRotY - tz;

    current_X[i] = HOME_X[i] + rX * LEG_X_DIR[i];
    current_Y[i] = HOME_Y[i] + rY * LEG_Y_DIR[i];
    current_Z[i] = HOME_Z[i] + rZ;
  }
}

void set_all_90() {
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 3; j++)
      writeLegServo(i, j, 90);
}

// ═══════════════════════════════════════════════════════════════════
//  DEBUG PRINT
// ═══════════════════════════════════════════════════════════════════
void print_debug() {
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug < 500) return; // Print at 2Hz max
  lastDebug = millis();

  Serial.printf("[HX1A] mode=%d gait=%d safe=%d stab=%d | "
                "cmdX=%d cmdY=%d cmdR=%d | tick=%d/%d\n",
                mode, gait, (int)safeMode, (int)stabilizationOn,
                commandedX, commandedY, commandedR, tick, numTicks);
  Serial.printf("  IMU  pitch=%.2f roll=%.2f yaw=%.2f\n",
                imu_pitch, imu_roll, imu_yaw);
  Serial.printf("  Tips [");
  for (int i = 0; i < 6; i++) Serial.printf("%d", (int)tibiaTip[i]);
  Serial.println("]");
  Serial.printf("  PCA  L=%s R=%s\n",
                pcaLeftOk?"OK":"FAIL", pcaRightOk?"OK":"FAIL");
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════
// ── Non-blocking blink helper used during setup ──────────────────
// Runs fault_update() for `ms` milliseconds so the LED keeps blinking
// even while we're waiting (e.g. for USB to enumerate).
static void blinkWait(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) fault_update();
}

void setup() {
  // ── 1. Fault LED first — so we can signal problems immediately ──
  fault_init(PIN_LED);
  fault_set(FAULT_OK); // Heartbeat while booting so we know Pico is alive

  // ── 2. USB Serial — wait up to 2 s for host to connect ──────────
  Serial.begin(115200);
  {
    uint32_t t = millis();
    while (!Serial && (millis() - t < 2000)) fault_update();
  }
  Serial.println("\n[HX1A] Booting v1.0.0 — RP2350");

  // ── 3. I2C init — probe runs on core1 to avoid blocking core0 ────
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.begin();
  gpio_pull_up(4); // SDA
  gpio_pull_up(5); // SCL
  Wire.setClock(100000); // Drop to 100kHz for stability without resistors

  startI2CProbeCore1(); // launches core1 I2C probe worker

  // ── 4. PCA9685 probe ─────────────────────────────────────────────
  bool pcaOk = initPCA();
  if (!pcaOk) {
    if (!pcaLeftOk && !pcaRightOk) fault_set(FAULT_BOTH_PCA);
    else if (!pcaLeftOk)           fault_set(FAULT_LEFT_PCA);
    else                           fault_set(FAULT_RIGHT_PCA);
    Serial.printf("[HX1A] PCA fault — L=%d R=%d\n", pcaLeftOk, pcaRightOk);
    // Don't freeze — continue booting, servos simply won't move
  }
  attachServos();
  blinkWait(50); // let LED state settle visually

  // ── 5. BMI160 SPI probe ──────────────────────────────────────────
  bool bmiOk = initBMI160();
  if (!bmiOk) {
    safeMode = true;
    Serial.println("[HX1A] !! SAFE MODE — BMI160 not detected !!");
    if (!pcaOk) fault_set(FAULT_NOTHING); // SOS: nothing works
    else        fault_set(FAULT_BMI);     // Long-short-short: IMU missing
  }

  // ── 6. Tibia tip buttons ─────────────────────────────────────────
  for (int i = 0; i < 6; i++) {
    pinMode(TIBIA_PINS[i], INPUT_PULLUP);
  }

  // ── 7. UART command port  (GP1=RX, GP0=TX) ──────────────────────
  Serial1.setTX(PIN_CMD_TX);
  Serial1.setRX(PIN_CMD_RX);
  Serial1.begin(115200);

  // ── 8. UART telemetry port  (GP9=TX, GP8=RX) ────────────────────
  Serial2.setTX(PIN_TEL_TX);
  Serial2.setRX(PIN_TEL_RX);
  Serial2.begin(460800);

  comms_init();

  // ── 9. IMU calibration (non-blocking loop so LED keeps blinking) ─
  if (!safeMode) {
    Serial.println("[IMU] Calibrating baseline — keep robot still...");
    imu_last_us = micros();
    // Replace blocking calibrateIMUBaseline() with inline non-blocking version
    float sp = 0, sr = 0;
    const int N = 200;
    for (int i = 0; i < N; i++) {
      updateIMU();
      sp += cf_pitch;
      sr += cf_roll;
      fault_update(); // keep LED blinking during calibration
      delay(5);
    }
    pitch_offset = sp / N;
    roll_offset  = sr / N;
    Serial.printf("[IMU] Baseline — pitch=%.2f roll=%.2f\n", pitch_offset, roll_offset);
  }

  // ── 10. Init motion state ────────────────────────────────────────
  for (int i = 0; i < 6; i++) {
    offset_X[i] = offset_Y[i] = offset_Z[i] = 0.0f;
    current_X[i] = HOME_X[i];
    current_Y[i] = HOME_Y[i];
    current_Z[i] = HOME_Z[i];
  }
  step_height_multiplier = 1.5f;
  mode = 0; gait = 0; gait_speed = 0;
  reset_position = true;
  previousTime = millis();

  // ── 11. Final fault state ────────────────────────────────────────
  if (pcaOk && !safeMode) fault_set(FAULT_OK);

  Serial.printf("[HX1A] Boot complete. safeMode=%d pcaL=%d pcaR=%d\n",
                (int)safeMode, (int)pcaLeftOk, (int)pcaRightOk);
}

// ═══════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════
void loop() {
  // ── 1. IMU — runs as fast as possible (not frame-locked)
  updateIMU();

  // ── 2. Fault LED blinker (non-blocking state machine)
  fault_update();

  // ── 3. Tibia tips
  updateTibiaTips();

  // ── 4. Parse inbound command frames (non-blocking)
  HexCmd cmd;
  while (comms_readCommand(Serial1, cmd)) {
    processCommand(cmd);
  }

  // ── 5. 50Hz gait frame
  currentTime = millis();
  if ((currentTime - previousTime) >= FRAME_TIME_MS) {
    previousTime = currentTime;

    // Adaptive gait — must run before gait state machines
    applyAdaptiveGait();

    // Reset position if requested
    if (reset_position) {
      for (int i = 0; i < 6; i++) {
        current_X[i] = HOME_X[i];
        current_Y[i] = HOME_Y[i];
        current_Z[i] = HOME_Z[i];
      }
      reset_position = false;
    }

    // Gait / mode dispatch
    if (mode == 1) {
      if (gait == 0) tripod_gait();
      if (gait == 1) wave_gait();
      if (gait == 2) ripple_gait();
      if (gait == 3) tetrapod_gait();
    }
    else if (mode == 2) translate_control();
    else if (mode == 3) rotate_control();
    else if (mode == 99) set_all_90();

    // Active stabilization — adjusts current_Z after gait
    applyStabilization();

    // IK solve for each leg
    for (int i = 0; i < 6; i++) {
      if (legPosOverride[i]) {
        // Direct XYZ override bypasses gait but still runs IK
        leg_IK(i, legOverrideX[i], legOverrideY[i], legOverrideZ[i]);
      } else {
        leg_IK(i,
               current_X[i] + offset_X[i],
               current_Y[i] + offset_Y[i],
               current_Z[i] + offset_Z[i]);
      }
    }

    // Telemetry — send every 4th frame (12.5Hz) to avoid saturating UART
    static uint8_t telDiv = 0;
    if (++telDiv >= 4) { telDiv = 0; sendTelemetry(); }

    print_debug();
  }
}