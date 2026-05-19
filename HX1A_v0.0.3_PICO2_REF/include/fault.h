/*
  fault.h — Non-blocking LED fault code blinker
  Pico 2 onboard LED = GP25
*/

#pragma once
#include <Arduino.h>

// ── Fault codes ──────────────────────────────────────────────────
enum FaultCode {
  FAULT_NONE       = 0,
  FAULT_OK         = 1,  // Heartbeat: ··
  FAULT_BMI        = 2,  // IMU missing: L··
  FAULT_LEFT_PCA   = 3,  // Left PCA dead: slow double blink
  FAULT_RIGHT_PCA  = 4,  // Right PCA dead: single fast blink
  FAULT_BOTH_PCA   = 5,  // Both PCAs dead: triple rapid blink
  FAULT_NOTHING    = 6,  // SOS: ···---···
};

struct BlinkStep {
  uint16_t on_ms;
  uint16_t off_ms;
};

// Timing constants
#define S_ON  120
#define S_OFF 150
#define L_ON  400
#define L_OFF 150
#define W_GAP 1000 

// ── Patterns ─────────────────────────────────────────────────────
static const BlinkStep PAT_OK[] = {
  {80, 80}, {80, 2800}, {0, 0}
};

static const BlinkStep PAT_BMI[] = {
  {L_ON, L_OFF}, {S_ON, S_OFF}, {S_ON, W_GAP}, {0, 0}
};

static const BlinkStep PAT_LEFT_PCA[] = {
  {300, 300}, {300, 1500}, {0, 0}
};

static const BlinkStep PAT_RIGHT_PCA[] = {
  {100, 1500}, {0, 0}
};

static const BlinkStep PAT_BOTH_PCA[] = {
  {100, 100}, {100, 100}, {100, 1000}, {0, 0}
};

static const BlinkStep PAT_SOS[] = {
  {S_ON, S_OFF}, {S_ON, S_OFF}, {S_ON, S_OFF},
  {L_ON, L_OFF}, {L_ON, L_OFF}, {L_ON, L_OFF},
  {S_ON, S_OFF}, {S_ON, S_OFF}, {S_ON, W_GAP},
  {0, 0}
};

// ── Internal State ───────────────────────────────────────────────
// Use 'inline' for C++17 to ensure a single instance across includes
inline uint8_t           _ledPin      = 25;
inline FaultCode         _faultCode   = FAULT_NONE;
inline const BlinkStep*  _pattern     = nullptr;
inline int               _step        = 0;
inline bool              _ledOn       = false;
inline unsigned long     _lastChange  = 0;

static const BlinkStep* _getPattern(FaultCode code) {
  switch (code) {
    case FAULT_OK:        return PAT_OK;
    case FAULT_BMI:       return PAT_BMI;
    case FAULT_LEFT_PCA:  return PAT_LEFT_PCA;
    case FAULT_RIGHT_PCA: return PAT_RIGHT_PCA;
    case FAULT_BOTH_PCA:  return PAT_BOTH_PCA;
    case FAULT_NOTHING:   return PAT_SOS;
    default:              return nullptr;
  }
}

// ── Public API ───────────────────────────────────────────────────

inline void fault_init(uint8_t ledPin = 25) {
  _ledPin = ledPin;
  pinMode(_ledPin, OUTPUT);
  digitalWrite(_ledPin, LOW);
}

inline void fault_set(FaultCode code) {
  if (_faultCode == code) return; // No change

  _faultCode  = code;
  _pattern    = _getPattern(code);
  _step       = 0;
  _lastChange = millis();

  if (_pattern) {
    _ledOn = true;
    digitalWrite(_ledPin, HIGH);
  } else {
    _ledOn = false;
    digitalWrite(_ledPin, LOW);
  }
}

inline void fault_update() {
  if (!_pattern) return;

  unsigned long now = millis();
  uint16_t wait = _ledOn ? _pattern[_step].on_ms : _pattern[_step].off_ms;

  if (now - _lastChange >= wait) {
    _lastChange = now;
    _ledOn = !_ledOn; // Toggle state

    if (_ledOn) {
      // Starting the ON phase of the current step
      digitalWrite(_ledPin, HIGH);
    } else {
      // Finished the ON phase — turn off, then advance to next step
      digitalWrite(_ledPin, LOW);
      _step++;
      if (_pattern[_step].on_ms == 0) _step = 0; // Wrap sequence
    }
  }
}

inline FaultCode fault_get() { 
    return _faultCode; 
}

inline void fault_clear() { 
    fault_set(FAULT_NONE); 
}