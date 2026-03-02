#include "DCMotorWithEncoder.h"

DCMotorWithEncoder *DCMotorWithEncoder::_instances[4] = {nullptr, nullptr, nullptr, nullptr};

// ISR dispatch stubs
void DCMotorWithEncoder::_isr0() { if (_instances[0]) _instances[0]->_handleEncoder(); }
void DCMotorWithEncoder::_isr1() { if (_instances[1]) _instances[1]->_handleEncoder(); }
void DCMotorWithEncoder::_isr2() { if (_instances[2]) _instances[2]->_handleEncoder(); }
void DCMotorWithEncoder::_isr3() { if (_instances[3]) _instances[3]->_handleEncoder(); }

static void (*const _isrTable[4])() = {
    DCMotorWithEncoder::_isr0,
    DCMotorWithEncoder::_isr1,
    DCMotorWithEncoder::_isr2,
    DCMotorWithEncoder::_isr3
};

DCMotorWithEncoder::DCMotorWithEncoder(L293DShield &shield,
                                       uint8_t motorNum,
                                       int encoderPinA,
                                       int encoderPinB,
                                       float countsPerRevolution,
                                       float gearRatio,
                                       float maxMotorRPM)
    : _shield(shield),
      _motorNum(motorNum),
      _encoderPinA(encoderPinA),
      _encoderPinB(encoderPinB),
      _countsPerRev(countsPerRevolution),
      _gearRatio(gearRatio),
      _maxMotorRPM(maxMotorRPM) {
    // Register instance in the first available slot (slots 0–3 map to ISR 0–3)
    for (uint8_t i = 0; i < 4; i++) {
        if (_instances[i] == nullptr) {
            _instances[i] = this;
            pinMode(_encoderPinA, INPUT_PULLUP);
            pinMode(_encoderPinB, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(_encoderPinA), _isrTable[i], RISING);
            break;
        }
    }
}

// ── Speed control ────────────────────────────────────────────────────────────

void DCMotorWithEncoder::_applySpeed(float percentage) {
    _currentPWMPct = percentage;
    const int8_t dir = (percentage > 0) ? 1 : (percentage < 0) ? -1 : 0;
    const auto pwm = static_cast<uint8_t>(
        (percentage < 0 ? -percentage : percentage) * 255.0f / 100.0f + 0.5f);
    _shield.setMotorDirection(_motorNum, dir);
    _shield.setMotorSpeed(_motorNum, pwm);
}

void DCMotorWithEncoder::setSpeed(float percentage) {
    _targetActive = false;
    if (percentage > -0.5f && percentage < 0.5f) {
        // Treat as stop: disable closed-loop and coast to zero
        _closedLoopActive = false;
        _targetRPM    = 0.0f;
        _pidIntegral  = 0.0f;
        _pidLastError = 0.0f;
        _applySpeed(0.0f);
        return;
    }
    // Activate closed-loop RPM control; apply feedforward immediately so the
    // motor starts moving before the first PID tick (50 ms later).
    _closedLoopActive = true;
    _targetRPM    = (percentage / 100.0f) * _maxMotorRPM;
    _pidIntegral  = 0.0f;
    _pidLastError = 0.0f;
    _applySpeed(percentage);
}

void DCMotorWithEncoder::setSpeedRPM(float motorRPM) {
    setSpeed(motorRPM / _maxMotorRPM * 100.0f);
}

void DCMotorWithEncoder::setSpeedWheelRPM(float wheelRPM) {
    setSpeedRPM(wheelRPM * _gearRatio);
}

void DCMotorWithEncoder::setSpeed(Speed speed) {
    if (speed.isPercentage()) {
        setSpeed(speed.value);
    } else if (speed.isMotorRPM()) {
        setSpeedRPM(speed.value);
    } else {
        setSpeedWheelRPM(speed.value);
    }
}

void DCMotorWithEncoder::stop() {
    _targetActive     = false;
    _closedLoopActive = false;
    _targetRPM        = 0.0f;
    _pidIntegral      = 0.0f;
    _pidLastError     = 0.0f;
    _applySpeed(0.0f);
}

// ── Rotation control ─────────────────────────────────────────────────────────

void DCMotorWithEncoder::rotate(float revs, bool isWheelRevs, Speed speed) {
    const float motorRevs = isWheelRevs ? revs * _gearRatio : revs;
    const long delta = static_cast<long>(motorRevs * _countsPerRev + 0.5f);

    float pct;
    if (speed.isPercentage()) {
        pct = speed.value;
    } else if (speed.isMotorRPM()) {
        pct = speed.value / _maxMotorRPM * 100.0f;
    } else {
        pct = (speed.value * _gearRatio) / _maxMotorRPM * 100.0f;
    }

    // Negative revs → negative speed
    if (revs < 0) pct = -pct;

    _closedLoopActive = false;   // rotation is controlled by encoder count, not RPM
    _targetRPM        = 0.0f;
    _pidIntegral      = 0.0f;
    _targetCount      = _encoderCount + (revs >= 0 ? delta : -delta);
    _runningSpeedPct  = pct;
    _targetActive     = true;
    _applySpeed(pct);
}

void DCMotorWithEncoder::rotateMotorRevolutions(float revs, float speedPct) {
    rotate(revs, false, Speed::fromPercentage(speedPct));
}

void DCMotorWithEncoder::rotateMotorRevolutionsRPM(float revs, float motorRPM) {
    rotate(revs, false, Speed::fromMotorRPM(motorRPM));
}

void DCMotorWithEncoder::rotateWheelRevolutions(float revs, float speedPct) {
    rotate(revs, true, Speed::fromPercentage(speedPct));
}

void DCMotorWithEncoder::rotateWheelRevolutionsRPM(float revs, float motorRPM) {
    rotate(revs, true, Speed::fromMotorRPM(motorRPM));
}

// ── Encoder feedback ─────────────────────────────────────────────────────────

long DCMotorWithEncoder::getEncoderCount() {
    noInterrupts();
    long c = _encoderCount;
    interrupts();
    return c;
}

void DCMotorWithEncoder::resetEncoder() {
    noInterrupts();
    _encoderCount = 0;
    interrupts();
    _lastCount = 0;
    _lastRPMTime = millis();
}

float DCMotorWithEncoder::getMotorRPM() const { return _currentRPM; }
float DCMotorWithEncoder::getWheelRPM() { return _currentRPM / _gearRatio; }
bool DCMotorWithEncoder::isRunning() { return _targetActive; }

void DCMotorWithEncoder::update() {
    // ── RPM calculation + PID (at most every 50 ms) ──────────────────────────
    unsigned long now = millis();
    unsigned long elapsed = now - _lastRPMTime;
    if (elapsed >= 50) {
        long  count = getEncoderCount();
        long  diff  = count - _lastCount;
        float dt    = static_cast<float>(elapsed) / 60000.0f; // minutes
        _currentRPM  = static_cast<float>(diff) / _countsPerRev / dt;
        _lastCount   = count;
        _lastRPMTime = now;

        // ── Closed-loop PID ──────────────────────────────────────────────────
        if (_closedLoopActive) {
            float dtSec   = static_cast<float>(elapsed) / 1000.0f;
            float error   = _targetRPM - _currentRPM;

            _pidIntegral += error * dtSec;
            // Anti-windup: clamp integral contribution to ±50 % of full range
            float maxI = (_ki > 0.0f) ? (50.0f / _ki) : 500.0f;
            if (_pidIntegral >  maxI) _pidIntegral =  maxI;
            if (_pidIntegral < -maxI) _pidIntegral = -maxI;

            float derivative = (error - _pidLastError) / dtSec;
            _pidLastError    = error;

            // Feedforward (open-loop estimate) + PID correction
            float ff     = (_targetRPM / _maxMotorRPM) * 100.0f;
            float output = ff + _kp * error + _ki * _pidIntegral + _kd * derivative;

            // Clamp to valid PWM range
            if (output >  100.0f) output =  100.0f;
            if (output < -100.0f) output = -100.0f;

            _applySpeed(output);
        }
    }

    // ── Rotation target tracking ─────────────────────────────────────────────
    if (!_targetActive) return;

    const long count = getEncoderCount();
    const bool done  = (_runningSpeedPct >= 0) ? (count >= _targetCount)
                                               : (count <= _targetCount);
    if (done) {
        stop();
    }
}

void DCMotorWithEncoder::_handleEncoder() {
    // Channel B HIGH at rising edge of A → forward (+1), else reverse (-1)
    if (digitalRead(_encoderPinB) == HIGH) {
        _encoderCount++;
    } else {
        _encoderCount--;
    }
}

// ── Closed-loop helpers ───────────────────────────────────────────────────────

void DCMotorWithEncoder::setPIDGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

float DCMotorWithEncoder::getTargetRPM()     const { return _targetRPM; }
float DCMotorWithEncoder::getCurrentPWMPct() const { return _currentPWMPct; }

bool DCMotorWithEncoder::isSaturated() const {
    if (!_closedLoopActive) return false;
    if (_targetRPM > -0.1f && _targetRPM < 0.1f) return false;

    float absTarget = _targetRPM  < 0.0f ? -_targetRPM  : _targetRPM;
    float absActual = _currentRPM < 0.0f ? -_currentRPM : _currentRPM;
    float absPWM    = _currentPWMPct < 0.0f ? -_currentPWMPct : _currentPWMPct;

    return (absPWM    >= 99.0f)            &&   // at or near full power
           (absActual <  absTarget * 0.9f) &&   // noticeably below target
           (absActual >  absTarget * 0.05f);    // not a true stall
}

void DCMotorWithEncoder::adjustTargetRPM(float factor) {
    if (_closedLoopActive) {
        _targetRPM *= factor;
    }
}