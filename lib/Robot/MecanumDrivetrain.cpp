#include "MecanumDrivetrain.h"
#include <math.h>

MecanumDrivetrain::MecanumDrivetrain(DCMotorWithEncoder &fl, DCMotorWithEncoder &fr,
                                     DCMotorWithEncoder &rl, DCMotorWithEncoder &rr,
                                     float trackWidth_mm, float wheelBase_mm)
    : _fl(fl), _fr(fr), _rl(rl), _rr(rr),
      _trackWidth(trackWidth_mm), _wheelBase(wheelBase_mm) {
}

// ── Kinematics ────────────────────────────────────────────────────────────────

//  v_FL =  Vy + Vx - ω
//  v_FR =  Vy - Vx + ω
//  v_RL =  Vy - Vx - ω
//  v_RR =  Vy + Vx + ω
//
// Values are normalised so max |v| == 1 before scaling by speedPct.

void MecanumDrivetrain::strafe(float x, float y, Speed speed) {
    strafe(x, y, 0.0f, speed);
}

void MecanumDrivetrain::strafe(float x, float y, float rotation, Speed speed) {
    const float fl = y + x - rotation;
    const float fr = y - x + rotation;
    const float rl = y - x - rotation;
    const float rr = y + x + rotation;

    const float speedPct = _resolveSpeedPct(speed);
    _applyWheelValues(fl, fr, rl, rr, speedPct);
}

void MecanumDrivetrain::circle(float radius_mm, Direction direction, Speed speed) {
    const float lEff = (_trackWidth + _wheelBase) / 4.0f;
    const float omega = lEff / radius_mm;

    float vx, omega_signed;
    if (direction == Direction::Left) {
        vx = +1.0f;
        omega_signed = +omega;
    } else {
        vx = -1.0f;
        omega_signed = -omega;
    }

    const float fl = 0.0f + vx - omega_signed;
    const float fr = 0.0f - vx + omega_signed;
    const float rl = 0.0f - vx - omega_signed;
    const float rr = 0.0f + vx + omega_signed;

    const float speedPct = _resolveSpeedPct(speed);
    _applyWheelValues(fl, fr, rl, rr, speedPct);
}

void MecanumDrivetrain::stop() {
    _motionActive = false;
    _fl.stop();
    _fr.stop();
    _rl.stop();
    _rr.stop();
}

void MecanumDrivetrain::update() {
    _fl.update();
    _fr.update();
    _rl.update();
    _rr.update();

    if (!_motionActive) return;

    // ── Saturation compensation ───────────────────────────────────────────────
    // Find the motor with the worst achievable-speed ratio.  A motor is
    // "relevantly saturated" when it is at full power, measurably spinning
    // (not stalled), but still below its RPM target.
    float minRatio = _compensationFactor;

    if (_fl.isSaturated()) {
        float r = (_fl.getMotorRPM() < 0 ? -_fl.getMotorRPM() : _fl.getMotorRPM())
                / (_fl.getTargetRPM() < 0 ? -_fl.getTargetRPM() : _fl.getTargetRPM());
        if (r < minRatio) minRatio = r;
    }
    if (_fr.isSaturated()) {
        float r = (_fr.getMotorRPM() < 0 ? -_fr.getMotorRPM() : _fr.getMotorRPM())
                / (_fr.getTargetRPM() < 0 ? -_fr.getTargetRPM() : _fr.getTargetRPM());
        if (r < minRatio) minRatio = r;
    }
    if (_rl.isSaturated()) {
        float r = (_rl.getMotorRPM() < 0 ? -_rl.getMotorRPM() : _rl.getMotorRPM())
                / (_rl.getTargetRPM() < 0 ? -_rl.getTargetRPM() : _rl.getTargetRPM());
        if (r < minRatio) minRatio = r;
    }
    if (_rr.isSaturated()) {
        float r = (_rr.getMotorRPM() < 0 ? -_rr.getMotorRPM() : _rr.getMotorRPM())
                / (_rr.getTargetRPM() < 0 ? -_rr.getTargetRPM() : _rr.getTargetRPM());
        if (r < minRatio) minRatio = r;
    }

    // Apply proportional adjustment to all motors when needed.
    if (minRatio < _compensationFactor - 0.01f) {
        float factor = minRatio / _compensationFactor;
        _fl.adjustTargetRPM(factor);
        _fr.adjustTargetRPM(factor);
        _rl.adjustTargetRPM(factor);
        _rr.adjustTargetRPM(factor);
        _compensationFactor = minRatio;
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

float MecanumDrivetrain::_resolveSpeedPct(Speed speed) const {
    if (speed.isPercentage()) return speed.value;
    // For RPM-based speeds, use FL motor as representative
    // (all motors assumed identical in this implementation)
    if (speed.isMotorRPM()) {
        // maxMotorRPM retrieved via getWheelRPM comparison isn't accessible,
        // so we use Speed::fromMotorRPM and let each motor resolve it.
        // Here we just return the percentage equivalent using FL's params.
        return speed.value; // caller uses setSpeed(Speed) directly when RPM matters
    }
    return speed.value;
}

void MecanumDrivetrain::_applyWheelValues(float fl, float fr, float rl, float rr,
                                          float speedPct) {
    // Normalise so max absolute value is 1
    float maxVal = fabs(fl);
    if (fabs(fr) > maxVal) maxVal = fabs(fr);
    if (fabs(rl) > maxVal) maxVal = fabs(rl);
    if (fabs(rr) > maxVal) maxVal = fabs(rr);

    if (maxVal > 1.0f) {
        fl /= maxVal;
        fr /= maxVal;
        rl /= maxVal;
        rr /= maxVal;
    }

    // Persist normalised fractions so update() can re-apply compensation.
    _lastFL = fl;
    _lastFR = fr;
    _lastRL = rl;
    _lastRR = rr;
    _lastSpeedPct       = speedPct;
    _compensationFactor = 1.0f;   // new user command resets any previous scaling
    _motionActive       = true;

    _fl.setSpeed(fl * speedPct);
    _fr.setSpeed(fr * speedPct);
    _rl.setSpeed(rl * speedPct);
    _rr.setSpeed(rr * speedPct);
}
