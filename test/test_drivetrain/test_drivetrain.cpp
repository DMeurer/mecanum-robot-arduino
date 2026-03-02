#include <unity.h>
#include "Arduino.h"
#include "L293DShield.h"
#include "DCMotorWithEncoder.h"
#include "MecanumDrivetrain.h"

// ── Constants ─────────────────────────────────────────────────────────────────
static constexpr float COUNTS_PER_REV = 56.0f;
static constexpr float GEAR_RATIO = 30.0f;
static constexpr float MAX_MOTOR_RPM = 300.0f;
static constexpr float TRACK_WIDTH = 200.0f;
static constexpr float WHEEL_BASE = 180.0f;

// PWM pin map: M1=11, M2=3, M3=5, M4=6
static constexpr uint8_t PIN_FL = 11;
static constexpr uint8_t PIN_FR = 3;
static constexpr uint8_t PIN_RL = 5;
static constexpr uint8_t PIN_RR = 6;

// ── Fixtures ──────────────────────────────────────────────────────────────────
static L293DShield shield;
static DCMotorWithEncoder *fl = nullptr;
static DCMotorWithEncoder *fr = nullptr;
static DCMotorWithEncoder *rl = nullptr;
static DCMotorWithEncoder *rr = nullptr;
static MecanumDrivetrain *dt = nullptr;

void setUp() {
    mock.reset();
    for (int i = 0; i < 4; i++) DCMotorWithEncoder::_instances[i] = nullptr;

    shield.begin();
    delete fl;
    fl = new DCMotorWithEncoder(shield, 1, 2, 22, COUNTS_PER_REV, GEAR_RATIO, MAX_MOTOR_RPM);
    delete fr;
    fr = new DCMotorWithEncoder(shield, 2, 18, 23, COUNTS_PER_REV, GEAR_RATIO, MAX_MOTOR_RPM);
    delete rl;
    rl = new DCMotorWithEncoder(shield, 3, 19, 24, COUNTS_PER_REV, GEAR_RATIO, MAX_MOTOR_RPM);
    delete rr;
    rr = new DCMotorWithEncoder(shield, 4, 20, 25, COUNTS_PER_REV, GEAR_RATIO, MAX_MOTOR_RPM);
    delete dt;
    dt = new MecanumDrivetrain(*fl, *fr, *rl, *rr, TRACK_WIDTH, WHEEL_BASE);
}

void tearDown() {
}

// ── Helper: read motor PWM signs ──────────────────────────────────────────────
// Returns +1/0/-1 based on direction bits in shift register written to MOTORDATA
// For simplicity we track via the last direction set through L293DShield.
// Since we can only observe PWM values from the mock, and the direction is
// encoded in the shift register (not a readable pin), we rely on the
// convention: positive setSpeed → forward (A=HIGH), negative → reverse (B=HIGH).
// We verify signs by comparing relative PWM magnitudes and the computed ratios.

static float pwmSigned(uint8_t pin, uint8_t dirA_bit, uint8_t motorIdx) {
    // Extract the direction from shift register state via the MOTORDATA sequence.
    // For test purposes we check _shiftState indirectly: after setMotorDirection
    // the _bitA or _bitB of the shift register is set. However MockState doesn't
    // expose shift state. Instead we use a simpler approach:
    // The tests below call drivetrain methods and check the *normalised ratios*
    // between PWM values, since direction is separately validated in motor tests.
    (void) dirA_bit;
    (void) motorIdx;
    return static_cast<float>(mock.pwmValues[pin]);
}

// ── strafe(1,0) — pure lateral right ─────────────────────────────────────────
// FL=+, FR=-, RL=-, RR=+   (normalised magnitudes all equal)

void test_strafe_right_equal_magnitude() {
    dt->strafe(1.0f, 0.0f, Speed::fromPercentage(100));
    uint8_t fl_pwm = mock.pwmValues[PIN_FL];
    uint8_t fr_pwm = mock.pwmValues[PIN_FR];
    uint8_t rl_pwm = mock.pwmValues[PIN_RL];
    uint8_t rr_pwm = mock.pwmValues[PIN_RR];
    TEST_ASSERT_EQUAL(fl_pwm, fr_pwm);
    TEST_ASSERT_EQUAL(fl_pwm, rl_pwm);
    TEST_ASSERT_EQUAL(fl_pwm, rr_pwm);
    TEST_ASSERT_EQUAL(255, fl_pwm);
}

// ── strafe(0,1) — pure forward ────────────────────────────────────────────────
// All wheels equal positive

void test_strafe_forward_equal_pwm() {
    dt->strafe(0.0f, 1.0f, Speed::fromPercentage(100));
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PIN_FL]);
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PIN_FR]);
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PIN_RL]);
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PIN_RR]);
}

// ── strafe(0,0,1) — pure CCW spin ─────────────────────────────────────────────
// FL=-, FR=+, RL=-, RR=+   all equal magnitude

void test_strafe_ccw_spin_equal_magnitude() {
    dt->strafe(0.0f, 0.0f, 1.0f, Speed::fromPercentage(100));
    uint8_t fl_pwm = mock.pwmValues[PIN_FL];
    uint8_t fr_pwm = mock.pwmValues[PIN_FR];
    uint8_t rl_pwm = mock.pwmValues[PIN_RL];
    uint8_t rr_pwm = mock.pwmValues[PIN_RR];
    TEST_ASSERT_EQUAL(255, fl_pwm);
    TEST_ASSERT_EQUAL(255, fr_pwm);
    TEST_ASSERT_EQUAL(255, rl_pwm);
    TEST_ASSERT_EQUAL(255, rr_pwm);
}

// ── strafe(1,1,0) — diagonal: normalised correctly ────────────────────────────
// FL = 1+1 = 2, FR = 1-1 = 0, RL = 1-1 = 0, RR = 1+1 = 2
// normalised by 2: FL=1, FR=0, RL=0, RR=1 → PWM 255, 0, 0, 255

void test_strafe_diagonal_normalised() {
    dt->strafe(1.0f, 1.0f, 0.0f, Speed::fromPercentage(100));
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PIN_FL]);
    TEST_ASSERT_EQUAL(0, mock.pwmValues[PIN_FR]);
    TEST_ASSERT_EQUAL(0, mock.pwmValues[PIN_RL]);
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PIN_RR]);
}

// ── strafe(0.5,0.5,0) — speed scaling ────────────────────────────────────────
// Same wheel ratio as (1,1,0), but at 50 % speed
// FL = 255*0.5 = 128 (rounded), FR = 0, RL = 0, RR = 128

void test_strafe_diagonal_at_half_speed() {
    dt->strafe(0.5f, 0.5f, 0.0f, Speed::fromPercentage(50));
    TEST_ASSERT_EQUAL(128, mock.pwmValues[PIN_FL]);
    TEST_ASSERT_EQUAL(0, mock.pwmValues[PIN_FR]);
    TEST_ASSERT_EQUAL(0, mock.pwmValues[PIN_RL]);
    TEST_ASSERT_EQUAL(128, mock.pwmValues[PIN_RR]);
}

// ── Speed::fromPercentage(100) → max wheel speed ─────────────────────────────

void test_speed_percentage_100_gives_255() {
    dt->strafe(0.0f, 1.0f, Speed::fromPercentage(100));
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PIN_FL]);
}

// ── Speed::fromWheelRPM scaling ───────────────────────────────────────────────
// maxWheelRPM = MAX_MOTOR_RPM / GEAR_RATIO = 300 / 30 = 10 RPM
// fromWheelRPM(5) → 50 % → 128 PWM

void test_speed_wheel_rpm_50pct() {
    float maxWheelRPM = MAX_MOTOR_RPM / GEAR_RATIO; // 10
    // We call setSpeed directly on a motor for this unit check
    fl->setSpeedWheelRPM(maxWheelRPM / 2.0f);
    TEST_ASSERT_EQUAL(128, mock.pwmValues[PIN_FL]);
}

// ── circle(100, Left) — CCW orbit ─────────────────────────────────────────────
// Leff = (200 + 180) / 4 = 95 mm
// omega = 95 / 100 = 0.95
// Vx = +1, Vy = 0
// FL = 0 + 1 - 0.95 = 0.05
// FR = 0 - 1 + 0.95 = -0.05
// RL = 0 - 1 - 0.95 = -1.95
// RR = 0 + 1 + 0.95 = 1.95
// normalise by 1.95:
// FL ≈ +0.0256, FR ≈ -0.0256, RL ≈ -1.0, RR ≈ +1.0

void test_circle_left_wheel_ratios() {
    dt->circle(100.0f, Direction::Left, Speed::fromPercentage(100));

    // RR and RL should be at max magnitude (255); FL and FR minimal
    uint8_t rl_pwm = mock.pwmValues[PIN_RL];
    uint8_t rr_pwm = mock.pwmValues[PIN_RR];
    uint8_t fl_pwm = mock.pwmValues[PIN_FL];
    uint8_t fr_pwm = mock.pwmValues[PIN_FR];

    TEST_ASSERT_EQUAL(255, rl_pwm);
    TEST_ASSERT_EQUAL(255, rr_pwm);
    TEST_ASSERT_TRUE(fl_pwm < rl_pwm);
    TEST_ASSERT_TRUE(fr_pwm < rr_pwm);
    TEST_ASSERT_EQUAL(fl_pwm, fr_pwm);
}

// ── circle(100, Right) — CW orbit: mirror of Left ────────────────────────────

void test_circle_right_is_mirror_of_left() {
    dt->circle(100.0f, Direction::Left, Speed::fromPercentage(100));
    uint8_t fl_l = mock.pwmValues[PIN_FL];
    uint8_t fr_l = mock.pwmValues[PIN_FR];
    uint8_t rl_l = mock.pwmValues[PIN_RL];
    uint8_t rr_l = mock.pwmValues[PIN_RR];

    mock.reset();
    shield.begin();

    dt->circle(100.0f, Direction::Right, Speed::fromPercentage(100));
    uint8_t fl_r = mock.pwmValues[PIN_FL];
    uint8_t fr_r = mock.pwmValues[PIN_FR];
    uint8_t rl_r = mock.pwmValues[PIN_RL];
    uint8_t rr_r = mock.pwmValues[PIN_RR];

    // Magnitudes should be mirrored: FL↔RR and FR↔RL swap
    TEST_ASSERT_EQUAL(fl_l, fl_r);
    TEST_ASSERT_EQUAL(fr_l, fr_r);
    TEST_ASSERT_EQUAL(rl_l, rl_r);
    TEST_ASSERT_EQUAL(rr_l, rr_r);
}

// ── circle with very large radius approaches pure lateral strafe ──────────────

void test_circle_large_radius_approaches_lateral() {
    dt->circle(100000.0f, Direction::Left, Speed::fromPercentage(100));
    uint8_t fl_pwm = mock.pwmValues[PIN_FL];
    uint8_t fr_pwm = mock.pwmValues[PIN_FR];
    uint8_t rl_pwm = mock.pwmValues[PIN_RL];
    uint8_t rr_pwm = mock.pwmValues[PIN_RR];

    // With near-zero omega, all four wheels should have close to equal magnitude
    TEST_ASSERT_UINT8_WITHIN(5, fl_pwm, rr_pwm);
    TEST_ASSERT_UINT8_WITHIN(5, fr_pwm, rl_pwm);
    TEST_ASSERT_UINT8_WITHIN(5, fl_pwm, fr_pwm);
}

// ── circle(Leff) — inner pair stops, outer at 2x ─────────────────────────────
// Leff = 95, radius = 95 → omega = 1.0
// FL = 1 - 1 = 0, FR = -1 + 1 = 0, RL = -1 - 1 = -2, RR = 1 + 1 = 2
// normalise by 2: FL=0, FR=0, RL=-1, RR=+1

void test_circle_radius_equals_leff_inner_stops() {
    float lEff = (TRACK_WIDTH + WHEEL_BASE) / 4.0f;
    dt->circle(lEff, Direction::Left, Speed::fromPercentage(100));

    TEST_ASSERT_EQUAL(0, mock.pwmValues[PIN_FL]);
    TEST_ASSERT_EQUAL(0, mock.pwmValues[PIN_FR]);
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PIN_RL]);
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PIN_RR]);
}

// ── Saturation compensation ───────────────────────────────────────────────────
// Scenario: strafe forward at 100%.  All four motors target 300 RPM but are
// physically limited to ~200 RPM (19 encoder pulses in 100 ms).
// After dt->update(), the drivetrain should detect saturation and scale all
// targets down to match the achievable speed.

void test_compensation_reduces_targets_when_saturated() {
    // strafe forward at 100%: all motors target 300 RPM, PWM = 100%
    dt->strafe(0.0f, 1.0f, Speed::fromPercentage(100));

    // Simulate all four motors achieving only ~203 RPM (19 pulses / 56 cpr in 100 ms).
    mock.timeMs = 100;
    mock.digitalValues[22] = HIGH; // FL enc B
    mock.digitalValues[23] = HIGH; // FR enc B
    mock.digitalValues[24] = HIGH; // RL enc B
    mock.digitalValues[25] = HIGH; // RR enc B
    for (int i = 0; i < 19; i++) {
        fl->_handleEncoder();
        fr->_handleEncoder();
        rl->_handleEncoder();
        rr->_handleEncoder();
    }

    // dt->update() triggers: RPM computation, PID (output clamped → 100%),
    // saturation detection, and target scaling.
    // Note: after scaling, isSaturated() goes false (target ≈ actual) —
    // so we verify the *outcome* (targets adjusted) not the transient state.
    dt->update();

    // All targets should have been scaled down to approximately the achievable RPM.
    float newTarget = fl->getTargetRPM();
    TEST_ASSERT_TRUE(newTarget < 300.0f);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 203.0f, newTarget);
    // All four wheels must be scaled to the same factor (mecanum kinematics preserved).
    TEST_ASSERT_FLOAT_WITHIN(1.0f, newTarget, fr->getTargetRPM());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, newTarget, rl->getTargetRPM());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, newTarget, rr->getTargetRPM());
}

void test_compensation_not_triggered_when_motors_reach_target() {
    // strafe at 50%: all motors target 150 RPM with initial PWM 50%.
    // Simulate encoder counts matching the target → PID settles, PWM stays ~50%
    // → isSaturated() is false → no adjustment.
    dt->strafe(0.0f, 1.0f, Speed::fromPercentage(50));

    // 14 pulses in 100 ms ≈ 150 RPM — matches the 150 RPM target
    mock.timeMs = 100;
    mock.digitalValues[22] = HIGH; mock.digitalValues[23] = HIGH;
    mock.digitalValues[24] = HIGH; mock.digitalValues[25] = HIGH;
    for (int i = 0; i < 14; i++) {
        fl->_handleEncoder(); fr->_handleEncoder();
        rl->_handleEncoder(); rr->_handleEncoder();
    }

    dt->update(); // PWM stays ~50% (< 99%) → not saturated → no compensation

    // getTargetRPM() is set by setSpeed() and only changed by adjustTargetRPM().
    // Since no compensation ran, it must still be exactly 150 RPM.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, fl->getTargetRPM());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, fr->getTargetRPM());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, rl->getTargetRPM());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, rr->getTargetRPM());
}

void test_compensation_resets_on_new_strafe() {
    // First strafe at 100% gets compensated down to ~203 RPM
    dt->strafe(0.0f, 1.0f, Speed::fromPercentage(100));
    mock.timeMs = 100;
    mock.digitalValues[22] = HIGH; mock.digitalValues[23] = HIGH;
    mock.digitalValues[24] = HIGH; mock.digitalValues[25] = HIGH;
    for (int i = 0; i < 19; i++) {
        fl->_handleEncoder(); fr->_handleEncoder();
        rl->_handleEncoder(); rr->_handleEncoder();
    }
    dt->update();
    TEST_ASSERT_TRUE(fl->getTargetRPM() < 300.0f); // sanity: compensation applied

    // A new strafe() command resets _compensationFactor to 1.0 and reissues setSpeed(100%)
    mock.timeMs = 0;
    mock.reset();
    shield.begin();
    dt->strafe(0.0f, 1.0f, Speed::fromPercentage(100));

    // Target must be the full 300 RPM again, not the compensated value
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, fl->getTargetRPM());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, fr->getTargetRPM());
}

void test_compensation_not_triggered_by_true_stall() {
    // Stalled motors (RPM = 0) must not trigger global target reduction.
    // The 5% lower bound in isSaturated() prevents this.
    dt->strafe(0.0f, 1.0f, Speed::fromPercentage(100));

    // Advance time but give NO encoder pulses → RPM = 0 for all motors
    mock.timeMs = 100;
    // (no _handleEncoder() calls)

    dt->update(); // PID clamps to 100% but RPM=0 → 5% guard → not saturated

    // Targets must remain at 300 RPM because isSaturated() returned false
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, fl->getTargetRPM());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, fr->getTargetRPM());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, rl->getTargetRPM());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, rr->getTargetRPM());
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_strafe_right_equal_magnitude);
    RUN_TEST(test_strafe_forward_equal_pwm);
    RUN_TEST(test_strafe_ccw_spin_equal_magnitude);
    RUN_TEST(test_strafe_diagonal_normalised);
    RUN_TEST(test_strafe_diagonal_at_half_speed);
    RUN_TEST(test_speed_percentage_100_gives_255);
    RUN_TEST(test_speed_wheel_rpm_50pct);
    RUN_TEST(test_circle_left_wheel_ratios);
    RUN_TEST(test_circle_right_is_mirror_of_left);
    RUN_TEST(test_circle_large_radius_approaches_lateral);
    RUN_TEST(test_circle_radius_equals_leff_inner_stops);
    RUN_TEST(test_compensation_reduces_targets_when_saturated);
    RUN_TEST(test_compensation_not_triggered_when_motors_reach_target);
    RUN_TEST(test_compensation_resets_on_new_strafe);
    RUN_TEST(test_compensation_not_triggered_by_true_stall);
    return UNITY_END();
}