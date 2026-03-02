#include <unity.h>
#include "Arduino.h"
#include "L293DShield.h"
#include "DCMotorWithEncoder.h"

// ── Constants ─────────────────────────────────────────────────────────────────
static constexpr float COUNTS_PER_REV = 56.0f; // 14 poles × 4 edges
static constexpr float GEAR_RATIO = 30.0f;
static constexpr float MAX_MOTOR_RPM = 300.0f;

// M1 pins
static constexpr uint8_t PWM_PIN = 11; // M1 PWM
static constexpr int ENC_A = 2;
static constexpr int ENC_B = 22;

// ── Helpers ───────────────────────────────────────────────────────────────────
static L293DShield shield;
static DCMotorWithEncoder *motor = nullptr;

static void makeMotor() {
    // Reset static instance table
    for (int i = 0; i < 4; i++) DCMotorWithEncoder::_instances[i] = nullptr;
    delete motor;
    motor = new DCMotorWithEncoder(shield, 1, ENC_A, ENC_B,
                                   COUNTS_PER_REV, GEAR_RATIO, MAX_MOTOR_RPM);
}

void setUp() {
    mock.reset();
    shield.begin();
    makeMotor();
}

void tearDown() {
}

// ── setSpeed(%) ───────────────────────────────────────────────────────────────

void test_setSpeed_zero_gives_zero_pwm() {
    motor->setSpeed(0.0f);
    TEST_ASSERT_EQUAL(0, mock.pwmValues[PWM_PIN]);
}

void test_setSpeed_100_gives_255_pwm() {
    motor->setSpeed(100.0f);
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PWM_PIN]);
}

void test_setSpeed_50_gives_128_pwm() {
    motor->setSpeed(50.0f);
    // 50/100 * 255 + 0.5 = 128
    TEST_ASSERT_EQUAL(128, mock.pwmValues[PWM_PIN]);
}

void test_setSpeed_negative_sets_reverse_direction() {
    motor->setSpeed(-50.0f);
    TEST_ASSERT_EQUAL(128, mock.pwmValues[PWM_PIN]);
    // Direction is encoded in shift register; we verify the PWM magnitude
}

// ── setSpeedRPM ───────────────────────────────────────────────────────────────

void test_setSpeedRPM_max_gives_255() {
    motor->setSpeedRPM(MAX_MOTOR_RPM);
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PWM_PIN]);
}

void test_setSpeedRPM_half_gives_127_or_128() {
    motor->setSpeedRPM(MAX_MOTOR_RPM / 2.0f);
    // 50 % → 128
    TEST_ASSERT_EQUAL(128, mock.pwmValues[PWM_PIN]);
}

// ── setSpeedWheelRPM ──────────────────────────────────────────────────────────

void test_setSpeedWheelRPM_applies_gear_ratio() {
    float maxWheelRPM = MAX_MOTOR_RPM / GEAR_RATIO;
    motor->setSpeedWheelRPM(maxWheelRPM);
    TEST_ASSERT_EQUAL(255, mock.pwmValues[PWM_PIN]);
}

// ── stop ──────────────────────────────────────────────────────────────────────

void test_stop_zeroes_pwm() {
    motor->setSpeed(80.0f);
    motor->stop();
    TEST_ASSERT_EQUAL(0, mock.pwmValues[PWM_PIN]);
}

void test_stop_clears_running_target() {
    motor->rotateMotorRevolutions(5.0f, 50.0f);
    TEST_ASSERT_TRUE(motor->isRunning());
    motor->stop();
    TEST_ASSERT_FALSE(motor->isRunning());
}

// ── Rotation target: encoder count ───────────────────────────────────────────

void test_rotateMotorRevolutions_correct_target() {
    // 2 motor revolutions → 2 * COUNTS_PER_REV counts from current position
    motor->resetEncoder();
    motor->rotateMotorRevolutions(2.0f, 50.0f);
    TEST_ASSERT_TRUE(motor->isRunning());

    // Simulate forward encoder pulses (B=HIGH → increment)
    mock.digitalValues[ENC_B] = HIGH;
    for (long i = 0; i < (long) (2 * COUNTS_PER_REV); i++) {
        motor->_handleEncoder();
    }
    // Add one more to go past the target
    motor->_handleEncoder();

    mock.timeMs += 100;
    motor->update();
    TEST_ASSERT_FALSE(motor->isRunning());
}

void test_rotateWheelRevolutions_applies_gear_ratio() {
    motor->resetEncoder();
    motor->rotateWheelRevolutions(1.0f, 50.0f);
    TEST_ASSERT_TRUE(motor->isRunning());

    // 1 wheel rev = GEAR_RATIO motor revs = GEAR_RATIO * COUNTS_PER_REV counts
    long targetCounts = (long) (GEAR_RATIO * COUNTS_PER_REV);
    mock.digitalValues[ENC_B] = HIGH; // B=HIGH → increment (forward)
    for (long i = 0; i <= targetCounts; i++) {
        motor->_handleEncoder();
    }

    mock.timeMs += 100;
    motor->update();
    TEST_ASSERT_FALSE(motor->isRunning());
}

// ── Encoder direction ─────────────────────────────────────────────────────────

void test_encoder_increments_when_B_high() {
    motor->resetEncoder();
    mock.digitalValues[ENC_B] = HIGH;
    motor->_handleEncoder();
    motor->_handleEncoder();
    TEST_ASSERT_EQUAL(2, motor->getEncoderCount());
}

void test_encoder_decrements_when_B_low() {
    motor->resetEncoder();
    mock.digitalValues[ENC_B] = LOW;
    motor->_handleEncoder();
    motor->_handleEncoder();
    TEST_ASSERT_EQUAL(-2, motor->getEncoderCount());
}

// ── isRunning / update ────────────────────────────────────────────────────────

void test_isRunning_false_initially() {
    TEST_ASSERT_FALSE(motor->isRunning());
}

void test_isRunning_true_after_rotate_call() {
    motor->rotateMotorRevolutions(1.0f, 50.0f);
    TEST_ASSERT_TRUE(motor->isRunning());
}

// ── Closed-loop PID ───────────────────────────────────────────────────────────

void test_setSpeed_stores_target_rpm() {
    // setSpeed(50%) → target = 50% of maxRPM = 150 RPM
    motor->setSpeed(50.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, motor->getTargetRPM());
}

void test_setSpeed_negative_stores_negative_target_rpm() {
    motor->setSpeed(-75.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -225.0f, motor->getTargetRPM());
}

void test_stop_clears_closed_loop() {
    motor->setSpeed(80.0f);
    motor->stop();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, motor->getTargetRPM());
    TEST_ASSERT_EQUAL(0, mock.pwmValues[PWM_PIN]);
}

void test_isSaturated_false_initially() {
    motor->setSpeed(50.0f);
    // No time has passed, RPM = 0 but currentPWMPct = 50% (< 99%) → not saturated
    TEST_ASSERT_FALSE(motor->isSaturated());
}

void test_isSaturated_true_when_at_max_pwm_and_low_rpm() {
    // setSpeed(100%) → target = 300 RPM, currentPWMPct = 100%
    motor->setSpeed(100.0f);
    // After 100ms with encoder giving ~200 RPM: PID clamps output to 100%
    mock.timeMs = 100;
    // 19 pulses / 56 cpr / (100ms/60000) ≈ 203 RPM
    mock.digitalValues[ENC_B] = HIGH;
    for (int i = 0; i < 19; i++) motor->_handleEncoder();
    motor->update(); // RPM ≈ 203, PID output clamped to 100%
    // 203 < 300*0.9=270 and PWM ≥ 99% and 203 > 300*0.05=15 → saturated
    TEST_ASSERT_TRUE(motor->isSaturated());
}

void test_isSaturated_false_when_rpm_at_target() {
    motor->setSpeed(50.0f); // target = 150 RPM, PWM = 50%
    // 14 pulses in 100ms ≈ 150 RPM
    mock.timeMs = 100;
    mock.digitalValues[ENC_B] = HIGH;
    for (int i = 0; i < 14; i++) motor->_handleEncoder();
    motor->update(); // RPM ≈ 150, PID reduces PWM toward 50% (not saturated)
    TEST_ASSERT_FALSE(motor->isSaturated());
}

void test_pid_increases_pwm_when_below_target_rpm() {
    motor->setSpeed(50.0f); // target 150 RPM, initial PWM = 50%
    TEST_ASSERT_EQUAL(128, mock.pwmValues[PWM_PIN]); // feedforward applied

    // Advance time and give only ~half the expected RPM (7 pulses ≈ 75 RPM)
    mock.timeMs = 100;
    mock.digitalValues[ENC_B] = HIGH;
    for (int i = 0; i < 7; i++) motor->_handleEncoder();
    motor->update(); // PID sees error ≈ 75 RPM → increases PWM above 128

    TEST_ASSERT_TRUE(mock.pwmValues[PWM_PIN] > 128);
}

void test_adjustTargetRPM_scales_target() {
    motor->setSpeed(100.0f); // target = 300 RPM
    motor->adjustTargetRPM(0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 150.0f, motor->getTargetRPM());
}

void test_adjustTargetRPM_no_effect_when_closed_loop_off() {
    motor->rotateMotorRevolutions(2.0f, 50.0f); // closed-loop off during rotation
    float before = motor->getTargetRPM();        // = 0 (closed-loop inactive)
    motor->adjustTargetRPM(0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, before, motor->getTargetRPM());
}

void test_setSpeed_zero_disables_closed_loop() {
    motor->setSpeed(50.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, motor->getTargetRPM()); // sanity
    motor->setSpeed(0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, motor->getTargetRPM());
    TEST_ASSERT_EQUAL(0, mock.pwmValues[PWM_PIN]);
}

void test_rotate_disables_closed_loop() {
    // setSpeed activates closed-loop; rotate() should immediately turn it off
    motor->setSpeed(50.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, motor->getTargetRPM()); // sanity
    motor->rotateMotorRevolutions(2.0f, 50.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, motor->getTargetRPM()); // closed-loop off
    TEST_ASSERT_TRUE(motor->isRunning());                           // rotation target active
}

void test_setSpeedRPM_activates_closed_loop() {
    // setSpeedRPM(150) → setSpeed(50%) → targetRPM = 150, feedforward PWM applied
    motor->setSpeedRPM(150.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 150.0f, motor->getTargetRPM());
    TEST_ASSERT_EQUAL(128, mock.pwmValues[PWM_PIN]); // 50% feedforward
}

void test_pid_reduces_pwm_when_above_target_rpm() {
    motor->setSpeed(50.0f); // target 150 RPM, initial PWM = 50% (128)
    TEST_ASSERT_EQUAL(128, mock.pwmValues[PWM_PIN]);

    // 16 pulses in 100 ms ≈ 171 RPM — above the 150 RPM target
    mock.timeMs = 100;
    mock.digitalValues[ENC_B] = HIGH;
    for (int i = 0; i < 16; i++) motor->_handleEncoder();
    motor->update(); // PID sees negative error → lowers PWM below feedforward

    TEST_ASSERT_TRUE(mock.pwmValues[PWM_PIN] < 128);
}

void test_pid_gains_configurable() {
    // With all gains zeroed, PID adds nothing — output is pure feedforward.
    // Any RPM error should leave the PWM unchanged from the ff value.
    motor->setPIDGains(0.0f, 0.0f, 0.0f);
    motor->setSpeed(50.0f); // target 150 RPM, ff = 50% → 128 PWM

    mock.timeMs = 100;
    mock.digitalValues[ENC_B] = HIGH;
    for (int i = 0; i < 7; i++) motor->_handleEncoder(); // ≈75 RPM — big error
    motor->update(); // output = ff + 0 + 0 + 0 = 50% → still 128

    TEST_ASSERT_EQUAL(128, mock.pwmValues[PWM_PIN]);
}

// ── RPM calculation ───────────────────────────────────────────────────────────

void test_getMotorRPM_from_encoder_delta() {
    motor->resetEncoder();
    mock.digitalValues[ENC_B] = HIGH;

    // Simulate 1 full motor revolution in 100 ms
    mock.timeMs = 0;
    motor->update(); // prime _lastRPMTime = 0, _lastCount = 0

    // Advance 100 ms and add COUNTS_PER_REV counts
    mock.timeMs = 200; // ensure >= 50 ms threshold
    for (int i = 0; i < (int) COUNTS_PER_REV; i++) motor->_handleEncoder();
    motor->update();

    // 56 counts / 56 counts_per_rev = 1 rev in 200 ms = 5 rev/s = 300 RPM
    float rpm = motor->getMotorRPM();
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 300.0f, rpm);
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_setSpeed_zero_gives_zero_pwm);
    RUN_TEST(test_setSpeed_100_gives_255_pwm);
    RUN_TEST(test_setSpeed_50_gives_128_pwm);
    RUN_TEST(test_setSpeed_negative_sets_reverse_direction);
    RUN_TEST(test_setSpeedRPM_max_gives_255);
    RUN_TEST(test_setSpeedRPM_half_gives_127_or_128);
    RUN_TEST(test_setSpeedWheelRPM_applies_gear_ratio);
    RUN_TEST(test_stop_zeroes_pwm);
    RUN_TEST(test_stop_clears_running_target);
    RUN_TEST(test_rotateMotorRevolutions_correct_target);
    RUN_TEST(test_rotateWheelRevolutions_applies_gear_ratio);
    RUN_TEST(test_encoder_increments_when_B_high);
    RUN_TEST(test_encoder_decrements_when_B_low);
    RUN_TEST(test_isRunning_false_initially);
    RUN_TEST(test_isRunning_true_after_rotate_call);
    RUN_TEST(test_getMotorRPM_from_encoder_delta);
    RUN_TEST(test_setSpeed_stores_target_rpm);
    RUN_TEST(test_setSpeed_negative_stores_negative_target_rpm);
    RUN_TEST(test_stop_clears_closed_loop);
    RUN_TEST(test_isSaturated_false_initially);
    RUN_TEST(test_isSaturated_true_when_at_max_pwm_and_low_rpm);
    RUN_TEST(test_isSaturated_false_when_rpm_at_target);
    RUN_TEST(test_pid_increases_pwm_when_below_target_rpm);
    RUN_TEST(test_adjustTargetRPM_scales_target);
    RUN_TEST(test_adjustTargetRPM_no_effect_when_closed_loop_off);
    RUN_TEST(test_setSpeed_zero_disables_closed_loop);
    RUN_TEST(test_rotate_disables_closed_loop);
    RUN_TEST(test_setSpeedRPM_activates_closed_loop);
    RUN_TEST(test_pid_reduces_pwm_when_above_target_rpm);
    RUN_TEST(test_pid_gains_configurable);
    return UNITY_END();
}