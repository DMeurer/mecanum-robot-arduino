#include <Arduino.h>
#include "L293DShield.h"
#include "DCMotorWithEncoder.h"
#include "MecanumDrivetrain.h"

// ── Hardware configuration ────────────────────────────────────────────────────
//
//  Motor Shield (L293D / 74HC595):
//    MOTORLATCH=12, MOTORCLK=4, MOTORENABLE=7, MOTORDATA=8
//    M1 PWM=11 (FL)  M2 PWM=3 (FR)  M3 PWM=5 (RL)  M4 PWM=6 (RR)
//
//  Encoders (14-pole magnetic, quadrature A+B):
//    Motor   Enc A (ext. interrupt)   Enc B (digital)
//    FL(M1)  Pin 2                    Pin 22
//    FR(M2)  Pin 18                   Pin 23
//    RL(M3)  Pin 19                   Pin 24
//    RR(M4)  Pin 20                   Pin 25
//
// ─────────────────────────────────────────────────────────────────────────────

static constexpr float COUNTS_PER_MOTOR_REV = 14.0f * 4.0f; // 14 poles × 4 edges
static constexpr float GEAR_RATIO = 30.0f; // adjust to your gearbox
static constexpr float MAX_MOTOR_RPM = 300.0f; // no-load motor speed

static constexpr float TRACK_WIDTH_MM = 200.0f;
static constexpr float WHEEL_BASE_MM = 180.0f;

L293DShield shield;

DCMotorWithEncoder motorFL(shield, 1, 2, 22, COUNTS_PER_MOTOR_REV, GEAR_RATIO, MAX_MOTOR_RPM);
DCMotorWithEncoder motorFR(shield, 2, 18, 23, COUNTS_PER_MOTOR_REV, GEAR_RATIO, MAX_MOTOR_RPM);
DCMotorWithEncoder motorRL(shield, 3, 19, 24, COUNTS_PER_MOTOR_REV, GEAR_RATIO, MAX_MOTOR_RPM);
DCMotorWithEncoder motorRR(shield, 4, 20, 25, COUNTS_PER_MOTOR_REV, GEAR_RATIO, MAX_MOTOR_RPM);

MecanumDrivetrain drivetrain(motorFL, motorFR, motorRL, motorRR,
                             TRACK_WIDTH_MM, WHEEL_BASE_MM);

void setup() {
    Serial.begin(115200);
    shield.begin();

    // Example: strafe right at 60 % speed
    drivetrain.strafe(1.0f, 0.0f, Speed::fromPercentage(60));
    delay(1000);

    // Example: drive forward at 50 % speed
    drivetrain.strafe(0.0f, 1.0f);
    delay(1000);

    // Example: spin CCW at 40 %
    drivetrain.strafe(0.0f, 0.0f, 1.0f, Speed::fromPercentage(40));
    delay(1000);

    // Example: circle CCW with 300 mm radius
    drivetrain.circle(300.0f, Direction::Left, Speed::fromPercentage(50));
    delay(2000);

    drivetrain.stop();
}

void loop() {
    drivetrain.update();
}
