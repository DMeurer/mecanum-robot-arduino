#include "L293DShield.h"

void L293DShield::begin() {
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);

    for (uint8_t i = 0; i < 4; i++) {
        pinMode(_pwmPins[i], OUTPUT);
        analogWrite(_pwmPins[i], 0);
    }

    _shiftState = 0;
    _writeShiftReg();

    // Enable the shift register outputs (active LOW)
    digitalWrite(MOTORENABLE, LOW);
}

void L293DShield::setMotorDirection(uint8_t motor, int8_t dir) {
    if (motor < 1 || motor > 4) return;
    uint8_t idx = motor - 1;

    // Clear both bits first (coast)
    _shiftState &= ~(1 << _bitA[idx]);
    _shiftState &= ~(1 << _bitB[idx]);

    if (dir > 0) {
        // Forward: A=HIGH, B=LOW
        _shiftState |= (1 << _bitA[idx]);
    } else if (dir < 0) {
        // Backward: A=LOW, B=HIGH
        _shiftState |= (1 << _bitB[idx]);
    }
    // dir == 0: both LOW → coast (already cleared above)

    _writeShiftReg();
}

void L293DShield::setMotorSpeed(uint8_t motor, uint8_t pwm) {
    if (motor < 1 || motor > 4) return;
    analogWrite(_pwmPins[motor - 1], pwm);
}

void L293DShield::_writeShiftReg() {
    digitalWrite(MOTORLATCH, LOW);

    // Shift out 8 bits, MSB first
    for (int8_t bit = 7; bit >= 0; bit--) {
        digitalWrite(MOTORCLK, LOW);
        digitalWrite(MOTORDATA, (_shiftState >> bit) & 1 ? HIGH : LOW);
        digitalWrite(MOTORCLK, HIGH);
    }

    digitalWrite(MOTORLATCH, HIGH);
}