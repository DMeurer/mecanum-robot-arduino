#pragma once

#include <cstdint>
#include <cstring>

// ── Types ─────────────────────────────────────────────────────────────────────
using byte = uint8_t;

// ── Pin modes / levels ────────────────────────────────────────────────────────
static constexpr int INPUT = 0;
static constexpr int OUTPUT = 1;
static constexpr int INPUT_PULLUP = 2;
static constexpr int HIGH = 1;
static constexpr int LOW = 0;
static constexpr int RISING = 3;

// ── Mock state (accessible from tests) ───────────────────────────────────────
struct MockState {
    uint8_t pinModes[256] = {};
    uint8_t digitalValues[256] = {};
    uint8_t pwmValues[256] = {};
    int interruptCount = 0;
    unsigned long timeMs = 0;

    void reset() {
        std::memset(pinModes, 0, sizeof(pinModes));
        std::memset(digitalValues, 0, sizeof(digitalValues));
        std::memset(pwmValues, 0, sizeof(pwmValues));
        interruptCount = 0;
        timeMs = 0;
    }
};

// C++17 inline variable — single definition across all translation units
inline MockState mock;

// ── Arduino API stubs (inline — no separate .cpp needed) ─────────────────────
inline void pinMode(uint8_t pin, uint8_t mode) { mock.pinModes[pin] = mode; }
inline void digitalWrite(uint8_t pin, uint8_t val) { mock.digitalValues[pin] = val; }
inline int digitalRead(uint8_t pin) { return mock.digitalValues[pin]; }
inline void analogWrite(uint8_t pin, uint8_t val) { mock.pwmValues[pin] = val; }
inline unsigned long millis() { return mock.timeMs; }
inline void delay(unsigned long ms) { mock.timeMs += ms; }

inline void noInterrupts() {
}

inline void interrupts() {
}

inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int, void (*)(), int) { mock.interruptCount++; }