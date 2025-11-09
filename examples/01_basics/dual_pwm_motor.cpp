// ============================================================================
// EXAMPLE: Dual PWM Motor Driver Control
// ============================================================================
// This example demonstrates how to use the .withDualPWM() fluent API method
// to configure a motor driver that uses two PWM pins instead of PWM+DIR.
//
// HARDWARE:
//   - Motor driver with dual PWM inputs (e.g., L298N, custom H-bridge)
//   - DC motor
//   - Arduino or compatible microcontroller
//
// WIRING:
//   - Motor driver IN1/INA -> Arduino PWM pin (e.g., 5)
//   - Motor driver IN2/INB -> Arduino PWM pin (e.g., 6)
//   - Motor driver outputs -> DC motor
//   - Optional: STBY/SLEEP pin -> Arduino digital pin (e.g., 7)
//
// DUAL PWM MODE vs PWM+DIR MODE:
//
//   PWM+DIR Mode (default):
//     - One PWM pin controls speed (0-255)
//     - One digital pin controls direction (HIGH/LOW)
//     - Example: TB6612FNG, DRV8833 (typical usage)
//
//   Dual PWM Mode:
//     - First PWM pin controls forward speed (0-255)
//     - Second PWM pin controls reverse speed (0-255)
//     - Example: L298N, some custom H-bridges
//     - Also called "locked anti-phase" or "sign-magnitude" mode
//
// WHEN TO USE DUAL PWM:
//   - Your motor driver has two PWM inputs (IN1/IN2 or INA/INB)
//   - Datasheet specifies dual PWM control
//   - You want independent control of forward/reverse PWM
//
// ============================================================================

#include "../custom_headers/custom_motor_driver.h"
#include <iostream>

using namespace units;
using namespace units::literals;

// ============================================================================
// SIMULATION: Pretend Arduino functions for desktop compilation
// ============================================================================
#ifndef ARDUINO

void delay(int ms) {
    std::cout << "    [Simulated delay: " << ms << " ms]" << std::endl;
}

#endif

// ============================================================================
// MAIN EXAMPLE
// ============================================================================
int main() {
    std::cout << "==========================================================\n";
    std::cout << "Dual PWM Motor Driver - Example\n";
    std::cout << "==========================================================\n\n";

    // ========================================================================
    // Example 1: Default PWM+DIR Mode (for comparison)
    // ========================================================================
    std::cout << "Example 1: Standard PWM+DIR Mode\n";
    std::cout << "----------------------------------------\n";

    // Create motor in default PWM+DIR mode
    SimplePWMMotor standard_motor(9, 8, rpm(200), 20.0);

    std::cout << "Standard motor configured:\n";
    std::cout << "  Mode: " << (standard_motor.isPWMDir() ? "PWM+DIR" : "Dual PWM") << "\n";
    std::cout << "  Pin A (PWM): " << standard_motor.getPinA() << "\n";
    std::cout << "  Pin B (DIR): " << standard_motor.getPinB() << "\n\n";

    standard_motor.setDutyCycle(0.5);
    std::cout << "PWM+DIR: 50% forward -> PWM=127 on pin 9, DIR=HIGH on pin 8\n\n";

    // ========================================================================
    // Example 2: Dual PWM Mode Configuration
    // ========================================================================
    std::cout << "Example 2: Dual PWM Mode\n";
    std::cout << "----------------------------------------\n";

    // Create motor with two PWM-capable pins
    SimplePWMMotor dual_motor(5, 6, rpm(200), 20.0);

    // Switch to dual PWM mode
    dual_motor.withDualPWM();

    std::cout << "Dual PWM motor configured:\n";
    std::cout << "  Mode: " << (dual_motor.isDualPWM() ? "Dual PWM" : "PWM+DIR") << "\n";
    std::cout << "  Pin A (Forward PWM): " << dual_motor.getPinA() << "\n";
    std::cout << "  Pin B (Reverse PWM): " << dual_motor.getPinB() << "\n\n";

    // ========================================================================
    // Example 3: Forward Motion
    // ========================================================================
    std::cout << "Example 3: Forward Motion\n";
    std::cout << "----------------------------------------\n";

    dual_motor.setDutyCycle(0.5);
    std::cout << "50% forward:\n";
    std::cout << "  Pin 5 (forward): PWM = 127 (50%)\n";
    std::cout << "  Pin 6 (reverse): PWM = 0 (off)\n";
    delay(1000);

    dual_motor.setDutyCycle(1.0);
    std::cout << "\n100% forward:\n";
    std::cout << "  Pin 5 (forward): PWM = 255 (100%)\n";
    std::cout << "  Pin 6 (reverse): PWM = 0 (off)\n";
    delay(1000);
    std::cout << "\n";

    // ========================================================================
    // Example 4: Reverse Motion
    // ========================================================================
    std::cout << "Example 4: Reverse Motion\n";
    std::cout << "----------------------------------------\n";

    dual_motor.setDutyCycle(-0.5);
    std::cout << "50% reverse:\n";
    std::cout << "  Pin 5 (forward): PWM = 0 (off)\n";
    std::cout << "  Pin 6 (reverse): PWM = 127 (50%)\n";
    delay(1000);

    dual_motor.setDutyCycle(-1.0);
    std::cout << "\n100% reverse:\n";
    std::cout << "  Pin 5 (forward): PWM = 0 (off)\n";
    std::cout << "  Pin 6 (reverse): PWM = 255 (100%)\n";
    delay(1000);
    std::cout << "\n";

    // ========================================================================
    // Example 5: Stopping
    // ========================================================================
    std::cout << "Example 5: Stopping\n";
    std::cout << "----------------------------------------\n";

    dual_motor.stop();
    std::cout << "Motor stopped:\n";
    std::cout << "  Pin 5 (forward): PWM = 0 (off)\n";
    std::cout << "  Pin 6 (reverse): PWM = 0 (off)\n";
    std::cout << "  Duty cycle: " << dual_motor.getDutyCycle() << "\n\n";

    // ========================================================================
    // Example 6: Fluent API - Dual PWM + Sleep Pin
    // ========================================================================
    std::cout << "Example 6: Dual PWM with Sleep Pin\n";
    std::cout << "----------------------------------------\n";

    // Configure everything in one fluent chain!
    SimplePWMMotor advanced_motor(3, 11, rpm(150), 15.0);
    advanced_motor.withDualPWM()       // Enable dual PWM mode
                  .withSleepPin(7)     // Add sleep pin
                  .setInverted(true);  // Invert direction

    std::cout << "Advanced motor configured:\n";
    std::cout << "  Mode: Dual PWM\n";
    std::cout << "  Forward pin: " << advanced_motor.getPinA() << "\n";
    std::cout << "  Reverse pin: " << advanced_motor.getPinB() << "\n";
    std::cout << "  Sleep pin: " << advanced_motor.getSleepPin() << "\n";
    std::cout << "  Inverted: Yes\n\n";

    // Power management with dual PWM
    advanced_motor.disableSleep();
    std::cout << "Motor awake\n";

    advanced_motor.setDutyCycle(0.75);
    std::cout << "Running at 75% (inverted means reverse)\n";
    delay(1500);

    advanced_motor.stop();
    advanced_motor.enableSleep();
    std::cout << "Motor stopped and sleeping (power saving)\n\n";

    // ========================================================================
    // Example 7: Differential Drive with Dual PWM
    // ========================================================================
    std::cout << "Example 7: Differential Drive Robot\n";
    std::cout << "----------------------------------------\n";
    std::cout << "Using dual PWM for left and right motors\n\n";

    // Left motor (pins 5, 6)
    SimplePWMMotor left_motor(5, 6, rpm(180), 12.0);
    left_motor.withDualPWM().withSleepPin(4);

    // Right motor (pins 9, 10)
    SimplePWMMotor right_motor(9, 10, rpm(180), 12.0);
    right_motor.withDualPWM().withSleepPin(8).setInverted(true);  // Invert right

    std::cout << "Left motor:\n";
    std::cout << "  Pins: " << left_motor.getPinA() << ", " << left_motor.getPinB() << "\n";
    std::cout << "  Sleep: " << left_motor.getSleepPin() << "\n";

    std::cout << "Right motor:\n";
    std::cout << "  Pins: " << right_motor.getPinA() << ", " << right_motor.getPinB() << "\n";
    std::cout << "  Sleep: " << right_motor.getSleepPin() << "\n";
    std::cout << "  Inverted: Yes\n\n";

    // Wake both motors
    left_motor.disableSleep();
    right_motor.disableSleep();
    std::cout << "Both motors awake\n\n";

    // Forward
    std::cout << "Moving forward:\n";
    left_motor.setDutyCycle(0.6);
    right_motor.setDutyCycle(0.6);
    std::cout << "  Left: 60% forward\n";
    std::cout << "  Right: 60% forward (inverted)\n";
    delay(2000);

    // Turn right
    std::cout << "\nTurning right:\n";
    left_motor.setDutyCycle(0.8);
    right_motor.setDutyCycle(0.2);
    std::cout << "  Left: 80% forward (faster)\n";
    std::cout << "  Right: 20% forward (slower)\n";
    delay(1500);

    // Spin in place (left forward, right reverse)
    std::cout << "\nSpinning in place:\n";
    left_motor.setDutyCycle(0.5);
    right_motor.setDutyCycle(-0.5);
    std::cout << "  Left: 50% forward\n";
    std::cout << "  Right: 50% reverse\n";
    delay(1000);

    // Stop both
    left_motor.stop();
    right_motor.stop();
    std::cout << "\nBoth motors stopped\n";

    // Sleep both for power saving
    left_motor.enableSleep();
    right_motor.enableSleep();
    std::cout << "Both motors sleeping (power saving)\n\n";

    // ========================================================================
    // Example 8: Mode Comparison
    // ========================================================================
    std::cout << "Example 8: Mode Comparison\n";
    std::cout << "----------------------------------------\n";

    SimplePWMMotor motor_a(2, 3, rpm(200));
    SimplePWMMotor motor_b(2, 3, rpm(200));
    motor_b.withDualPWM();

    std::cout << "Same pins, different modes:\n\n";

    std::cout << "Motor A (PWM+DIR):\n";
    std::cout << "  50% forward:\n";
    std::cout << "    Pin 2: PWM = 127\n";
    std::cout << "    Pin 3: DIR = HIGH\n\n";

    std::cout << "Motor B (Dual PWM):\n";
    std::cout << "  50% forward:\n";
    std::cout << "    Pin 2: PWM = 127 (forward)\n";
    std::cout << "    Pin 3: PWM = 0 (reverse off)\n\n";

    std::cout << "Motor A (PWM+DIR):\n";
    std::cout << "  50% reverse:\n";
    std::cout << "    Pin 2: PWM = 127\n";
    std::cout << "    Pin 3: DIR = LOW\n\n";

    std::cout << "Motor B (Dual PWM):\n";
    std::cout << "  50% reverse:\n";
    std::cout << "    Pin 2: PWM = 0 (forward off)\n";
    std::cout << "    Pin 3: PWM = 127 (reverse)\n\n";

    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "==========================================================\n";
    std::cout << "Summary\n";
    std::cout << "==========================================================\n";
    std::cout << "Dual PWM features demonstrated:\n";
    std::cout << "  ✓ .withDualPWM() fluent API method\n";
    std::cout << "  ✓ Forward motion (PWM on pin A, pin B off)\n";
    std::cout << "  ✓ Reverse motion (PWM on pin B, pin A off)\n";
    std::cout << "  ✓ isDualPWM() for checking mode\n";
    std::cout << "  ✓ Combining with sleep pin support\n";
    std::cout << "  ✓ Differential drive robot configuration\n";
    std::cout << "  ✓ Method chaining with other configurations\n\n";

    std::cout << "When to use Dual PWM:\n";
    std::cout << "  • Motor driver requires two PWM inputs\n";
    std::cout << "  • L298N in dual PWM configuration\n";
    std::cout << "  • Custom H-bridges with dual PWM\n";
    std::cout << "  • Independent forward/reverse control needed\n\n";

    std::cout << "When to use PWM+DIR (default):\n";
    std::cout << "  • Motor driver has separate speed and direction pins\n";
    std::cout << "  • TB6612FNG, DRV8833, DRV8835 (typical usage)\n";
    std::cout << "  • One PWM pin + one digital pin\n";
    std::cout << "  • Most common configuration\n\n";

    std::cout << "Key differences:\n";
    std::cout << "  PWM+DIR: Speed on PWM, direction on digital (HIGH/LOW)\n";
    std::cout << "  Dual PWM: Forward speed on pin A, reverse on pin B\n\n";

    return 0;
}

// ============================================================================
// WIRING GUIDE
// ============================================================================
/*
L298N Motor Driver (Dual PWM Mode):
====================================
Arduino Pin 5  -> IN1 (Forward PWM)
Arduino Pin 6  -> IN2 (Reverse PWM)
Arduino GND    -> GND
Motor          -> OUT1, OUT2

For PWM+DIR mode, use:
Arduino Pin 9  -> ENA (PWM speed)
Arduino Pin 8  -> IN1 (Direction)
Arduino GND    -> GND
IN2            -> Tied to opposite of IN1 (or use enable pin)

TB6612FNG Motor Driver (PWM+DIR Mode - Default):
================================================
Arduino Pin 9  -> PWMA (Speed)
Arduino Pin 8  -> AIN1 (Direction bit 1)
Arduino Pin 7  -> AIN2 (Direction bit 2)
Arduino Pin 6  -> STBY (Sleep pin)
Arduino GND    -> GND
Motor          -> A01, A02

Note: TB6612FNG typically uses AIN1/AIN2 for direction (not dual PWM),
      but could be configured for dual PWM by using PWM on both AIN pins.
*/
