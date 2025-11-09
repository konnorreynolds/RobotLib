// ============================================================================
// EXAMPLE: Motor Driver with Sleep Pin Control
// ============================================================================
// This example demonstrates how to use the .withSleepPin() fluent API method
// to configure a motor driver with power-saving sleep functionality.
//
// HARDWARE:
//   - H-bridge motor driver with sleep/standby pin (TB6612FNG, DRV8833, etc.)
//   - DC motor
//   - Arduino or compatible microcontroller
//
// WIRING:
//   - Motor driver PWM pin -> Arduino PWM pin (e.g., 9)
//   - Motor driver DIR pin -> Arduino digital pin (e.g., 8)
//   - Motor driver STBY/SLEEP pin -> Arduino digital pin (e.g., 7)
//   - Motor driver outputs -> DC motor
//
// WHAT THIS DEMONSTRATES:
//   - Using .withSleepPin() to configure sleep functionality
//   - Fluent API method chaining
//   - Controlling motor driver power states
//   - Power-saving techniques for battery-powered robots
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
    std::cout << "Motor Driver with Sleep Pin - Example\n";
    std::cout << "==========================================================\n\n";

    // ========================================================================
    // Example 1: Basic Sleep Pin Configuration
    // ========================================================================
    std::cout << "Example 1: Basic Sleep Pin Usage\n";
    std::cout << "----------------------------------------\n";

    // Create motor driver (PWM pin 9, DIR pin 8, max 200 RPM, 20:1 gearing)
    SimplePWMMotor motor(9, 8, rpm(200), 20.0);

    // Configure sleep pin on digital pin 7
    motor.withSleepPin(7);

    std::cout << "Motor configured:\n";
    std::cout << "  PWM Pin: 9\n";
    std::cout << "  DIR Pin: 8\n";
    std::cout << "  Sleep Pin: " << motor.getSleepPin() << "\n";
    std::cout << "  Has sleep pin: " << (motor.hasSleepPin() ? "Yes" : "No") << "\n";
    std::cout << "  Currently sleeping: " << (motor.isSleeping() ? "Yes" : "No") << "\n\n";

    // ========================================================================
    // Example 2: Normal Motor Operation
    // ========================================================================
    std::cout << "Example 2: Normal Motor Operation\n";
    std::cout << "----------------------------------------\n";

    motor.disableSleep();  // Wake up motor driver
    std::cout << "Motor driver awake (sleep disabled)\n";

    motor.setDutyCycle(0.5);
    std::cout << "Running motor at 50% duty cycle forward\n";
    delay(1000);

    motor.setVelocity(rpm(100));
    std::cout << "Running motor at 100 RPM\n";
    delay(2000);

    motor.stop();
    std::cout << "Motor stopped\n\n";

    // ========================================================================
    // Example 3: Power-Saving with Sleep Mode
    // ========================================================================
    std::cout << "Example 3: Power-Saving Sleep Mode\n";
    std::cout << "----------------------------------------\n";

    // When robot is idle, put motor driver to sleep to save power
    motor.enableSleep();
    std::cout << "Motor driver sleeping (low power mode)\n";
    std::cout << "  Currently sleeping: " << (motor.isSleeping() ? "Yes" : "No") << "\n";
    delay(3000);  // Save power for 3 seconds

    // Wake up when needed
    motor.disableSleep();
    std::cout << "Motor driver awake\n";
    std::cout << "  Currently sleeping: " << (motor.isSleeping() ? "Yes" : "No") << "\n";

    motor.setDutyCycle(0.75);
    std::cout << "Running motor at 75% duty cycle\n";
    delay(2000);

    motor.stop();
    std::cout << "Motor stopped\n\n";

    // ========================================================================
    // Example 4: Fluent API Method Chaining
    // ========================================================================
    std::cout << "Example 4: Fluent API Chaining\n";
    std::cout << "----------------------------------------\n";

    // Configure everything in one fluent chain!
    SimplePWMMotor left_motor(10, 11, rpm(150), 15.0);
    left_motor.withSleepPin(6)      // Configure sleep pin
              .setInverted(true);    // Invert direction

    SimplePWMMotor right_motor(12, 13, rpm(150), 15.0);
    right_motor.withSleepPin(5);    // Sleep pin on different pin

    std::cout << "Left motor configured:\n";
    std::cout << "  Sleep pin: " << left_motor.getSleepPin() << "\n";
    std::cout << "  Inverted: Yes\n";

    std::cout << "Right motor configured:\n";
    std::cout << "  Sleep pin: " << right_motor.getSleepPin() << "\n";
    std::cout << "  Inverted: No\n\n";

    // ========================================================================
    // Example 5: Toggle Sleep (useful for periodic sleep/wake cycles)
    // ========================================================================
    std::cout << "Example 5: Toggle Sleep\n";
    std::cout << "----------------------------------------\n";

    for (int i = 0; i < 3; i++) {
        motor.toggleSleep();
        std::cout << "Cycle " << (i+1) << ": Motor is "
                  << (motor.isSleeping() ? "sleeping" : "awake") << "\n";
        delay(1000);
    }
    std::cout << "\n";

    // ========================================================================
    // Example 6: Power-Saving Pattern for Battery-Powered Robots
    // ========================================================================
    std::cout << "Example 6: Battery-Saving Pattern\n";
    std::cout << "----------------------------------------\n";
    std::cout << "Simulating intermittent robot operation...\n\n";

    // Ensure motor is stopped and sleeping initially
    motor.stop();
    motor.enableSleep();

    for (int cycle = 1; cycle <= 3; cycle++) {
        std::cout << "Cycle " << cycle << ":\n";

        // Wake up
        std::cout << "  Waking up motor driver...\n";
        motor.disableSleep();
        delay(100);  // Small delay for driver to stabilize

        // Do work
        std::cout << "  Running motor (doing work)...\n";
        motor.setDutyCycle(0.6);
        delay(1500);

        // Stop and sleep
        std::cout << "  Stopping motor...\n";
        motor.stop();
        delay(50);

        std::cout << "  Putting motor driver to sleep...\n";
        motor.enableSleep();
        delay(500);  // Idle time (saving power)

        std::cout << "\n";
    }

    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "==========================================================\n";
    std::cout << "Summary\n";
    std::cout << "==========================================================\n";
    std::cout << "Sleep pin features demonstrated:\n";
    std::cout << "  ✓ .withSleepPin() fluent API method\n";
    std::cout << "  ✓ enableSleep() / disableSleep() for power control\n";
    std::cout << "  ✓ toggleSleep() for alternating states\n";
    std::cout << "  ✓ isSleeping() for querying state\n";
    std::cout << "  ✓ hasSleepPin() for checking if configured\n";
    std::cout << "  ✓ Method chaining with other configurations\n";
    std::cout << "  ✓ Battery-saving patterns\n\n";

    std::cout << "Benefits of sleep pin:\n";
    std::cout << "  • Reduces power consumption when motors idle\n";
    std::cout << "  • Extends battery life in mobile robots\n";
    std::cout << "  • Reduces heat generation in motor drivers\n";
    std::cout << "  • Provides clean on/off control of all motors\n\n";

    std::cout << "Compatible motor drivers:\n";
    std::cout << "  • TB6612FNG (STBY pin)\n";
    std::cout << "  • DRV8833 (nSLEEP pin)\n";
    std::cout << "  • DRV8835 (MODE/nSLEEP pin)\n";
    std::cout << "  • And many others with sleep/standby pins\n\n";

    return 0;
}

// ============================================================================
// EXPECTED OUTPUT (simulated)
// ============================================================================
/*
==========================================================
Motor Driver with Sleep Pin - Example
==========================================================

Example 1: Basic Sleep Pin Usage
----------------------------------------
Motor configured:
  PWM Pin: 9
  DIR Pin: 8
  Sleep Pin: 7
  Has sleep pin: Yes
  Currently sleeping: No

Example 2: Normal Motor Operation
----------------------------------------
Motor driver awake (sleep disabled)
Running motor at 50% duty cycle forward
    [Simulated delay: 1000 ms]
Running motor at 100 RPM
    [Simulated delay: 2000 ms]
Motor stopped

Example 3: Power-Saving Sleep Mode
----------------------------------------
Motor driver sleeping (low power mode)
  Currently sleeping: Yes
    [Simulated delay: 3000 ms]
Motor driver awake
  Currently sleeping: No
Running motor at 75% duty cycle
    [Simulated delay: 2000 ms]
Motor stopped

Example 4: Fluent API Chaining
----------------------------------------
Left motor configured:
  Sleep pin: 6
  Inverted: Yes
Right motor configured:
  Sleep pin: 5
  Inverted: No

Example 5: Toggle Sleep
----------------------------------------
Cycle 1: Motor is sleeping
    [Simulated delay: 1000 ms]
Cycle 2: Motor is awake
    [Simulated delay: 1000 ms]
Cycle 3: Motor is sleeping
    [Simulated delay: 1000 ms]

Example 6: Battery-Saving Pattern
----------------------------------------
Simulating intermittent robot operation...

Cycle 1:
  Waking up motor driver...
    [Simulated delay: 100 ms]
  Running motor (doing work)...
    [Simulated delay: 1500 ms]
  Stopping motor...
    [Simulated delay: 50 ms]
  Putting motor driver to sleep...
    [Simulated delay: 500 ms]

Cycle 2:
  Waking up motor driver...
    [Simulated delay: 100 ms]
  Running motor (doing work)...
    [Simulated delay: 1500 ms]
  Stopping motor...
    [Simulated delay: 50 ms]
  Putting motor driver to sleep...
    [Simulated delay: 500 ms]

Cycle 3:
  Waking up motor driver...
    [Simulated delay: 100 ms]
  Running motor (doing work)...
    [Simulated delay: 1500 ms]
  Stopping motor...
    [Simulated delay: 50 ms]
  Putting motor driver to sleep...
    [Simulated delay: 500 ms]

==========================================================
Summary
==========================================================
Sleep pin features demonstrated:
  ✓ .withSleepPin() fluent API method
  ✓ enableSleep() / disableSleep() for power control
  ✓ toggleSleep() for alternating states
  ✓ isSleeping() for querying state
  ✓ hasSleepPin() for checking if configured
  ✓ Method chaining with other configurations
  ✓ Battery-saving patterns

Benefits of sleep pin:
  • Reduces power consumption when motors idle
  • Extends battery life in mobile robots
  • Reduces heat generation in motor drivers
  • Provides clean on/off control of all motors

Compatible motor drivers:
  • TB6612FNG (STBY pin)
  • DRV8833 (nSLEEP pin)
  • DRV8835 (MODE/nSLEEP pin)
  • And many others with sleep/standby pins

*/
