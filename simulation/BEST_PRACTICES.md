# Simulation Best Practices

**Guidelines, patterns, and tips for effective robot simulation**

Learn from experience and avoid common pitfalls!

---

## Table of Contents

1. [General Principles](#general-principles)
2. [Code Organization](#code-organization)
3. [Parameter Tuning](#parameter-tuning)
4. [Testing Strategies](#testing-strategies)
5. [Performance Tips](#performance-tips)
6. [Debugging Techniques](#debugging-techniques)
7. [Hardware Transfer](#hardware-transfer)
8. [Common Pitfalls](#common-pitfalls)

---

## General Principles

### ✅ Start Simple, Add Complexity

**Bad:**
```cpp
// Trying to do everything at once
if (distance < 0.3 && lineDetected && encoderCount > 1000 &&
    batteryVoltage < 11.5 && gyroAngle > 45) {
    // Complex behavior
}
```

**Good:**
```cpp
// Build up incrementally
// Step 1: Just drive forward
robot.setMotorPWM(0.5, 0.5);

// Step 2: Add obstacle avoidance
if (distance < 0.3) {
    robot.setMotorPWM(-0.3, 0.3);
}

// Step 3: Add line following
// ... later
```

**Why:** Easier to debug, understand, and validate.

---

### ✅ Use Realistic Parameters

**Bad:**
```cpp
DifferentialDriveSimulator robot(
    0.01,   // 1cm wheelbase?? Too small!
    0.001,  // 1mm radius?? Unrealistic!
    10,     // Only 10 counts/meter? Poor resolution!
    100.0,  // 100m world? Unnecessarily large!
    100.0
);
```

**Good:**
```cpp
// Based on real FRC robot
DifferentialDriveSimulator robot(
    0.60,   // 60cm wheelbase (typical)
    0.35,   // 35cm radius (bumpers)
    2048,   // NEO encoder: 2048 counts/rev × gear ratio
    16.46,  // FRC field width (54 feet)
    8.23    // FRC field height (27 feet)
);
```

**Why:** Sim-to-real transfer works better with realistic parameters.

---

### ✅ Match Your Hardware

```cpp
// Measure your actual robot!
const double MEASURED_WHEELBASE = 0.58;  // Measured with ruler
const double MEASURED_WHEEL_DIAMETER = 0.15;  // 15cm wheels
const double GEAR_RATIO = 10.71;
const double ENCODER_CPR = 42;  // Counts per revolution

double countsPerMeter =
    (ENCODER_CPR * GEAR_RATIO) / (M_PI * WHEEL_DIAMETER);

DifferentialDriveSimulator robot(
    MEASURED_WHEELBASE,
    /* radius */ 0.33,
    countsPerMeter,
    /* world size */
);
```

---

### ✅ Test Edge Cases

Don't just test the happy path!

```cpp
// Test these scenarios:
// 1. Robot starts at world boundary
robot.setPosition(0.1, 0.1, 0);

// 2. Robot facing obstacle directly
robot.setPosition(2.9, 1.5, 0);  // Wall at x=3.0

// 3. Robot in corner
robot.setPosition(0.1, 0.1, M_PI/4);

// 4. Multiple obstacles nearby
robot.addObstacle(Rectangle(2.0, 1.5, 0.5, 0.5));
robot.addObstacle(Rectangle(2.1, 1.5, 0.5, 0.5));  // Touching

// 5. Extreme motor commands
robot.setMotorPWM(1.0, -1.0);  // Max differential
```

---

## Code Organization

### ✅ Separate Configuration from Logic

**Bad:**
```cpp
int main() {
    DifferentialDriveSimulator robot(0.15, 0.10, 1000, 4.0, 3.0);
    // Magic numbers everywhere!
    robot.setMotorPWM(0.5, 0.5);
    // ...
}
```

**Good:**
```cpp
// config.h
struct RobotConfig {
    static constexpr double WHEELBASE = 0.15;
    static constexpr double RADIUS = 0.10;
    static constexpr double ENCODER_CPM = 1000;

    static constexpr double MAX_SPEED = 0.5;
    static constexpr double TURN_SPEED = 0.3;

    static constexpr double OBSTACLE_THRESHOLD = 0.3;
};

// main.cpp
int main() {
    using Cfg = RobotConfig;

    DifferentialDriveSimulator robot(
        Cfg::WHEELBASE,
        Cfg::RADIUS,
        Cfg::ENCODER_CPM,
        4.0, 3.0
    );

    if (distance < Cfg::OBSTACLE_THRESHOLD) {
        robot.setMotorPWM(-Cfg::TURN_SPEED, Cfg::TURN_SPEED);
    }
}
```

**Why:** Easy to tune, reuse, and understand.

---

### ✅ Use Functions for Behaviors

**Bad:**
```cpp
// 200 lines of code in main()
while (viz.isRunning()) {
    // ... complex logic ...
    // ... more logic ...
    // ... even more ...
}
```

**Good:**
```cpp
void obstacleAvoidance(DifferentialDriveSimulator& robot) {
    double front = robot.getUltrasonicDistance(0);
    if (front < 0.3) {
        robot.setMotorPWM(-0.3, 0.3);
    } else {
        robot.setMotorPWM(0.5, 0.5);
    }
}

void lineFollowing(DifferentialDriveSimulator& robot, PIDController& pid) {
    // ... line following logic ...
}

int main() {
    // Clean and readable!
    while (viz.isRunning()) {
        obstacleAvoidance(robot);
        // or: lineFollowing(robot, pid);
    }
}
```

---

### ✅ State Machines for Complex Behavior

**Bad:**
```cpp
bool foundTarget = false;
bool approaching = false;
bool atTarget = false;
bool returning = false;
// Hard to follow flow!
```

**Good:**
```cpp
enum class State {
    SEARCH,
    APPROACH,
    GRAB,
    RETURN,
    DONE
};

State currentState = State::SEARCH;

switch (currentState) {
    case State::SEARCH:
        // Search behavior
        if (targetFound) {
            currentState = State::APPROACH;
        }
        break;

    case State::APPROACH:
        // Approach behavior
        if (atTarget) {
            currentState = State::GRAB;
        }
        break;

    // ... etc
}
```

---

## Parameter Tuning

### ✅ PID Tuning Process

**Step 1: Start with P only**

```cpp
PIDController pid(1.0, 0.0, 0.0);  // Only P

// Tune P until oscillation starts
// Too low: Slow response, steady error
// Too high: Oscillates wildly
// Just right: Fast approach, slight overshoot
```

**Step 2: Add D to reduce oscillation**

```cpp
PIDController pid(1.0, 0.0, 0.1);  // P + D

// D dampens oscillation
// Increase D until oscillation stops
```

**Step 3: Add I for steady-state error**

```cpp
PIDController pid(1.0, 0.05, 0.1);  // P + I + D

// I eliminates steady-state error
// Start small (0.01-0.1)
// Too high → integral windup
```

**Step 4: Fine tune**

```cpp
// Make small adjustments (10-20% at a time)
PIDController pid(1.2, 0.08, 0.12);
```

---

### ✅ Parameter Logging

Track parameters to find optimal values:

```cpp
struct ParameterSet {
    double kP, kI, kD;
    double avgError;
    double settlingTime;
    double overshoot;
};

std::vector<ParameterSet> results;

// Test different parameters
for (double kP = 0.5; kP <= 3.0; kP += 0.5) {
    for (double kI = 0.0; kI <= 0.5; kI += 0.1) {
        // Run simulation
        // Measure performance
        // Store results
    }
}

// Find best parameters
auto best = std::min_element(results.begin(), results.end(),
    [](const auto& a, const auto& b) {
        return a.avgError < b.avgError;
    });
```

---

### ✅ Use Named Constants

**Bad:**
```cpp
if (distance < 0.3) {  // Why 0.3?
    robot.setMotorPWM(0.5, -0.5);  // Why 0.5?
}
```

**Good:**
```cpp
const double SAFE_DISTANCE = 0.3;  // meters
const double TURN_POWER = 0.5;     // 50% PWM

if (distance < SAFE_DISTANCE) {
    robot.setMotorPWM(TURN_POWER, -TURN_POWER);
}
```

---

## Testing Strategies

### ✅ Unit Test Individual Components

```cpp
void testPIDController() {
    PIDController pid(1.0, 0.1, 0.05);

    // Test 1: Zero error should give zero output
    double out1 = pid.calculate(5.0, 5.0, 0.02);
    assert(std::abs(out1) < 0.001);

    // Test 2: Positive error should give positive output
    double out2 = pid.calculate(5.0, 3.0, 0.02);
    assert(out2 > 0);

    // Test 3: Integral should accumulate
    for (int i = 0; i < 10; i++) {
        pid.calculate(5.0, 4.0, 0.02);
    }
    double out3 = pid.calculate(5.0, 4.0, 0.02);
    assert(out3 > out2);  // Integral accumulated

    std::cout << "✓ PID tests passed\n";
}
```

---

### ✅ Regression Testing

Save known-good trajectories:

```cpp
void recordTrajectory(const std::string& filename) {
    std::ofstream file(filename);

    while (running) {
        file << robot.getX() << ","
             << robot.getY() << ","
             << robot.getTheta() << "\n";

        // ... run simulation ...
    }
}

bool compareTrajectory(const std::string& filename) {
    // Load recorded trajectory
    // Run simulation again
    // Compare positions (with tolerance)
    // Return true if similar enough
}
```

---

### ✅ Automated Testing

```cpp
struct TestCase {
    std::string name;
    Point2D start;
    Point2D goal;
    std::vector<Rectangle> obstacles;
    double timeLimit;
};

bool runTest(const TestCase& test) {
    DifferentialDriveSimulator robot(/* ... */);
    robot.setPosition(test.start.x, test.start.y, 0);

    for (const auto& obs : test.obstacles) {
        robot.addObstacle(obs);
    }

    double elapsed = 0;
    while (elapsed < test.timeLimit) {
        // Run control logic
        robot.update(0.02);
        elapsed += 0.02;

        // Check if goal reached
        double dx = robot.getX() - test.goal.x;
        double dy = robot.getY() - test.goal.y;
        if (sqrt(dx*dx + dy*dy) < 0.2) {
            return true;  // Success!
        }
    }

    return false;  // Timeout
}

int main() {
    std::vector<TestCase> tests = {
        {"Simple forward", {0.5, 1.5}, {3.5, 1.5}, {}, 10.0},
        {"With obstacle", {0.5, 1.5}, {3.5, 1.5},
            {Rectangle(2.0, 1.5, 0.5, 0.5)}, 15.0},
        // ... more tests ...
    };

    int passed = 0;
    for (const auto& test : tests) {
        if (runTest(test)) {
            std::cout << "✓ " << test.name << "\n";
            passed++;
        } else {
            std::cout << "✗ " << test.name << "\n";
        }
    }

    std::cout << passed << "/" << tests.size() << " tests passed\n";
}
```

---

## Performance Tips

### ✅ Choose Appropriate Timestep

```cpp
// Too small (0.001s = 1ms):
// - Very accurate
// - Slow simulation
// - Overkill for most robots

// Too large (0.1s = 100ms):
// - Fast simulation
// - Inaccurate physics
// - Misses collisions

// Just right (0.01-0.02s = 10-20ms):
double dt = 0.02;  // 50 Hz - good balance
```

---

### ✅ Limit Path History

```cpp
// Bad: Unlimited path storage
std::vector<Point2D> path;  // Grows forever!

// Good: Limited path
std::vector<Point2D> path;
const size_t MAX_PATH_POINTS = 1000;

if (path.size() > MAX_PATH_POINTS) {
    path.erase(path.begin());  // Remove oldest
}
```

---

### ✅ Reduce Obstacle Count

```cpp
// Bad: Individual cells
for (int y = 0; y < 100; y++) {
    for (int x = 0; x < 100; x++) {
        if (isObstacle(x, y)) {
            robot.addObstacle(Rectangle(x*0.1, y*0.1, 0.1, 0.1));
        }
    }
}
// 10,000 obstacles!

// Good: Merge adjacent obstacles
// Create larger rectangles from adjacent cells
// Result: ~50 obstacles
```

---

## Debugging Techniques

### ✅ Visual Debugging

```cpp
// Add print statements for visualization
void debugPrint(const DifferentialDriveSimulator& robot) {
    static int frame = 0;
    if (frame++ % 50 == 0) {  // Print every 50 frames
        std::cout << "Pos: (" << robot.getX() << ", " << robot.getY() << ")"
                  << " Theta: " << robot.getTheta()
                  << " Vel: (" << robot.getLeftVelocity() << ", "
                  << robot.getRightVelocity() << ")\n";
    }
}
```

---

### ✅ Record and Replay

```cpp
struct FrameData {
    double time;
    double x, y, theta;
    double leftVel, rightVel;
    double sensorDist;
};

std::vector<FrameData> recording;

// Record mode
recording.push_back({
    SDL_GetTicks() / 1000.0,
    robot.getX(), robot.getY(), robot.getTheta(),
    robot.getLeftVelocity(), robot.getRightVelocity(),
    robot.getUltrasonicDistance(0)
});

// Replay mode
for (const auto& frame : recording) {
    robot.setPosition(frame.x, frame.y, frame.theta);
    viz.render(robot);
    SDL_Delay(20);
}
```

---

### ✅ Slow Motion

```cpp
// Normal speed
double dt = 0.02;
SDL_Delay((uint32_t)(dt * 1000));

// Slow motion (2x slower)
double dt = 0.02;
SDL_Delay((uint32_t)(dt * 2000));  // 2x delay

// Or adjust timestep
double dt = 0.02 * 0.5;  // Half speed physics
SDL_Delay((uint32_t)(0.02 * 1000));  // Normal render speed
```

---

## Hardware Transfer

### ✅ Use Same Code Structure

```cpp
// Shared robot interface
class RobotInterface {
public:
    virtual void setMotorPWM(double left, double right) = 0;
    virtual double getUltrasonicDistance(double angle) = 0;
    virtual void update(double dt) = 0;
    // ... other methods ...
};

// Simulation implementation
class SimRobot : public RobotInterface {
    DifferentialDriveSimulator sim_;
    // Implement interface using simulator
};

// Hardware implementation
class RealRobot : public RobotInterface {
    // Implement interface using real hardware
    void setMotorPWM(double left, double right) override {
        // Arduino: analogWrite(), etc.
    }
};

// Same control code works for both!
void driveLogic(RobotInterface& robot) {
    double dist = robot.getUltrasonicDistance(0);
    if (dist < 0.3) {
        robot.setMotorPWM(-0.3, 0.3);
    }
}
```

---

### ✅ Calibration Process

1. **Measure actual robot dimensions**
2. **Test basic movements** (drive 1m, turn 90°)
3. **Calibrate encoders** (counts per meter)
4. **Calibrate sensors** (distance accuracy)
5. **Tune control parameters** on real robot

---

### ✅ Safety First

```cpp
// Add safety limits for real robot
const double MAX_SAFE_SPEED = 0.5;  // m/s
const double MIN_SAFE_DISTANCE = 0.2;  // m

void safeSetMotorPWM(double left, double right) {
    // Clamp speeds
    left = std::max(-MAX_SAFE_SPEED, std::min(MAX_SAFE_SPEED, left));
    right = std::max(-MAX_SAFE_SPEED, std::min(MAX_SAFE_SPEED, right));

    // Emergency stop if too close
    if (getUltrasonicDistance(0) < MIN_SAFE_DISTANCE) {
        left = 0;
        right = 0;
    }

    setMotorPWM(left, right);
}
```

---

## Common Pitfalls

### ❌ Radians vs Degrees

```cpp
// WRONG!
robot.getUltrasonicDistance(90);  // 90 radians!! Not 90°

// CORRECT!
robot.getUltrasonicDistance(M_PI / 2);  // 90° = π/2 radians
robot.getUltrasonicDistance(deg(90).toRadians());  // Using RobotLib units
```

---

### ❌ Forgetting to Call update()

```cpp
// WRONG!
while (viz.isRunning()) {
    robot.setMotorPWM(0.5, 0.5);
    viz.render(robot);  // Robot doesn't move!
}

// CORRECT!
while (viz.isRunning()) {
    robot.setMotorPWM(0.5, 0.5);
    robot.update(dt);  // Actually update physics!
    viz.render(robot);
}
```

---

### ❌ Not Normalizing Angles

```cpp
// WRONG!
double error = targetAngle - currentAngle;
// Might be +350° instead of -10°!

// CORRECT!
double error = targetAngle - currentAngle;
while (error > M_PI) error -= 2 * M_PI;
while (error < -M_PI) error += 2 * M_PI;
// Now in range [-π, π]
```

---

### ❌ Integer Division

```cpp
// WRONG!
double speed = 5 / 2;  // = 2.0 (integer division!)

// CORRECT!
double speed = 5.0 / 2.0;  // = 2.5
// or:
double speed = 5 / 2.0;  // = 2.5
```

---

### ❌ Not Checking for NaN/Inf

```cpp
// WRONG!
double result = someCalculation();
robot.setMotorPWM(result, result);  // Might be NaN!

// CORRECT!
double result = someCalculation();
if (std::isnan(result) || std::isinf(result)) {
    result = 0;  // Safe fallback
}
robot.setMotorPWM(result, result);
```

---

## Checklist for Success

Before starting:
- [ ] SDL2 installed and tested
- [ ] Examples compile and run
- [ ] Understand robot dimensions

During development:
- [ ] Start with simple behavior
- [ ] Test incrementally
- [ ] Use realistic parameters
- [ ] Add logging/debugging
- [ ] Test edge cases

Before hardware transfer:
- [ ] Code well-organized
- [ ] Parameters in constants
- [ ] Safety checks added
- [ ] Calibration plan ready
- [ ] Backup strategy in place

---

## Resources

- [Tutorials](TUTORIALS.md) - Step-by-step guides
- [API Reference](API_REFERENCE.md) - Complete API docs
- [Advanced Features](ADVANCED_FEATURES.md) - Complex techniques
- [Installation](INSTALL.md) - SDL2 setup

---

**Remember:** Simulation is a tool, not a replacement for hardware testing!

**Happy simulating!** 🤖✨

---

**Questions or suggestions?** [Open an issue](https://github.com/konnorreynolds/RobotLib/issues)

**Last Updated:** November 7, 2025
