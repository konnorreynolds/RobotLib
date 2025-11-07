# RobotLib Simulation Tutorials

**Step-by-Step Guides for Building Simulated Robots**

Learn to use the RobotLib simulation system through hands-on tutorials, from beginner to advanced.

---

## Table of Contents

### Beginner Tutorials
1. [Your First Simulation](#tutorial-1-your-first-simulation)
2. [Adding Sensors](#tutorial-2-adding-sensors)
3. [Simple Navigation](#tutorial-3-simple-navigation)

### Intermediate Tutorials
4. [PID Line Following](#tutorial-4-pid-line-following)
5. [Wall Following Robot](#tutorial-5-wall-following-robot)
6. [Waypoint Navigation](#tutorial-6-waypoint-navigation)

### Advanced Tutorials
7. [SLAM Simulation](#tutorial-7-slam-simulation)
8. [Multi-Robot Coordination](#tutorial-8-multi-robot-coordination)
9. [Competition Auto Routine](#tutorial-9-competition-auto-routine)

---

## Tutorial 1: Your First Simulation

**Time:** 15 minutes
**Difficulty:** ⭐ Beginner
**Goal:** Create a simple robot that drives forward and stops when it detects a wall

### Step 1: Set Up Your Project

Create a new file `my_first_sim.cpp`:

```cpp
#include "../include/RobotLib.h"
#include "../include/units_simulation.h"
#include "../include/units_visualization.h"

using namespace robotlib::simulation;
using namespace robotlib::visualization;
using namespace units;

int main() {
    // We'll add code here!
    return 0;
}
```

### Step 2: Create the Robot

Add inside `main()`:

```cpp
// Create a differential drive robot
DifferentialDriveSimulator robot(
    0.15,  // Wheelbase: 15cm (distance between wheels)
    0.10,  // Radius: 10cm (size of robot)
    1000,  // Encoder resolution: 1000 counts per meter
    4.0,   // World width: 4 meters
    3.0    // World height: 3 meters
);

// Start robot at position (0.5m, 1.5m) facing right (0 radians)
robot.setPosition(0.5, 1.5, 0);
```

**💡 Explanation:**
- `0.15` wheelbase = distance from left to right wheel
- `0.10` radius = robot size for collision detection
- `1000` counts/meter = encoder resolution (like a real robot)
- `4.0 x 3.0` = size of virtual world

### Step 3: Create the Visualizer

```cpp
// Create visualization window
RobotVisualizer viz(
    800,   // Window width in pixels
    600,   // Window height in pixels
    150,   // Scale: 150 pixels = 1 meter
    "My First Robot Simulation"  // Window title
);
```

### Step 4: Add an Obstacle (Wall)

```cpp
// Add a wall obstacle
robot.addObstacle(Rectangle(
    3.0,   // X position: 3 meters from left
    1.5,   // Y position: 1.5 meters from bottom
    0.1,   // Width: 10cm thick
    2.0    // Height: 2 meters tall
));
```

### Step 5: Create the Main Loop

```cpp
double dt = 0.02;  // 20ms time step (50 Hz simulation)

while (viz.isRunning()) {
    // Handle window events (close button, ESC key, etc.)
    viz.handleEvents();

    // Read ultrasonic sensor (forward direction)
    double distance = robot.getUltrasonicDistance(0);

    // Simple behavior: drive forward until obstacle detected
    if (distance > 0.5) {
        // Clear path - drive forward
        robot.setMotorPWM(0.5, 0.5);  // 50% power, both motors
    } else {
        // Obstacle detected - stop!
        robot.stop();
    }

    // Update robot physics
    robot.update(dt);

    // Render visualization
    viz.render(robot);

    // Wait for next frame
    SDL_Delay((uint32_t)(dt * 1000));
}
```

### Step 6: Compile and Run

```bash
g++ -std=c++11 -I../include my_first_sim.cpp -lSDL2 -o my_first_sim
./my_first_sim
```

### What You Should See

- A blue robot (circle) on the left
- A gray wall on the right
- Robot drives forward
- Robot stops when it gets close to the wall
- Red path line shows where robot has been

### 🎓 What You Learned

✅ How to create a simulated robot
✅ How to add obstacles to the world
✅ How to read simulated sensors
✅ How to control motors with PWM
✅ How to update physics and render visualization

### 🎯 Challenge

Make the robot:
1. Back up after stopping
2. Turn 90° and continue
3. Stop after 5 seconds (use `SDL_GetTicks()`)

---

## Tutorial 2: Adding Sensors

**Time:** 20 minutes
**Difficulty:** ⭐ Beginner
**Goal:** Use multiple sensors for better obstacle detection

### Step 1: Multi-Direction Ultrasonic Sensors

Modify the sensor reading part:

```cpp
// Read multiple ultrasonic sensors
double frontDist = robot.getUltrasonicDistance(0);           // 0° (forward)
double leftDist = robot.getUltrasonicDistance(M_PI / 4);     // 45° left
double rightDist = robot.getUltrasonicDistance(-M_PI / 4);   // 45° right

// Decision logic using all sensors
if (frontDist < 0.3) {
    // Front obstacle - turn
    robot.setMotorPWM(-0.3, 0.3);  // Turn left
} else if (leftDist < 0.2) {
    // Left obstacle - turn right
    robot.setMotorPWM(0.5, 0.3);
} else if (rightDist < 0.2) {
    // Right obstacle - turn left
    robot.setMotorPWM(0.3, 0.5);
} else {
    // Clear - go forward
    robot.setMotorPWM(0.5, 0.5);
}
```

### Step 2: Visualize Sensor Rays

The visualizer already shows sensor rays! Press `T` to toggle path tracing, `C` to clear path.

### Step 3: Add Line Sensors

```cpp
// Simulate 3 IR line sensors
bool leftLine = robot.getLineSensor(0.08, -0.03);   // 8cm forward, 3cm left
bool centerLine = robot.getLineSensor(0.08, 0);      // 8cm forward, center
bool rightLine = robot.getLineSensor(0.08, 0.03);    // 8cm forward, 3cm right

// Print sensor states
if (centerLine) {
    std::cout << "On line!\n";
}
```

### Step 4: Encoder Odometry

```cpp
// Read encoders
long leftCount = robot.getLeftEncoder();
long rightCount = robot.getRightEncoder();

// Calculate distance traveled
double leftDist = leftCount / robot.getCountsPerMeter();
double rightDist = rightCount / robot.getCountsPerMeter();
double avgDist = (leftDist + rightDist) / 2.0;

std::cout << "Traveled: " << avgDist << " meters\n";

// Reset encoders when needed
robot.resetEncoders();
```

### 🎓 What You Learned

✅ Multiple ultrasonic sensor directions
✅ Line sensor simulation
✅ Encoder reading and odometry
✅ Sensor fusion for decision making

---

## Tutorial 3: Simple Navigation

**Time:** 30 minutes
**Difficulty:** ⭐⭐ Intermediate
**Goal:** Navigate to a target position using sensors

### Complete Example: Go-to-Goal Behavior

```cpp
#include "../include/RobotLib.h"
#include "../include/units_simulation.h"
#include "../include/units_visualization.h"
#include <cmath>

using namespace robotlib::simulation;
using namespace robotlib::visualization;

int main() {
    DifferentialDriveSimulator robot(0.15, 0.10, 1000, 5.0, 5.0);
    robot.setPosition(0.5, 0.5, 0);

    // Add some obstacles
    robot.addObstacle(Rectangle(2.5, 2.5, 0.5, 0.5));
    robot.addObstacle(Rectangle(3.5, 1.5, 0.3, 0.8));

    RobotVisualizer viz(800, 600, 120, "Go-to-Goal Navigation");

    // Goal position
    Point2D goal(4.0, 4.0);
    double goalTolerance = 0.2;  // 20cm tolerance

    double dt = 0.02;

    while (viz.isRunning()) {
        viz.handleEvents();

        // Calculate error to goal
        double dx = goal.x - robot.getX();
        double dy = goal.y - robot.getY();
        double distanceToGoal = std::sqrt(dx * dx + dy * dy);
        double angleToGoal = std::atan2(dy, dx);

        // Calculate heading error
        double headingError = angleToGoal - robot.getTheta();

        // Normalize to [-PI, PI]
        while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;

        if (distanceToGoal > goalTolerance) {
            // Not at goal yet

            // Check for obstacles
            double frontDist = robot.getUltrasonicDistance(0);

            if (frontDist < 0.3) {
                // Obstacle ahead - turn to avoid
                robot.setMotorPWM(-0.3, 0.3);
            } else {
                // Clear path - navigate to goal

                // Proportional control for heading
                double turnRate = headingError * 2.0;  // P gain = 2.0
                turnRate = std::max(-1.0, std::min(1.0, turnRate));  // Clamp

                // Differential drive control
                double baseSpeed = 0.4;
                double leftPWM = baseSpeed + turnRate;
                double rightPWM = baseSpeed - turnRate;

                robot.setMotorPWM(leftPWM, rightPWM);
            }
        } else {
            // Reached goal!
            robot.stop();
            std::cout << "Goal reached!\n";
        }

        robot.update(dt);
        viz.render(robot);
        SDL_Delay((uint32_t)(dt * 1000));
    }

    return 0;
}
```

### 🎓 What You Learned

✅ Goal-seeking navigation
✅ Angle calculations and normalization
✅ Proportional control for heading
✅ Obstacle avoidance while navigating

---

## Tutorial 4: PID Line Following

**Time:** 45 minutes
**Difficulty:** ⭐⭐ Intermediate
**Goal:** Follow a line using PID control

### Step 1: Set Up Line Follower

```cpp
#include "../include/RobotLib.h"
#include "../include/units_simulation.h"
#include "../include/units_visualization.h"

using namespace robotlib::simulation;
using namespace robotlib::visualization;
using namespace units;
using namespace robotics;

int main() {
    DifferentialDriveSimulator robot(0.15, 0.10, 1000, 6.0, 3.0);
    robot.setPosition(0.5, 1.4, 0);  // Start just left of center line

    RobotVisualizer viz(900, 450, 150, "PID Line Follower");

    // Line is at y = 1.5 (center of world)
    double lineY = 1.5;

    // Create PID controller for line following
    PIDController pid(
        1.5,   // kP: Proportional gain
        0.3,   // kI: Integral gain
        0.1    // kD: Derivative gain
    );

    double dt = 0.02;
    double baseSpeed = 0.3;

    while (viz.isRunning()) {
        viz.handleEvents();

        // Read 5 line sensors
        bool sensor[5];
        sensor[0] = robot.getLineSensor(0.08, -0.04);  // Far left
        sensor[1] = robot.getLineSensor(0.08, -0.02);  // Left
        sensor[2] = robot.getLineSensor(0.08, 0);      // Center
        sensor[3] = robot.getLineSensor(0.08, 0.02);   // Right
        sensor[4] = robot.getLineSensor(0.08, 0.04);   // Far right

        // Calculate weighted line position
        // -2 = far left, 0 = center, +2 = far right
        double linePosition = 0;
        int sensorsActive = 0;

        for (int i = 0; i < 5; i++) {
            if (sensor[i]) {
                linePosition += (i - 2);  // -2, -1, 0, 1, 2
                sensorsActive++;
            }
        }

        if (sensorsActive > 0) {
            linePosition /= sensorsActive;
        }

        // PID control
        double correction = pid.calculate(
            0.0,            // Setpoint: 0 (line in center)
            linePosition,   // Current position
            dt
        );

        // Apply correction to motors
        double leftPWM = baseSpeed - correction;
        double rightPWM = baseSpeed + correction;

        robot.setMotorPWM(leftPWM, rightPWM);

        robot.update(dt);
        viz.render(robot);
        SDL_Delay((uint32_t)(dt * 1000));
    }

    return 0;
}
```

### Step 2: Tune PID Parameters

Try different values:

```cpp
// Aggressive (fast, might oscillate)
PIDController pid(2.0, 0.5, 0.2);

// Smooth (slow, stable)
PIDController pid(1.0, 0.1, 0.05);

// Only P (simple, might have steady-state error)
PIDController pid(1.5, 0.0, 0.0);
```

### 🎓 What You Learned

✅ PID controller implementation
✅ Sensor array weighted average
✅ Line following algorithm
✅ PID tuning process

---

## Tutorial 5: Wall Following Robot

**Time:** 30 minutes
**Difficulty:** ⭐⭐ Intermediate
**Goal:** Follow along a wall at constant distance

```cpp
int main() {
    DifferentialDriveSimulator robot(0.15, 0.10, 1000, 6.0, 4.0);
    robot.setPosition(0.5, 2.0, 0);

    // Create a wall to follow
    robot.addObstacle(Rectangle(3.0, 2.0, 0.1, 3.0));

    RobotVisualizer viz(900, 600, 150, "Wall Follower");

    double desiredDistance = 0.4;  // Follow wall at 40cm
    PIDController wallPID(2.0, 0.1, 0.1);

    double dt = 0.02;

    while (viz.isRunning()) {
        viz.handleEvents();

        // Read side sensor (perpendicular to robot)
        double sideDistance = robot.getUltrasonicDistance(M_PI / 2);  // 90° left

        // PID to maintain distance
        double error = desiredDistance - sideDistance;
        double correction = wallPID.calculate(0, error, dt);

        // Move forward while correcting angle
        double baseSpeed = 0.4;
        robot.setMotorPWM(
            baseSpeed + correction,
            baseSpeed - correction
        );

        robot.update(dt);
        viz.render(robot);
        SDL_Delay((uint32_t)(dt * 1000));
    }

    return 0;
}
```

---

## Tutorial 6: Waypoint Navigation

**Time:** 45 minutes
**Difficulty:** ⭐⭐⭐ Advanced
**Goal:** Navigate through multiple waypoints

```cpp
struct Waypoint {
    double x, y;
    double tolerance;
};

int main() {
    DifferentialDriveSimulator robot(0.15, 0.10, 1000, 8.0, 8.0);
    robot.setPosition(1.0, 1.0, 0);

    // Create waypoint path
    std::vector<Waypoint> waypoints = {
        {2.0, 2.0, 0.2},
        {4.0, 3.0, 0.2},
        {6.0, 2.0, 0.2},
        {6.0, 6.0, 0.2},
        {2.0, 6.0, 0.2}
    };

    size_t currentWaypoint = 0;

    RobotVisualizer viz(1000, 1000, 125, "Waypoint Navigation");

    PIDController headingPID(3.0, 0.0, 0.2);
    double dt = 0.02;

    while (viz.isRunning() && currentWaypoint < waypoints.size()) {
        viz.handleEvents();

        auto& wp = waypoints[currentWaypoint];

        // Calculate error
        double dx = wp.x - robot.getX();
        double dy = wp.y - robot.getY();
        double distance = std::sqrt(dx * dx + dy * dy);
        double targetAngle = std::atan2(dy, dx);

        if (distance < wp.tolerance) {
            // Reached waypoint!
            std::cout << "Waypoint " << currentWaypoint << " reached!\n";
            currentWaypoint++;
            headingPID.reset();
            continue;
        }

        // Calculate heading error
        double headingError = targetAngle - robot.getTheta();
        while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;

        // PID control
        double turn = headingPID.calculate(0, headingError, dt);

        // Speed based on distance
        double speed = std::min(0.5, distance * 0.3);

        robot.setMotorPWM(speed + turn, speed - turn);

        robot.update(dt);
        viz.render(robot);
        SDL_Delay((uint32_t)(dt * 1000));
    }

    std::cout << "All waypoints reached!\n";
    return 0;
}
```

---

## Tutorial 7: SLAM Simulation

**Time:** 60+ minutes
**Difficulty:** ⭐⭐⭐ Advanced
**Goal:** Implement basic SLAM (Simultaneous Localization and Mapping)

See `ADVANCED_FEATURES.md` for complete SLAM tutorial with particle filters and mapping.

---

## Tutorial 8: Multi-Robot Coordination

**Time:** 45 minutes
**Difficulty:** ⭐⭐⭐ Advanced
**Goal:** Simulate multiple robots working together

See `ADVANCED_FEATURES.md` for multi-robot simulation examples.

---

## Tutorial 9: Competition Auto Routine

**Time:** 60+ minutes
**Difficulty:** ⭐⭐⭐ Advanced
**Goal:** Create FRC-style autonomous routine

### Complete Competition Auto

```cpp
enum class AutoState {
    DRIVE_FORWARD,
    TURN_TO_GOAL,
    APPROACH_GOAL,
    SCORE,
    BACKUP,
    DONE
};

int main() {
    DifferentialDriveSimulator robot(0.15, 0.10, 1000, 16.46, 8.23);
    robot.setPosition(1.5, 2.0, 0);

    // Field obstacles
    robot.addObstacle(Rectangle(8.0, 4.0, 0.6, 0.6));  // Hub

    RobotVisualizer viz(1200, 600, 73, "FRC Auto Routine");

    AutoState state = AutoState::DRIVE_FORWARD;
    double stateStartTime = SDL_GetTicks() / 1000.0;

    double dt = 0.02;

    while (viz.isRunning() && state != AutoState::DONE) {
        viz.handleEvents();

        double currentTime = SDL_GetTicks() / 1000.0;
        double stateTime = currentTime - stateStartTime;

        switch (state) {
            case AutoState::DRIVE_FORWARD:
                robot.setMotorPWM(0.5, 0.5);
                if (stateTime > 2.0) {  // Drive for 2 seconds
                    state = AutoState::TURN_TO_GOAL;
                    stateStartTime = currentTime;
                }
                break;

            case AutoState::TURN_TO_GOAL:
                robot.setMotorPWM(-0.3, 0.3);  // Turn left
                if (stateTime > 1.5) {  // Turn for 1.5 seconds
                    state = AutoState::APPROACH_GOAL;
                    stateStartTime = currentTime;
                }
                break;

            case AutoState::APPROACH_GOAL:
                {
                    double dist = robot.getUltrasonicDistance(0);
                    if (dist < 0.8) {
                        state = AutoState::SCORE;
                        stateStartTime = currentTime;
                        robot.stop();
                    } else {
                        robot.setMotorPWM(0.4, 0.4);
                    }
                }
                break;

            case AutoState::SCORE:
                // Simulated scoring action
                robot.stop();
                if (stateTime > 1.0) {
                    state = AutoState::BACKUP;
                    stateStartTime = currentTime;
                }
                break;

            case AutoState::BACKUP:
                robot.setMotorPWM(-0.3, -0.3);
                if (stateTime > 1.0) {
                    state = AutoState::DONE;
                    robot.stop();
                }
                break;

            case AutoState::DONE:
                robot.stop();
                break;
        }

        robot.update(dt);
        viz.render(robot);
        SDL_Delay((uint32_t)(dt * 1000));
    }

    std::cout << "Auto routine complete!\n";
    return 0;
}
```

### 🎓 What You Learned

✅ State machine implementation
✅ Timed autonomous routines
✅ Sequential action planning
✅ Competition robot programming

---

## Next Steps

1. **Combine tutorials** - Mix and match techniques
2. **Add complexity** - More sensors, better algorithms
3. **Test edge cases** - What happens when things go wrong?
4. **Port to hardware** - Take working sim code to real robot
5. **Read advanced guides** - See `ADVANCED_FEATURES.md`

## Troubleshooting

**Robot spins in circles**
- Check PID gains (too high)
- Check motor PWM signs
- Verify wheelbase parameter

**Robot doesn't move**
- Check motor PWM values (0 to 1 range)
- Verify update() is being called
- Check for collisions

**Sensors don't work**
- Verify sensor angles (radians, not degrees!)
- Check sensor placement offsets
- Ensure obstacles exist

---

## Resources

- [Advanced Features](ADVANCED_FEATURES.md)
- [API Reference](API_REFERENCE.md)
- [Best Practices](BEST_PRACTICES.md)
- [Main README](README.md)

---

**Happy learning!** 🎓🤖

*Questions?* [Open an issue](https://github.com/konnorreynolds/RobotLib/issues)
