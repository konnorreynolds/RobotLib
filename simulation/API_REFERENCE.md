# Simulation API Reference

**Complete API documentation for RobotLib simulation system**

Version: 2.2.0

---

## Table of Contents

1. [DifferentialDriveSimulator](#differentialdrivesimulator)
2. [RobotVisualizer](#robotvisualizer)
3. [Geometry Types](#geometry-types)
4. [Helper Functions](#helper-functions)

---

## DifferentialDriveSimulator

**Header:** `units_simulation.h`
**Namespace:** `robotlib::simulation`

Simulates a differential drive (tank drive) robot with realistic physics and sensors.

### Constructor

```cpp
DifferentialDriveSimulator(
    double wheelbase = 0.15,
    double robotRadius = 0.10,
    double countsPerMeter = 1000,
    double worldWidth = 4.0,
    double worldHeight = 3.0
);
```

**Parameters:**
- `wheelbase` - Distance between left and right wheels (meters)
- `robotRadius` - Robot radius for collision detection (meters)
- `countsPerMeter` - Encoder resolution (counts per meter of wheel travel)
- `worldWidth` - Width of simulation world (meters)
- `worldHeight` - Height of simulation world (meters)

**Example:**
```cpp
// Create 15cm wheelbase robot in 4m x 3m world
DifferentialDriveSimulator robot(0.15, 0.10, 1000, 4.0, 3.0);
```

---

### Motor Control Methods

#### setWheelVelocities()

```cpp
void setWheelVelocities(double leftVel, double rightVel);
```

Set wheel velocities directly in m/s.

**Parameters:**
- `leftVel` - Left wheel velocity (m/s, positive = forward)
- `rightVel` - Right wheel velocity (m/s, positive = forward)

**Example:**
```cpp
robot.setWheelVelocities(0.3, 0.3);  // Both wheels 0.3 m/s forward
robot.setWheelVelocities(0.3, -0.3); // Spin in place
```

---

#### setMotorPWM()

```cpp
void setMotorPWM(double leftPWM, double rightPWM);
```

Set motor power using PWM values (-1.0 to 1.0).

**Parameters:**
- `leftPWM` - Left motor PWM (-1.0 to 1.0)
- `rightPWM` - Right motor PWM (-1.0 to 1.0)

**Note:** PWM is converted to velocity using simple model: velocity = PWM * 0.5 m/s

**Example:**
```cpp
robot.setMotorPWM(0.5, 0.5);   // Forward at 50% power
robot.setMotorPWM(-0.5, -0.5); // Backward at 50% power
robot.setMotorPWM(0.5, -0.5);  // Turn right
```

---

#### stop()

```cpp
void stop();
```

Stop both motors immediately.

**Example:**
```cpp
robot.stop();  // Emergency stop
```

---

### Physics Update

#### update()

```cpp
void update(double dt);
```

Update robot physics simulation for one time step.

**Parameters:**
- `dt` - Time step in seconds (typically 0.01 to 0.05)

**What it does:**
1. Calculates robot velocities from wheel speeds
2. Updates position and orientation
3. Checks for collisions
4. Updates encoder counts
5. Applies noise

**Example:**
```cpp
double dt = 0.02;  // 20ms = 50 Hz
robot.update(dt);
```

**Best Practice:** Call at regular intervals matching your control loop frequency.

---

### Sensor Methods

#### getUltrasonicDistance()

```cpp
double getUltrasonicDistance(double sensorAngle = 0) const;
```

Simulate ultrasonic distance sensor.

**Parameters:**
- `sensorAngle` - Sensor direction relative to robot heading (radians)
  - `0` = forward
  - `M_PI/2` = left
  - `-M_PI/2` = right
  - `M_PI/4` = 45° left

**Returns:** Distance to nearest obstacle in meters (0 to max range ~4m)

**Example:**
```cpp
double front = robot.getUltrasonicDistance(0);          // Forward
double left45 = robot.getUltrasonicDistance(M_PI/4);    // 45° left
double right45 = robot.getUltrasonicDistance(-M_PI/4);  // 45° right
```

**Note:** Includes realistic sensor noise (±2cm)

---

#### getLineSensor()

```cpp
bool getLineSensor(double offsetX, double offsetY) const;
```

Simulate IR line sensor.

**Parameters:**
- `offsetX` - Sensor X offset from robot center (m, forward = positive)
- `offsetY` - Sensor Y offset from robot center (m, left = positive)

**Returns:** `true` if line detected, `false` otherwise

**Example:**
```cpp
// 5-sensor array, 8cm in front of robot, 2cm spacing
bool sensors[5];
sensors[0] = robot.getLineSensor(0.08, -0.04);  // Far left
sensors[1] = robot.getLineSensor(0.08, -0.02);  // Left
sensors[2] = robot.getLineSensor(0.08, 0);      // Center
sensors[3] = robot.getLineSensor(0.08, 0.02);   // Right
sensors[4] = robot.getLineSensor(0.08, 0.04);   // Far right
```

**Note:** Line is simulated at y = worldHeight/2 (center of world)

---

#### getLeftEncoder()

```cpp
long getLeftEncoder() const;
```

Get left wheel encoder count.

**Returns:** Encoder ticks (accumulated since last reset)

**Example:**
```cpp
long leftTicks = robot.getLeftEncoder();
double leftDist = leftTicks / 1000.0;  // Convert to meters (if 1000 counts/m)
```

---

#### getRightEncoder()

```cpp
long getRightEncoder() const;
```

Get right wheel encoder count.

**Returns:** Encoder ticks (accumulated since last reset)

---

#### resetEncoders()

```cpp
void resetEncoders();
```

Reset both encoder counts to zero.

**Example:**
```cpp
robot.resetEncoders();
// Now both encoders read 0
```

---

### World Management

#### addObstacle()

```cpp
void addObstacle(const Rectangle& obs);
```

Add a rectangular obstacle to the world.

**Parameters:**
- `obs` - Rectangle obstacle (see Geometry Types)

**Example:**
```cpp
robot.addObstacle(Rectangle(2.0, 1.5, 0.5, 0.5));  // 50cm x 50cm box at (2.0, 1.5)
robot.addObstacle(Rectangle(3.0, 2.0, 0.1, 2.0));  // Wall: 10cm x 2m
```

---

#### clearObstacles()

```cpp
void clearObstacles();
```

Remove all obstacles from the world.

**Example:**
```cpp
robot.clearObstacles();
```

---

#### setPosition()

```cpp
void setPosition(double x, double y, double theta);
```

Set robot pose directly (teleport).

**Parameters:**
- `x` - X position (meters)
- `y` - Y position (meters)
- `theta` - Heading angle (radians, 0 = facing right)

**Example:**
```cpp
robot.setPosition(1.0, 1.5, M_PI/4);  // Position (1.0, 1.5), facing 45°
```

---

### Getters

#### getX()

```cpp
double getX() const;
```

Get robot X position in meters.

---

#### getY()

```cpp
double getY() const;
```

Get robot Y position in meters.

---

#### getTheta()

```cpp
double getTheta() const;
```

Get robot heading in radians (-π to π).
- `0` = facing right (East)
- `π/2` = facing up (North)
- `π` or `-π` = facing left (West)
- `-π/2` = facing down (South)

---

#### getLeftVelocity()

```cpp
double getLeftVelocity() const;
```

Get current left wheel velocity (m/s).

---

#### getRightVelocity()

```cpp
double getRightVelocity() const;
```

Get current right wheel velocity (m/s).

---

#### getWheelbase()

```cpp
double getWheelbase() const;
```

Get wheelbase distance (m).

---

#### getRobotRadius()

```cpp
double getRobotRadius() const;
```

Get robot radius for collision (m).

---

#### getWorldWidth()

```cpp
double getWorldWidth() const;
```

Get world width (m).

---

#### getWorldHeight()

```cpp
double getWorldHeight() const;
```

Get world height (m).

---

#### getObstacles()

```cpp
const std::vector<Rectangle>& getObstacles() const;
```

Get list of all obstacles.

**Returns:** Vector of Rectangle obstacles

---

## RobotVisualizer

**Header:** `units_visualization.h`
**Namespace:** `robotlib::visualization`

Real-time SDL2-based visualization of robot simulation.

### Constructor

```cpp
RobotVisualizer(
    int width = 800,
    int height = 600,
    double scale = 100.0,
    const char* title = "RobotLib Simulator"
);
```

**Parameters:**
- `width` - Window width in pixels
- `height` - Window height in pixels
- `scale` - Pixels per meter (zoom level)
- `title` - Window title string

**Example:**
```cpp
RobotVisualizer viz(800, 600, 150, "My Robot");
// 800x600 window, 150 pixels per meter
```

---

### Event Handling

#### handleEvents()

```cpp
bool handleEvents();
```

Process window events (close, keyboard, etc.).

**Returns:** `true` if should continue, `false` if window closed

**Example:**
```cpp
while (viz.isRunning()) {
    viz.handleEvents();  // Must call each frame
    // ... rest of loop
}
```

**Built-in Keys:**
- `ESC` - Close window
- `T` - Toggle path tracing
- `C` - Clear path

---

#### isRunning()

```cpp
bool isRunning() const;
```

Check if visualization window is still open.

**Returns:** `true` if window open, `false` if closed

---

### Rendering

#### render()

```cpp
void render(const DifferentialDriveSimulator& robot, double fps = 0);
```

Render complete frame with robot, obstacles, sensors, etc.

**Parameters:**
- `robot` - Robot to visualize
- `fps` - Frames per second (optional, for display)

**What it draws:**
1. Background and grid
2. All obstacles
3. Robot path (if enabled)
4. Sensor rays
5. Robot body and wheels

**Example:**
```cpp
viz.render(robot);  // Draw everything
```

---

#### clear()

```cpp
void clear();
```

Clear screen to background color.

---

#### present()

```cpp
void present();
```

Display rendered frame (swap buffers).

**Note:** `render()` calls this automatically.

---

### Path Visualization

#### enableTrace()

```cpp
void enableTrace(bool enable);
```

Enable/disable path tracing.

**Parameters:**
- `enable` - `true` to trace path, `false` to disable

**Example:**
```cpp
viz.enableTrace(true);   // Show robot path
viz.enableTrace(false);  // Hide path
```

---

#### clearPath()

```cpp
void clearPath();
```

Clear traced path history.

**Example:**
```cpp
viz.clearPath();  // Erase path, start fresh
```

---

### Coordinate Conversion

#### worldToScreenX()

```cpp
int worldToScreenX(double x) const;
```

Convert world X coordinate to screen pixel X.

---

#### worldToScreenY()

```cpp
int worldToScreenY(double y) const;
```

Convert world Y coordinate to screen pixel Y.

---

#### screenToWorldX()

```cpp
double screenToWorldX(int x) const;
```

Convert screen pixel X to world X coordinate.

---

#### screenToWorldY()

```cpp
double screenToWorldY(int y) const;
```

Convert screen pixel Y to world Y coordinate.

---

## Geometry Types

### Point2D

Simple 2D point structure.

```cpp
struct Point2D {
    double x, y;

    Point2D(double x_ = 0, double y_ = 0);

    double distanceTo(const Point2D& other) const;

    Point2D operator+(const Point2D& other) const;
    Point2D operator-(const Point2D& other) const;
    Point2D operator*(double scalar) const;
};
```

**Example:**
```cpp
Point2D p1(1.0, 2.0);
Point2D p2(4.0, 6.0);

double dist = p1.distanceTo(p2);  // Distance between points
Point2D sum = p1 + p2;             // Vector addition
Point2D scaled = p1 * 2.0;         // Scale vector
```

---

### Rectangle

Axis-aligned rectangular obstacle.

```cpp
struct Rectangle {
    double x, y;        // Center position (meters)
    double width, height; // Dimensions (meters)

    Rectangle(double x_ = 0, double y_ = 0, double w = 1, double h = 1);

    bool contains(double px, double py) const;
    bool intersectsCircle(double cx, double cy, double radius) const;
};
```

**Example:**
```cpp
Rectangle wall(2.0, 1.5, 0.1, 2.0);  // Vertical wall
// Center at (2.0, 1.5), size 10cm x 2m

bool hit = wall.intersectsCircle(2.05, 1.5, 0.1);  // Check collision
```

---

## Helper Functions

### SDL Timing

```cpp
uint32_t SDL_GetTicks();  // Milliseconds since SDL init
void SDL_Delay(uint32_t ms);  // Sleep for ms milliseconds
```

**Example:**
```cpp
double startTime = SDL_GetTicks() / 1000.0;  // Convert to seconds

// ... simulation ...

double elapsed = SDL_GetTicks() / 1000.0 - startTime;

SDL_Delay(20);  // Wait 20ms (for 50 Hz loop)
```

---

## Complete Example

Putting it all together:

```cpp
#include "../include/RobotLib.h"
#include "../include/units_simulation.h"
#include "../include/units_visualization.h"

using namespace robotlib::simulation;
using namespace robotlib::visualization;

int main() {
    // Create robot
    DifferentialDriveSimulator robot(
        0.15,   // 15cm wheelbase
        0.10,   // 10cm radius
        1000,   // 1000 counts/meter encoders
        4.0,    // 4m world width
        3.0     // 3m world height
    );

    // Set initial position
    robot.setPosition(0.5, 1.5, 0);

    // Add obstacles
    robot.addObstacle(Rectangle(2.5, 1.5, 0.5, 0.5));
    robot.addObstacle(Rectangle(3.5, 2.0, 0.3, 0.8));

    // Create visualizer
    RobotVisualizer viz(800, 600, 150, "My Simulation");

    // Simulation parameters
    double dt = 0.02;  // 20ms timestep

    // Main loop
    while (viz.isRunning()) {
        // Handle events
        viz.handleEvents();

        // Read sensors
        double distance = robot.getUltrasonicDistance(0);

        // Control logic
        if (distance > 0.3) {
            robot.setMotorPWM(0.5, 0.5);  // Forward
        } else {
            robot.setMotorPWM(-0.3, 0.3);  // Turn left
        }

        // Update physics
        robot.update(dt);

        // Render
        viz.render(robot);

        // Timing
        SDL_Delay((uint32_t)(dt * 1000));
    }

    return 0;
}
```

---

## Common Patterns

### Pattern: Odometry Tracking

```cpp
long prevLeft = robot.getLeftEncoder();
long prevRight = robot.getRightEncoder();

// ... time passes ...

long deltaLeft = robot.getLeftEncoder() - prevLeft;
long deltaRight = robot.getRightEncoder() - prevRight;

double leftDist = deltaLeft / 1000.0;  // counts to meters
double rightDist = deltaRight / 1000.0;

double avgDist = (leftDist + rightDist) / 2.0;
double deltaTheta = (rightDist - leftDist) / robot.getWheelbase();
```

---

### Pattern: PID Control

```cpp
PIDController pid(1.0, 0.1, 0.05);

// In loop:
double error = setpoint - measurement;
double output = pid.calculate(setpoint, measurement, dt);
robot.setMotorPWM(baseSpeed + output, baseSpeed - output);
```

---

### Pattern: State Machine

```cpp
enum State { SEARCH, APPROACH, GRAB, RETURN };
State currentState = SEARCH;

switch (currentState) {
    case SEARCH:
        // Search behavior
        if (targetFound) currentState = APPROACH;
        break;
    case APPROACH:
        // Approach behavior
        if (atTarget) currentState = GRAB;
        break;
    // ...
}
```

---

## Performance Tips

1. **Timestep:** Use 0.01 to 0.05 seconds (20-100 Hz)
2. **Scale:** Use 100-200 pixels/meter for good visualization
3. **Obstacles:** Keep under 100 for smooth performance
4. **Path length:** Limit to 1000 points max

---

## Troubleshooting

### Robot doesn't move
- Check `update(dt)` is being called
- Verify PWM values are non-zero
- Check for collisions stopping robot

### Sensors return 0
- Verify obstacles exist
- Check sensor angles (radians!)
- Ensure robot not outside world bounds

### Window doesn't appear
- Check SDL2 is installed and linked
- Verify window dimensions are reasonable
- Check for SDL initialization errors

---

## See Also

- [Tutorials](TUTORIALS.md) - Step-by-step guides
- [Advanced Features](ADVANCED_FEATURES.md) - Custom robots, sensors
- [Best Practices](BEST_PRACTICES.md) - Tips and patterns
- [Installation](INSTALL.md) - SDL2 setup

---

**Questions?** [Open an issue](https://github.com/konnorreynolds/RobotLib/issues)

**Last Updated:** November 7, 2025
**Version:** 2.2.0
