# Advanced Simulation Features

**RobotLib Simulation System - Advanced Guide**

This guide covers advanced features, customization, and extending the simulation system for complex robotics applications.

---

## Table of Contents

1. [Advanced Robot Models](#advanced-robot-models)
2. [Custom Sensor Implementation](#custom-sensor-implementation)
3. [World Building & Environments](#world-building--environments)
4. [Multi-Robot Simulation](#multi-robot-simulation)
5. [Performance Optimization](#performance-optimization)
6. [Data Recording & Playback](#data-recording--playback)
7. [Integration with External Tools](#integration-with-external-tools)
8. [Physics Customization](#physics-customization)

---

## Advanced Robot Models

### Custom Robot Dynamics

While the built-in `DifferentialDriveSimulator` is great for wheeled robots, you may need custom dynamics:

```cpp
class CustomRobotSimulator {
private:
    // State
    double x_, y_, theta_;
    double vx_, vy_, omega_;

    // Parameters
    double mass_;
    double inertia_;
    double dragCoeff_;

public:
    void applyForce(double fx, double fy, double torque) {
        // F = ma
        double ax = fx / mass_;
        double ay = fy / mass_;
        double alpha = torque / inertia_;

        // Apply drag
        ax -= dragCoeff_ * vx_;
        ay -= dragCoeff_ * vy_;

        // Update velocities
        vx_ += ax * dt;
        vy_ += ay * dt;
        omega_ += alpha * dt;
    }

    void update(double dt) {
        // Integrate position
        x_ += vx_ * dt;
        y_ += vy_ * dt;
        theta_ += omega_ * dt;

        // Normalize angle
        theta_ = std::fmod(theta_ + M_PI, 2 * M_PI) - M_PI;
    }
};
```

### Swerve Drive Model

For FRC-style swerve drive robots:

```cpp
class SwerveDriveSimulator {
private:
    struct SwerveModule {
        double x, y;           // Position relative to robot center
        double wheelAngle;     // Current wheel direction
        double wheelSpeed;     // Current wheel speed
        double driveVoltage;   // Motor voltage
        double steerVoltage;   // Steering motor voltage

        // Update module state
        void update(double dt) {
            // Simple first-order model
            double targetSpeed = driveVoltage * 2.0;  // Max 2 m/s
            double targetAngle = steerVoltage * M_PI; // ±180°

            // Rate limiting
            wheelSpeed += clamp(targetSpeed - wheelSpeed, -5*dt, 5*dt);

            double angleDiff = targetAngle - wheelAngle;
            // Normalize to [-PI, PI]
            while (angleDiff > M_PI) angleDiff -= 2*M_PI;
            while (angleDiff < -M_PI) angleDiff += 2*M_PI;
            wheelAngle += clamp(angleDiff, -2*dt, 2*dt);
        }
    };

    SwerveModule modules_[4];  // FL, FR, BL, BR
    double x_, y_, theta_;

public:
    SwerveDriveSimulator(double width, double length) {
        // Initialize module positions
        modules_[0] = {length/2, width/2};   // Front left
        modules_[1] = {length/2, -width/2};  // Front right
        modules_[2] = {-length/2, width/2};  // Back left
        modules_[3] = {-length/2, -width/2}; // Back right
    }

    void setModuleState(int module, double speed, double angle) {
        modules_[module].driveVoltage = speed;
        modules_[module].steerVoltage = angle / M_PI;
    }

    void update(double dt) {
        // Update each module
        for (auto& mod : modules_) {
            mod.update(dt);
        }

        // Calculate robot velocity from module states
        double vx = 0, vy = 0, omega = 0;

        for (const auto& mod : modules_) {
            // Module velocity in robot frame
            double modVx = mod.wheelSpeed * cos(mod.wheelAngle);
            double modVy = mod.wheelSpeed * sin(mod.wheelAngle);

            vx += modVx / 4;
            vy += modVy / 4;

            // Contribution to rotation
            omega += (modVx * mod.y - modVy * mod.x) / 4;
        }

        // Update robot pose
        x_ += (vx * cos(theta_) - vy * sin(theta_)) * dt;
        y_ += (vx * sin(theta_) + vy * cos(theta_)) * dt;
        theta_ += omega * dt;
    }
};
```

### Mecanum/Omni Drive

For holonomic drive systems:

```cpp
class MecanumDriveSimulator {
private:
    double x_, y_, theta_;
    double wheelSpeeds_[4];  // FL, FR, BL, BR

public:
    void setVelocity(double vx, double vy, double omega) {
        // Mecanum drive kinematics
        double L = 0.25;  // Half wheelbase
        double W = 0.25;  // Half track width

        wheelSpeeds_[0] = vx - vy - (L + W) * omega;  // FL
        wheelSpeeds_[1] = vx + vy + (L + W) * omega;  // FR
        wheelSpeeds_[2] = vx + vy - (L + W) * omega;  // BL
        wheelSpeeds_[3] = vx - vy + (L + W) * omega;  // BR
    }

    void update(double dt) {
        // Inverse kinematics - wheel speeds to robot velocity
        double L = 0.25, W = 0.25;
        double k = 1.0 / (4 * (L + W));

        double vx = (wheelSpeeds_[0] + wheelSpeeds_[1] +
                     wheelSpeeds_[2] + wheelSpeeds_[3]) / 4;
        double vy = (-wheelSpeeds_[0] + wheelSpeeds_[1] +
                     wheelSpeeds_[2] - wheelSpeeds_[3]) / 4;
        double omega = (-wheelSpeeds_[0] + wheelSpeeds_[1] -
                        wheelSpeeds_[2] + wheelSpeeds_[3]) * k;

        // Update pose
        x_ += (vx * cos(theta_) - vy * sin(theta_)) * dt;
        y_ += (vx * sin(theta_) + vy * cos(theta_)) * dt;
        theta_ += omega * dt;
    }
};
```

---

## Custom Sensor Implementation

### LIDAR Simulation

Simulating a 2D scanning LIDAR:

```cpp
class LIDARSimulator {
private:
    double maxRange_;
    int numBeams_;
    double fov_;  // Field of view in radians

public:
    LIDARSimulator(double maxRange = 10.0, int beams = 360, double fov = 2*M_PI)
        : maxRange_(maxRange), numBeams_(beams), fov_(fov) {}

    std::vector<double> scan(double x, double y, double theta,
                             const std::vector<Rectangle>& obstacles) {
        std::vector<double> ranges(numBeams_);

        double angleStep = fov_ / numBeams_;
        double startAngle = theta - fov_ / 2;

        for (int i = 0; i < numBeams_; i++) {
            double angle = startAngle + i * angleStep;
            ranges[i] = castRay(x, y, angle, obstacles);
        }

        return ranges;
    }

private:
    double castRay(double x, double y, double angle,
                   const std::vector<Rectangle>& obstacles) {
        double dx = cos(angle);
        double dy = sin(angle);

        double minDist = maxRange_;

        // Check obstacles
        for (const auto& obs : obstacles) {
            double dist = rayRectIntersection(x, y, dx, dy, obs);
            if (dist >= 0 && dist < minDist) {
                minDist = dist;
            }
        }

        // Add noise
        minDist += (rand() / (double)RAND_MAX - 0.5) * 0.02;

        return minDist;
    }

    double rayRectIntersection(double rx, double ry, double rdx, double rdy,
                               const Rectangle& rect) {
        // Ray-AABB intersection (slab method)
        double txmin = (rect.x - rect.width/2 - rx) / rdx;
        double txmax = (rect.x + rect.width/2 - rx) / rdx;
        double tymin = (rect.y - rect.height/2 - ry) / rdy;
        double tymax = (rect.y + rect.height/2 - ry) / rdy;

        if (txmin > txmax) std::swap(txmin, txmax);
        if (tymin > tymax) std::swap(tymin, tymax);

        double tmin = std::max(txmin, tymin);
        double tmax = std::min(txmax, tymax);

        if (tmax < 0 || tmin > tmax) {
            return -1;  // No intersection
        }

        return tmin > 0 ? tmin : tmax;
    }
};
```

### IMU Simulation

Simulating gyroscope and accelerometer:

```cpp
class IMUSimulator {
private:
    double prevVx_, prevVy_, prevOmega_;
    double gyroBias_, accelBias_;
    double gyroNoise_, accelNoise_;

public:
    IMUSimulator() : prevVx_(0), prevVy_(0), prevOmega_(0),
                     gyroBias_(0.001), accelBias_(0.01),
                     gyroNoise_(0.001), accelNoise_(0.05) {}

    struct IMUData {
        double ax, ay, az;  // Accelerometer (m/s²)
        double gx, gy, gz;  // Gyroscope (rad/s)
    };

    IMUData read(double vx, double vy, double omega, double dt) {
        IMUData data;

        // Calculate accelerations
        data.ax = (vx - prevVx_) / dt;
        data.ay = (vy - prevVy_) / dt;
        data.az = 0;  // 2D simulation

        // Gyroscope (rotation rate)
        data.gx = 0;
        data.gy = 0;
        data.gz = omega;

        // Add bias and noise
        data.ax += accelBias_ + randn() * accelNoise_;
        data.ay += accelBias_ + randn() * accelNoise_;
        data.gz += gyroBias_ + randn() * gyroNoise_;

        // Update previous values
        prevVx_ = vx;
        prevVy_ = vy;
        prevOmega_ = omega;

        return data;
    }

private:
    double randn() {
        // Box-Muller transform for Gaussian noise
        double u1 = rand() / (double)RAND_MAX;
        double u2 = rand() / (double)RAND_MAX;
        return sqrt(-2 * log(u1)) * cos(2 * M_PI * u2);
    }
};
```

### GPS/Odometry Fusion

Simulating GPS with drift and noise:

```cpp
class GPSSimulator {
private:
    double accuracy_;      // Typical accuracy in meters
    double updateRate_;    // Hz
    double timeSinceUpdate_;
    double lastX_, lastY_;

public:
    GPSSimulator(double accuracy = 2.0, double updateRate = 1.0)
        : accuracy_(accuracy), updateRate_(updateRate),
          timeSinceUpdate_(0), lastX_(0), lastY_(0) {}

    struct GPSReading {
        double x, y;
        bool newData;
        double accuracy;
    };

    GPSReading read(double trueX, double trueY, double dt) {
        GPSReading reading;
        timeSinceUpdate_ += dt;

        if (timeSinceUpdate_ >= 1.0 / updateRate_) {
            // New GPS update available
            reading.newData = true;
            reading.x = trueX + randn() * accuracy_;
            reading.y = trueY + randn() * accuracy_;
            reading.accuracy = accuracy_;

            lastX_ = reading.x;
            lastY_ = reading.y;
            timeSinceUpdate_ = 0;
        } else {
            // Return last reading
            reading.newData = false;
            reading.x = lastX_;
            reading.y = lastY_;
            reading.accuracy = accuracy_;
        }

        return reading;
    }

private:
    double randn() {
        double u1 = rand() / (double)RAND_MAX;
        double u2 = rand() / (double)RAND_MAX;
        return sqrt(-2 * log(u1)) * cos(2 * M_PI * u2);
    }
};
```

---

## World Building & Environments

### Procedural Maze Generation

Generate random mazes for testing:

```cpp
class MazeGenerator {
public:
    static std::vector<Rectangle> generateMaze(int width, int height,
                                                double cellSize = 0.5) {
        std::vector<Rectangle> walls;
        std::vector<std::vector<bool>> visited(height,
                                               std::vector<bool>(width, false));

        // Depth-first search maze generation
        std::stack<std::pair<int, int>> stack;
        stack.push({0, 0});
        visited[0][0] = true;

        while (!stack.empty()) {
            auto [y, x] = stack.top();

            // Find unvisited neighbors
            std::vector<std::pair<int, int>> neighbors;
            if (x > 0 && !visited[y][x-1]) neighbors.push_back({y, x-1});
            if (x < width-1 && !visited[y][x+1]) neighbors.push_back({y, x+1});
            if (y > 0 && !visited[y-1][x]) neighbors.push_back({y-1, x});
            if (y < height-1 && !visited[y+1][x]) neighbors.push_back({y+1, x});

            if (neighbors.empty()) {
                stack.pop();
            } else {
                // Choose random neighbor
                int idx = rand() % neighbors.size();
                auto [ny, nx] = neighbors[idx];

                // Don't add wall between current and neighbor
                visited[ny][nx] = true;
                stack.push({ny, nx});
            }
        }

        // Create walls for non-carved paths
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (!visited[y][x]) {
                    walls.push_back(Rectangle(
                        x * cellSize + cellSize/2,
                        y * cellSize + cellSize/2,
                        cellSize, cellSize
                    ));
                }
            }
        }

        return walls;
    }
};
```

### Track/Course Builder

Create racing tracks or competition courses:

```cpp
class CourseBuilder {
public:
    struct Waypoint {
        double x, y;
        double radius;  // Tolerance
    };

    struct Course {
        std::vector<Rectangle> boundaries;
        std::vector<Rectangle> obstacles;
        std::vector<Waypoint> waypoints;
        Point2D start;
        Point2D goal;
    };

    static Course buildFRCAutoCourse() {
        Course course;

        // Field boundaries (54' x 27')
        double fieldWidth = 16.46;  // meters
        double fieldHeight = 8.23;

        // Outer walls
        course.boundaries.push_back(Rectangle(fieldWidth/2, 0, fieldWidth, 0.1));
        course.boundaries.push_back(Rectangle(fieldWidth/2, fieldHeight, fieldWidth, 0.1));
        course.boundaries.push_back(Rectangle(0, fieldHeight/2, 0.1, fieldHeight));
        course.boundaries.push_back(Rectangle(fieldWidth, fieldHeight/2, 0.1, fieldHeight));

        // Game elements (simplified)
        course.obstacles.push_back(Rectangle(4.0, 4.0, 0.6, 0.6));  // Note
        course.obstacles.push_back(Rectangle(8.0, 4.0, 0.6, 0.6));
        course.obstacles.push_back(Rectangle(12.0, 4.0, 0.6, 0.6));

        // Waypoints for auto routine
        course.waypoints.push_back({2.0, 2.0, 0.2});
        course.waypoints.push_back({4.0, 3.5, 0.2});
        course.waypoints.push_back({6.0, 2.0, 0.2});
        course.waypoints.push_back({8.0, 3.5, 0.2});

        // Start and goal
        course.start = Point2D(1.5, 2.0);
        course.goal = Point2D(14.0, 6.0);

        return course;
    }

    static Course buildLineFollowerCourse() {
        Course course;

        // Simple oval track with line
        // Boundaries are implied by line following

        // Start position
        course.start = Point2D(1.0, 1.5);

        return course;
    }
};
```

---

## Multi-Robot Simulation

### Simulating Multiple Robots

```cpp
class MultiRobotSimulation {
private:
    std::vector<std::unique_ptr<DifferentialDriveSimulator>> robots_;
    std::vector<Rectangle> obstacles_;

public:
    void addRobot(double x, double y, double theta) {
        auto robot = std::make_unique<DifferentialDriveSimulator>();
        robot->setPosition(x, y, theta);

        // Share obstacles with all robots
        for (const auto& obs : obstacles_) {
            robot->addObstacle(obs);
        }

        robots_.push_back(std::move(robot));
    }

    void addObstacle(const Rectangle& obs) {
        obstacles_.push_back(obs);

        // Add to all existing robots
        for (auto& robot : robots_) {
            robot->addObstacle(obs);
        }
    }

    void update(double dt) {
        // Update all robots
        for (auto& robot : robots_) {
            robot->update(dt);
        }

        // Check inter-robot collisions
        checkCollisions();
    }

    void checkCollisions() {
        for (size_t i = 0; i < robots_.size(); i++) {
            for (size_t j = i + 1; j < robots_.size(); j++) {
                double dx = robots_[i]->getX() - robots_[j]->getX();
                double dy = robots_[i]->getY() - robots_[j]->getY();
                double dist = sqrt(dx*dx + dy*dy);

                double minDist = robots_[i]->getRobotRadius() +
                                robots_[j]->getRobotRadius();

                if (dist < minDist) {
                    // Collision! Stop both robots
                    robots_[i]->stop();
                    robots_[j]->stop();
                }
            }
        }
    }

    DifferentialDriveSimulator& getRobot(size_t index) {
        return *robots_[index];
    }

    size_t getNumRobots() const { return robots_.size(); }
};
```

---

## Performance Optimization

### Level of Detail (LOD)

Reduce computation for distant robots:

```cpp
class LODSimulation {
private:
    enum class DetailLevel {
        HIGH,    // Full physics, all sensors
        MEDIUM,  // Simplified physics, basic sensors
        LOW      // Position update only
    };

    DetailLevel getDetailLevel(double distanceToCamera) {
        if (distanceToCamera < 5.0) return DetailLevel::HIGH;
        if (distanceToCamera < 15.0) return DetailLevel::MEDIUM;
        return DetailLevel::LOW;
    }

public:
    void update(DifferentialDriveSimulator& robot, double dt,
                double cameraX, double cameraY) {
        double dx = robot.getX() - cameraX;
        double dy = robot.getY() - cameraY;
        double dist = sqrt(dx*dx + dy*dy);

        DetailLevel lod = getDetailLevel(dist);

        switch (lod) {
            case DetailLevel::HIGH:
                // Full physics simulation
                robot.update(dt);
                break;

            case DetailLevel::MEDIUM:
                // Simplified - no sensor noise
                robot.update(dt);
                break;

            case DetailLevel::LOW:
                // Just update position, no collision check
                double v = (robot.getLeftVelocity() +
                           robot.getRightVelocity()) / 2.0;
                double omega = (robot.getRightVelocity() -
                               robot.getLeftVelocity()) /
                               robot.getWheelbase();
                // Direct position update without collision
                break;
        }
    }
};
```

### Spatial Partitioning

Use quadtree for efficient collision detection:

```cpp
class Quadtree {
private:
    struct Node {
        Rectangle bounds;
        std::vector<Rectangle*> objects;
        std::unique_ptr<Node> children[4];

        bool isLeaf() const { return children[0] == nullptr; }
    };

    std::unique_ptr<Node> root_;
    int maxObjects_ = 10;
    int maxDepth_ = 5;

public:
    void insert(Rectangle* obj) {
        insertHelper(root_.get(), obj, 0);
    }

    std::vector<Rectangle*> query(const Rectangle& area) {
        std::vector<Rectangle*> results;
        queryHelper(root_.get(), area, results);
        return results;
    }

private:
    void insertHelper(Node* node, Rectangle* obj, int depth) {
        if (!node->isLeaf()) {
            // Insert into children
            int index = getQuadrant(node->bounds, *obj);
            if (index != -1) {
                insertHelper(node->children[index].get(), obj, depth + 1);
                return;
            }
        }

        node->objects.push_back(obj);

        // Subdivide if needed
        if (node->objects.size() > maxObjects_ && depth < maxDepth_) {
            subdivide(node);
        }
    }

    void subdivide(Node* node) {
        // Create 4 child nodes
        // Implementation details...
    }

    int getQuadrant(const Rectangle& bounds, const Rectangle& obj) {
        // Determine which quadrant object belongs to
        // Return -1 if it doesn't fit completely in one quadrant
        // Implementation details...
        return -1;
    }

    void queryHelper(Node* node, const Rectangle& area,
                     std::vector<Rectangle*>& results) {
        // Recursively find all objects in area
        // Implementation details...
    }
};
```

---

## Data Recording & Playback

### Simulation Data Logger

Record simulation data for analysis:

```cpp
class SimulationLogger {
private:
    struct DataPoint {
        double timestamp;
        double x, y, theta;
        double leftVel, rightVel;
        double sensorDistance;
    };

    std::vector<DataPoint> log_;
    double startTime_;
    bool recording_;

public:
    SimulationLogger() : startTime_(0), recording_(false) {}

    void startRecording() {
        recording_ = true;
        startTime_ = SDL_GetTicks() / 1000.0;
        log_.clear();
    }

    void stopRecording() {
        recording_ = false;
    }

    void record(const DifferentialDriveSimulator& robot) {
        if (!recording_) return;

        DataPoint dp;
        dp.timestamp = SDL_GetTicks() / 1000.0 - startTime_;
        dp.x = robot.getX();
        dp.y = robot.getY();
        dp.theta = robot.getTheta();
        dp.leftVel = robot.getLeftVelocity();
        dp.rightVel = robot.getRightVelocity();
        dp.sensorDistance = robot.getUltrasonicDistance(0);

        log_.push_back(dp);
    }

    void saveToCSV(const std::string& filename) {
        std::ofstream file(filename);
        file << "time,x,y,theta,left_vel,right_vel,sensor_dist\n";

        for (const auto& dp : log_) {
            file << dp.timestamp << ","
                 << dp.x << "," << dp.y << "," << dp.theta << ","
                 << dp.leftVel << "," << dp.rightVel << ","
                 << dp.sensorDistance << "\n";
        }
    }

    const std::vector<DataPoint>& getData() const { return log_; }
};
```

### Playback System

Replay recorded simulations:

```cpp
class SimulationPlayback {
private:
    std::vector<DataPoint> data_;
    size_t currentFrame_;
    bool playing_;
    double playbackSpeed_;

public:
    void load(const std::string& filename) {
        // Load CSV data
        // Implementation details...
    }

    void play(double speed = 1.0) {
        playing_ = true;
        playbackSpeed_ = speed;
        currentFrame_ = 0;
    }

    bool update(DifferentialDriveSimulator& robot, double dt) {
        if (!playing_ || currentFrame_ >= data_.size()) {
            return false;
        }

        // Interpolate between frames if needed
        const auto& frame = data_[currentFrame_];
        robot.setPosition(frame.x, frame.y, frame.theta);

        currentFrame_++;
        return true;
    }
};
```

---

## Integration with External Tools

### ROS2 Bridge

Connect simulation to ROS2:

```cpp
#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class ROS2SimulationBridge : public rclcpp::Node {
private:
    DifferentialDriveSimulator& robot_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;

public:
    ROS2SimulationBridge(DifferentialDriveSimulator& robot)
        : Node("sim_bridge"), robot_(robot) {

        cmdVelSub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&ROS2SimulationBridge::cmdVelCallback, this,
                     std::placeholders::_1));

        odomPub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Convert Twist to wheel velocities
        double v = msg->linear.x;
        double omega = msg->angular.z;
        double wheelbase = robot_.getWheelbase();

        double leftVel = v - omega * wheelbase / 2;
        double rightVel = v + omega * wheelbase / 2;

        robot_.setWheelVelocities(leftVel, rightVel);
    }

    void publishOdometry() {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = now();
        msg.header.frame_id = "odom";

        msg.pose.pose.position.x = robot_.getX();
        msg.pose.pose.position.y = robot_.getY();
        // Convert theta to quaternion...

        odomPub_->publish(msg);
    }
};
#endif
```

### MATLAB/Python Interface

Export simulation data for analysis:

```cpp
class MATLABExporter {
public:
    static void exportTrajectory(const std::vector<Point2D>& path,
                                  const std::string& filename) {
        std::ofstream file(filename);

        file << "% MATLAB trajectory data\n";
        file << "% Generated by RobotLib Simulation\n\n";
        file << "x = [";
        for (const auto& p : path) {
            file << p.x << " ";
        }
        file << "];\n\n";

        file << "y = [";
        for (const auto& p : path) {
            file << p.y << " ";
        }
        file << "];\n\n";

        file << "plot(x, y);\n";
        file << "xlabel('X (m)');\n";
        file << "ylabel('Y (m)');\n";
        file << "title('Robot Trajectory');\n";
        file << "grid on;\n";
    }
};
```

---

## Physics Customization

### Advanced Collision Response

Implement realistic collision physics:

```cpp
class PhysicsEngine {
public:
    static void resolveCollision(DifferentialDriveSimulator& robot,
                                 const Rectangle& obstacle) {
        // Get collision normal
        Vec2D normal = getCollisionNormal(robot, obstacle);

        // Calculate relative velocity
        double vx = robot.getLeftVelocity();  // Simplified
        Vec2D velocity(vx, 0);

        // Calculate impulse
        double restitution = 0.3;  // Coefficient of restitution
        double impulse = -(1 + restitution) * velocity.dot(normal);

        // Apply impulse
        Vec2D impulseVector = normal * impulse;

        // Update velocities
        double newLeftVel = robot.getLeftVelocity() + impulseVector.x;
        double newRightVel = robot.getRightVelocity() + impulseVector.x;

        robot.setWheelVelocities(newLeftVel * 0.5, newRightVel * 0.5);
    }

private:
    static Vec2D getCollisionNormal(const DifferentialDriveSimulator& robot,
                                    const Rectangle& obstacle) {
        // Calculate normal vector from obstacle to robot
        double dx = robot.getX() - obstacle.x;
        double dy = robot.getY() - obstacle.y;
        double len = sqrt(dx*dx + dy*dy);

        if (len > 0) {
            return Vec2D(dx / len, dy / len);
        }
        return Vec2D(1, 0);
    }
};
```

### Friction and Slipping

Model wheel slipping on different surfaces:

```cpp
class SurfaceModel {
public:
    enum class SurfaceType {
        CONCRETE,  // High friction
        CARPET,    // Medium friction
        ICE        // Low friction
    };

    static double getFrictionCoefficient(SurfaceType surface) {
        switch (surface) {
            case SurfaceType::CONCRETE: return 0.9;
            case SurfaceType::CARPET: return 0.6;
            case SurfaceType::ICE: return 0.1;
        }
        return 0.7;
    }

    static void applyFriction(DifferentialDriveSimulator& robot,
                              SurfaceType surface, double dt) {
        double mu = getFrictionCoefficient(surface);

        // Reduce velocities based on friction
        double leftVel = robot.getLeftVelocity();
        double rightVel = robot.getRightVelocity();

        // Simple friction model: v_new = v_old * (1 - mu * dt)
        double factor = 1.0 - mu * dt;

        robot.setWheelVelocities(leftVel * factor, rightVel * factor);
    }
};
```

---

## Next Steps

- Experiment with advanced robot models
- Build custom sensors for your application
- Create complex environments and courses
- Optimize performance for large simulations
- Integrate with external tools (ROS2, MATLAB)
- Implement realistic physics

For more information, see:
- [Simulation Tutorials](TUTORIALS.md)
- [API Reference](API_REFERENCE.md)
- [Best Practices](BEST_PRACTICES.md)

---

**Questions or suggestions?** [Open an issue](https://github.com/konnorreynolds/RobotLib/issues)

Happy simulating! 🤖🔬
