# RobotLib Quick Start Guide

**Get up and running with RobotLib in 5 minutes!**

---

## 🚀 5-Minute Setup

### Step 1: Installation (2 minutes)

#### Option A: Arduino IDE
1. Download this repository as ZIP
2. Extract to `Documents/Arduino/libraries/RobotLib/`
3. Restart Arduino IDE
4. Done! ✅

#### Option B: PlatformIO
```bash
cd your-project
# Add to platformio.ini:
lib_deps =
    https://github.com/konnorreynolds/RobotLib.git
```

#### Option C: Desktop (Linux/macOS)
```bash
git clone https://github.com/konnorreynolds/RobotLib.git
cd RobotLib/examples
make
./hello
```

#### Option D: Desktop (Windows)
```batch
git clone https://github.com/konnorreynolds/RobotLib.git
cd RobotLib\examples
build.bat
.\build\bin\Release\fluent_08_hello_units.exe
```

**Requirements:**
- Visual Studio 2017+ with C++ tools, OR
- MinGW-w64 (g++)
- CMake (optional, auto-detected by build.bat)

---

### Step 2: Your First Program (2 minutes)

Create `first_program.cpp`:

```cpp
#include <RobotLib.h>

using namespace units;
using namespace robotlib::output;

int main() {
    // Create type-safe distances
    auto distance = m(10.0);  // 10 meters
    auto time = s(2.0);        // 2 seconds

    // Automatic unit calculation!
    auto speed = distance / time;

    // Print results
    println("Distance: ", distance.toMeters(), " m");
    println("Time: ", time.toSeconds(), " s");
    println("Speed: ", speed.toMetersPerSecond(), " m/s");
    println("Speed: ", speed.toKilometersPerHour(), " km/h");

    return 0;
}
```

---

### Step 3: Compile & Run (1 minute)

#### Desktop (Linux/macOS):
```bash
g++ -std=c++11 -I./include first_program.cpp -o first
./first
```

#### Desktop (Windows - Command Prompt):
```batch
REM With MSVC:
cl /std:c++11 /EHsc /I..\include first_program.cpp
first.exe

REM With MinGW:
g++ -std=c++11 -I../include first_program.cpp -o first.exe
first.exe
```

#### Desktop (Windows - Easy Way):
Copy `first_program.cpp` to `examples/06_fluent_api/` and run:
```batch
cd examples
build.bat
.\build\bin\Release\fluent_01_first_program.exe
```

#### Arduino:
1. Open Arduino IDE
2. File → Examples → RobotLib → 06_fluent_api → 08_hello_units
3. Upload to board

**Expected Output:**
```
Distance: 10 m
Time: 2 s
Speed: 5 m/s
Speed: 18 km/h
```

✅ **Congratulations!** You're using RobotLib! 🎉

---

## 📚 15-Minute Tutorial

### Project 1: Motor Control (5 minutes)

Control a DC motor with type-safe units:

```cpp
#include <RobotLib.h>

using namespace units;

// Arduino pins
const int MOTOR_PWM = 9;
const int MOTOR_DIR = 8;

void setMotorSpeed(Velocity speed) {
    // Convert to PWM (0-255)
    int pwm = (int)(speed.toMetersPerSecond() / 0.5 * 255);
    pwm = constrain(pwm, 0, 255);

    digitalWrite(MOTOR_DIR, speed.toMetersPerSecond() > 0);
    analogWrite(MOTOR_PWM, pwm);
}

void setup() {
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_DIR, OUTPUT);
}

void loop() {
    // Accelerate to 0.3 m/s
    setMotorSpeed(mps(0.3));
    delay(2000);

    // Stop
    setMotorSpeed(mps(0));
    delay(1000);

    // Reverse at 0.2 m/s
    setMotorSpeed(mps(-0.2));
    delay(2000);

    // Stop
    setMotorSpeed(mps(0));
    delay(1000);
}
```

**What you learned:**
- ✅ Creating velocity units
- ✅ Converting to PWM values
- ✅ Motor control patterns

---

### Project 2: PID Line Follower (7 minutes)

Use PID control for smooth line following:

```cpp
#include <RobotLib.h>

using namespace units;
using namespace robotics;

// Hardware pins
const int LINE_SENSORS[] = {A0, A1, A2, A3, A4};  // 5 sensors
const int MOTOR_LEFT_PWM = 9;
const int MOTOR_RIGHT_PWM = 10;

// PID controller (tune these values!)
PIDController pid(
    1.5,  // kP - Proportional gain
    0.3,  // kI - Integral gain
    0.1   // kD - Derivative gain
);

const double BASE_SPEED = 0.3;  // 30% power

double readLinePosition() {
    // Read 5 sensors (true = line detected)
    bool sensors[5];
    for (int i = 0; i < 5; i++) {
        sensors[i] = (analogRead(LINE_SENSORS[i]) > 500);
    }

    // Calculate weighted position (-2 to +2)
    // -2 = far left, 0 = center, +2 = far right
    double position = 0;
    int activeCount = 0;

    for (int i = 0; i < 5; i++) {
        if (sensors[i]) {
            position += (i - 2);  // -2, -1, 0, 1, 2
            activeCount++;
        }
    }

    if (activeCount > 0) {
        position /= activeCount;
    }

    return position;
}

void setMotors(double leftPWM, double rightPWM) {
    // Convert -1..1 to 0..255
    int left = (int)((leftPWM + 1) * 127.5);
    int right = (int)((rightPWM + 1) * 127.5);

    analogWrite(MOTOR_LEFT_PWM, constrain(left, 0, 255));
    analogWrite(MOTOR_RIGHT_PWM, constrain(right, 0, 255));
}

void setup() {
    Serial.begin(9600);

    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);

    for (int i = 0; i < 5; i++) {
        pinMode(LINE_SENSORS[i], INPUT);
    }
}

void loop() {
    // Read line position
    double position = readLinePosition();

    // PID control (setpoint = 0, want line centered)
    double correction = pid.calculate(
        0.0,       // Setpoint: center
        position,  // Current position
        0.02       // dt: 20ms loop time
    );

    // Apply to motors
    double leftPWM = BASE_SPEED - correction;
    double rightPWM = BASE_SPEED + correction;

    setMotors(leftPWM, rightPWM);

    // Debug output
    Serial.print("Pos: ");
    Serial.print(position);
    Serial.print(" Correction: ");
    Serial.println(correction);

    delay(20);  // 50 Hz control loop
}
```

**What you learned:**
- ✅ PID controller usage
- ✅ Sensor array processing
- ✅ Real-time control loop
- ✅ Motor differential control

---

### Project 3: Differential Drive Robot (3 minutes)

Complete robot with odometry:

```cpp
#include <RobotLib.h>

using namespace units;
using namespace robotics;

// Robot parameters
const double WHEELBASE = 0.15;     // 15cm between wheels
const double WHEEL_DIAMETER = 0.06; // 6cm wheels
const int ENCODER_CPR = 20;         // Counts per revolution

// Hardware
const int ENCODER_LEFT = 2;
const int ENCODER_RIGHT = 3;

volatile long leftCount = 0;
volatile long rightCount = 0;

void leftEncoderISR() { leftCount++; }
void rightEncoderISR() { rightCount++; }

class Odometry {
private:
    double x, y, theta;
    long prevLeft, prevRight;
    double countsPerMeter;

public:
    Odometry() : x(0), y(0), theta(0), prevLeft(0), prevRight(0) {
        countsPerMeter = ENCODER_CPR / (M_PI * WHEEL_DIAMETER);
    }

    void update(long leftCounts, long rightCounts) {
        // Calculate distances traveled
        double leftDist = (leftCounts - prevLeft) / countsPerMeter;
        double rightDist = (rightCounts - prevRight) / countsPerMeter;

        // Update pose
        double distance = (leftDist + rightDist) / 2.0;
        double dtheta = (rightDist - leftDist) / WHEELBASE;

        x += distance * cos(theta);
        y += distance * sin(theta);
        theta += dtheta;

        // Normalize angle
        while (theta > M_PI) theta -= 2 * M_PI;
        while (theta < -M_PI) theta += 2 * M_PI;

        prevLeft = leftCounts;
        prevRight = rightCounts;
    }

    double getX() const { return x; }
    double getY() const { return y; }
    double getTheta() const { return theta; }
};

Odometry odom;

void setup() {
    Serial.begin(115200);

    pinMode(ENCODER_LEFT, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT),
                    leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT),
                    rightEncoderISR, RISING);
}

void loop() {
    // Update odometry
    odom.update(leftCount, rightCount);

    // Print position
    Serial.print("X: ");
    Serial.print(odom.getX(), 3);
    Serial.print(" Y: ");
    Serial.print(odom.getY(), 3);
    Serial.print(" θ: ");
    Serial.print(odom.getTheta() * 180 / M_PI, 1);
    Serial.println("°");

    delay(100);
}
```

**What you learned:**
- ✅ Encoder reading with interrupts
- ✅ Odometry calculations
- ✅ Pose estimation
- ✅ Differential drive kinematics

---

## 🎯 What's Next?

### Beginner Path
1. ✅ Completed quickstart
2. → Try `examples/01_basics/` - Simple sensors and motors
3. → Try `examples/02_with_feedback/` - PID control
4. → Try `examples/06_fluent_api/` - Easy robot programming

### Intermediate Path
1. → `examples/03_full_systems/` - Complete robots
2. → `examples/04_algorithms/` - Path planning, filters
3. → `simulation/TUTORIALS.md` - Test in simulation first!

### Advanced Path
1. → `examples/05_advanced/` - EKF, MPC, SLAM
2. → `simulation/ADVANCED_FEATURES.md` - Custom robots
3. → Build your own competition robot!

---

## 📖 Documentation

- **README.md** - Overview and features
- **examples/README.md** - All 23 examples explained
- **simulation/README.md** - Robot simulation guide
- **API docs** - Comments in header files
- **DISCLAIMER.md** - ⚠️ Important: AI-generated, review before use

---

## 💡 Tips for Success

### ✅ DO:
- Start with simple examples
- Test in simulation before hardware
- Use type-safe units everywhere
- Read inline documentation
- Tune PID parameters incrementally

### ❌ DON'T:
- Mix raw doubles with units
- Skip testing edge cases
- Deploy untested code to hardware
- Use in safety-critical systems without review

---

## 🆘 Troubleshooting

### "RobotLib.h: No such file or directory"
**Solution:** Check include path
```bash
g++ -I./include ...  # Add include path
```

### Arduino: "Error compiling for board"
**Solution:**
1. Verify library is in `Arduino/libraries/RobotLib/`
2. Restart Arduino IDE
3. Check board selection

### Windows: "No C++ compiler found!"
**Solution:**
1. Install Visual Studio 2017+ with "Desktop development with C++"
   - OR install MinGW-w64: https://www.mingw-w64.org/
2. Open "Developer Command Prompt for VS" (for MSVC)
   - OR add MinGW to PATH: `set PATH=C:\mingw-w64\bin;%PATH%`
3. Run `build.bat` again

### Windows: CMake not found
**Solution:**
1. Install CMake: https://cmake.org/download/
2. Add to PATH during installation
3. Restart Command Prompt
4. Run `build.bat` again

### Motors don't respond
**Check:**
- Pin numbers correct?
- Power supply connected?
- PWM pins configured as OUTPUT?
- Serial monitor for debug output

### PID oscillates wildly
**Solution:** Reduce gains
```cpp
PIDController pid(0.5, 0.0, 0.0);  // Start with P only
// Tune up gradually
```

---

## 🤝 Getting Help

- **Examples not working?** Check `examples/README.md`
- **Simulation issues?** See `simulation/INSTALL.md`
- **Found a bug?** [Report it](https://github.com/konnorreynolds/RobotLib/issues)
- **Have a question?** [Ask on discussions](https://github.com/konnorreynolds/RobotLib/discussions)

---

## ⭐ Show Your Support

If RobotLib helped you:
- ⭐ Star the repository
- 📢 Share with others
- 🐛 Report bugs
- 💡 Suggest features
- 🤝 Contribute code

---

**Happy robot building!** 🤖✨

---

**Next:** Read [README.md](README.md) for complete feature list
**Then:** Try [examples/README.md](examples/README.md) for all 23 examples
**Finally:** Build something awesome! 🚀
