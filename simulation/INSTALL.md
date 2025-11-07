# SDL2 Installation Guide

**Complete installation instructions for RobotLib simulation system**

This guide covers SDL2 installation on all major platforms.

---

## Table of Contents

1. [Ubuntu/Debian Linux](#ubuntudebian-linux)
2. [Fedora/RHEL/CentOS](#fedorарhelcentos)
3. [Arch Linux](#arch-linux)
4. [macOS](#macos)
5. [Windows](#windows)
6. [Verification](#verification)
7. [Troubleshooting](#troubleshooting)

---

## Ubuntu/Debian Linux

### Quick Install (Recommended)

```bash
sudo apt-get update
sudo apt-get install libsdl2-dev
```

### Manual Verification

```bash
# Check if installed
dpkg -l | grep libsdl2-dev

# Verify version
sdl2-config --version
```

### Expected Output

```
2.0.20  # Or similar version >= 2.0.x
```

### Test Compilation

```bash
cd ~/RobotLib/simulation
g++ -std=c++11 -I../include 01_basic_simulation.cpp -lSDL2 -o test
./test
```

### If Installation Fails

```bash
# Add universe repository
sudo add-apt-repository universe
sudo apt-get update

# Try again
sudo apt-get install libsdl2-dev
```

---

## Fedora/RHEL/CentOS

### Fedora

```bash
sudo dnf install SDL2-devel
```

### RHEL/CentOS 8+

```bash
sudo dnf install SDL2-devel
```

### RHEL/CentOS 7

```bash
# Enable EPEL repository first
sudo yum install epel-release

# Install SDL2
sudo yum install SDL2-devel
```

### Verification

```bash
sdl2-config --version
```

---

## Arch Linux

### Install

```bash
sudo pacman -S sdl2
```

### Verification

```bash
pacman -Q sdl2
```

---

## macOS

### Option 1: Homebrew (Recommended)

```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install SDL2
brew install sdl2
```

### Option 2: MacPorts

```bash
sudo port install libsdl2
```

### Verification

```bash
# Check installation
brew list sdl2

# Check version
sdl2-config --version
```

### Compilation on macOS

```bash
# Standard compilation
g++ -std=c++11 -I../include \
    -I/usr/local/include \
    -L/usr/local/lib \
    01_basic_simulation.cpp \
    -lSDL2 \
    -o test

# Or use pkg-config (recommended)
g++ -std=c++11 -I../include \
    $(pkg-config --cflags sdl2) \
    01_basic_simulation.cpp \
    $(pkg-config --libs sdl2) \
    -o test
```

### macOS Troubleshooting

If you get "SDL.h not found":

```bash
# Find SDL2 location
brew --prefix sdl2

# Add to compilation:
g++ -std=c++11 -I../include \
    -I$(brew --prefix sdl2)/include \
    -L$(brew --prefix sdl2)/lib \
    01_basic_simulation.cpp \
    -lSDL2 \
    -o test
```

---

## Windows

### Option 1: MSYS2 (Recommended for MinGW/GCC)

**Step 1: Install MSYS2**

Download and install from: https://www.msys2.org/

**Step 2: Open MSYS2 MinGW 64-bit terminal**

**Step 3: Install SDL2**

```bash
pacman -S mingw-w64-x86_64-SDL2
```

**Step 4: Verify**

```bash
pacman -Q mingw-w64-x86_64-SDL2
```

**Step 5: Compile**

```bash
g++ -std=c++11 -I../include \
    01_basic_simulation.cpp \
    -lmingw32 -lSDL2main -lSDL2 \
    -o test.exe
```

---

### Option 2: Visual Studio (MSVC)

**Step 1: Download SDL2 Development Libraries**

Go to: https://www.libsdl.org/download-2.0.php

Download: **SDL2-devel-2.X.X-VC.zip** (Visual C++)

**Step 2: Extract to C:\SDL2**

Extract the ZIP file to `C:\SDL2` (or any location)

**Step 3: Configure Visual Studio Project**

*In Visual Studio:*

1. **Right-click project → Properties**

2. **C/C++ → General → Additional Include Directories:**
   ```
   C:\SDL2\include
   ```

3. **Linker → General → Additional Library Directories:**
   ```
   C:\SDL2\lib\x64  # For 64-bit
   C:\SDL2\lib\x86  # For 32-bit
   ```

4. **Linker → Input → Additional Dependencies:**
   ```
   SDL2.lib
   SDL2main.lib
   ```

5. **Copy SDL2.dll**

   Copy `SDL2.dll` from `C:\SDL2\lib\x64` to your project's output directory (where .exe is generated)

**Step 4: Build and Run**

Build project in Visual Studio (Ctrl+B) and run.

---

### Option 3: vcpkg (Cross-platform package manager)

**Step 1: Install vcpkg**

```bash
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat  # Windows
```

**Step 2: Install SDL2**

```bash
.\vcpkg install sdl2
```

**Step 3: Integrate with Visual Studio**

```bash
.\vcpkg integrate install
```

Now SDL2 is automatically available in Visual Studio projects!

---

### Option 4: CMake with vcpkg

**CMakeLists.txt:**

```cmake
cmake_minimum_required(VERSION 3.10)
project(RobotSim)

set(CMAKE_CXX_STANDARD 11)

find_package(SDL2 CONFIG REQUIRED)

add_executable(sim 01_basic_simulation.cpp)

target_include_directories(sim PRIVATE ../include)
target_link_libraries(sim PRIVATE SDL2::SDL2 SDL2::SDL2main)
```

**Build:**

```bash
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=[vcpkg root]/scripts/buildsystems/vcpkg.cmake
cmake --build .
```

---

## Verification

### Test Compilation Script

Create `test_sdl2.cpp`:

```cpp
#include <SDL2/SDL.h>
#include <iostream>

int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << std::endl;
        return 1;
    }

    std::cout << "✓ SDL2 initialized successfully!" << std::endl;

    SDL_version compiled;
    SDL_version linked;

    SDL_VERSION(&compiled);
    SDL_GetVersion(&linked);

    std::cout << "✓ Compiled against SDL version: "
              << (int)compiled.major << "."
              << (int)compiled.minor << "."
              << (int)compiled.patch << std::endl;

    std::cout << "✓ Linked against SDL version: "
              << (int)linked.major << "."
              << (int)linked.minor << "."
              << (int)linked.patch << std::endl;

    SDL_Quit();

    std::cout << "✓ All tests passed!" << std::endl;
    return 0;
}
```

### Compile and Run

**Linux/macOS:**
```bash
g++ -std=c++11 test_sdl2.cpp -lSDL2 -o test_sdl2
./test_sdl2
```

**Windows (MinGW):**
```bash
g++ -std=c++11 test_sdl2.cpp -lmingw32 -lSDL2main -lSDL2 -o test_sdl2.exe
test_sdl2.exe
```

### Expected Output

```
✓ SDL2 initialized successfully!
✓ Compiled against SDL version: 2.0.20
✓ Linked against SDL version: 2.0.20
✓ All tests passed!
```

---

## Troubleshooting

### Error: "SDL.h: No such file or directory"

**Linux:**
```bash
# Find where SDL2 is installed
find /usr -name "SDL.h" 2>/dev/null

# Likely: /usr/include/SDL2/SDL.h

# Include in compilation:
g++ -I/usr/include/SDL2 ...
```

**macOS:**
```bash
# Use pkg-config
g++ $(pkg-config --cflags sdl2) ...
```

**Windows:**
- Verify SDL2 include path in IDE settings
- Make sure path uses forward slashes: `C:/SDL2/include`

---

### Error: "cannot find -lSDL2"

**Linux:**
```bash
# Check if library exists
find /usr -name "libSDL2*" 2>/dev/null

# Likely: /usr/lib/x86_64-linux-gnu/libSDL2.so

# Add library path:
g++ ... -L/usr/lib/x86_64-linux-gnu -lSDL2
```

**macOS:**
```bash
# Check library location
ls /usr/local/lib/libSDL2*

# Add to link command:
g++ ... -L/usr/local/lib -lSDL2
```

---

### Error: "SDL2.dll not found" (Windows)

**Solution:**

Copy `SDL2.dll` to the same directory as your `.exe` file.

**Find SDL2.dll:**
- MSYS2: `/mingw64/bin/SDL2.dll`
- Manual install: `C:\SDL2\lib\x64\SDL2.dll`

**Copy command:**
```bash
cp /mingw64/bin/SDL2.dll .  # MSYS2
```

---

### Error: "X11 not found" (Linux)

Some systems need X11 development files:

```bash
# Ubuntu/Debian
sudo apt-get install libx11-dev

# Fedora
sudo dnf install libX11-devel

# Arch
sudo pacman -S libx11
```

---

### Error: Runtime crash or "Segmentation fault"

**Check:**
1. SDL_Init was called before using SDL
2. SDL_Quit is called before exit
3. Window creation succeeded
4. All SDL functions return valid pointers

**Debug build:**
```bash
g++ -g -std=c++11 ... -lSDL2  # Add -g for debug symbols
gdb ./your_program
```

---

### Performance Issues

**Enable hardware acceleration:**

```cpp
SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
```

**Check FPS:**

```cpp
Uint32 frameStart = SDL_GetTicks();
// ... rendering ...
Uint32 frameTime = SDL_GetTicks() - frameStart;
double fps = 1000.0 / frameTime;
std::cout << "FPS: " << fps << std::endl;
```

---

## Platform-Specific Notes

### Linux

- **Wayland users:** SDL2 works with both X11 and Wayland
- **Headless servers:** SDL2 requires a display. For headless, use dummy video driver:
  ```bash
  export SDL_VIDEODRIVER=dummy
  ```

### macOS

- **Code signing:** Modern macOS may require code signing for SDL apps
- **Retina displays:** SDL2 handles high-DPI automatically
- **Apple Silicon (M1/M2):** SDL2 has native ARM support via Homebrew

### Windows

- **Anti-virus:** Some anti-virus may flag SDL apps. Add exception if needed
- **Multiple monitors:** SDL2 handles multi-monitor setups automatically
- **MinGW vs MSVC:** Both work, choose based on your compiler

---

## Building RobotLib Simulation

After SDL2 is installed:

### Quick Build (Linux/macOS)

```bash
cd ~/RobotLib/simulation
make
make run-basic
```

### Manual Compilation

```bash
# Linux/macOS
g++ -std=c++11 -I../include \
    01_basic_simulation.cpp \
    -lSDL2 \
    -o sim

# Windows (MinGW)
g++ -std=c++11 -I../include \
    01_basic_simulation.cpp \
    -lmingw32 -lSDL2main -lSDL2 \
    -o sim.exe
```

### CMake Build (All platforms)

```bash
mkdir build && cd build
cmake ..
cmake --build .
./basic_simulation  # Linux/macOS
basic_simulation.exe  # Windows
```

---

## Additional Resources

### Official Documentation

- **SDL2 Wiki:** https://wiki.libsdl.org/
- **SDL2 Tutorials:** https://lazyfoo.net/tutorials/SDL/
- **API Reference:** https://wiki.libsdl.org/CategoryAPI

### Getting Help

- **SDL Forums:** https://discourse.libsdl.org/
- **Stack Overflow:** Tag `sdl-2`
- **RobotLib Issues:** https://github.com/konnorreynolds/RobotLib/issues

---

## Quick Reference

### Installation Commands

```bash
# Ubuntu/Debian
sudo apt-get install libsdl2-dev

# Fedora
sudo dnf install SDL2-devel

# Arch
sudo pacman -S sdl2

# macOS
brew install sdl2

# Windows (MSYS2)
pacman -S mingw-w64-x86_64-SDL2
```

### Compilation Templates

```bash
# Linux/macOS
g++ -std=c++11 -I../include FILE.cpp -lSDL2 -o OUT

# macOS (with pkg-config)
g++ -std=c++11 -I../include $(pkg-config --cflags --libs sdl2) FILE.cpp -o OUT

# Windows (MinGW)
g++ -std=c++11 -I../include FILE.cpp -lmingw32 -lSDL2main -lSDL2 -o OUT.exe
```

---

## Next Steps

After successful installation:

1. ✅ Verify with test program
2. ✅ Build basic simulation example
3. ✅ Read [Tutorials](TUTORIALS.md)
4. ✅ Explore [API Reference](API_REFERENCE.md)
5. ✅ Create your own simulation!

---

**Still having issues?**

[Open an issue](https://github.com/konnorreynolds/RobotLib/issues) with:
- Your OS and version
- SDL2 version (`sdl2-config --version`)
- Full error message
- Compilation command used

We're here to help! 🤝

---

**Last Updated:** November 7, 2025
**SDL2 Version Tested:** 2.0.20+
