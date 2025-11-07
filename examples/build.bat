@echo off
REM ============================================================================
REM RobotLib Examples - Windows Build Script
REM ============================================================================
REM Easy build script for Windows users
REM
REM Requirements:
REM   - Visual Studio 2017 or newer (with C++ tools)
REM   OR
REM   - MinGW-w64 (g++)
REM
REM Usage:
REM   build.bat           - Build all examples (Release)
REM   build.bat debug     - Build with debug info
REM   build.bat clean     - Clean build directory
REM   build.bat help      - Show this help
REM ============================================================================

setlocal enabledelayedexpansion

REM Check for help
if "%1"=="help" goto :help
if "%1"=="-h" goto :help
if "%1"=="--help" goto :help
if "%1"=="/?" goto :help

REM Check for clean
if "%1"=="clean" goto :clean

REM Detect compiler
where cl.exe >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set COMPILER=MSVC
    goto :build_msvc
)

where g++.exe >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set COMPILER=MinGW
    goto :build_mingw
)

echo ERROR: No C++ compiler found!
echo.
echo Please install one of:
echo   - Visual Studio 2017+ with C++ tools
echo   - MinGW-w64 (https://www.mingw-w64.org/)
echo   - MSYS2 with mingw-w64-x86_64-gcc
echo.
exit /b 1

REM ============================================================================
REM Build with MSVC (Visual Studio)
REM ============================================================================
:build_msvc
echo ============================================================================
echo RobotLib Examples - Building with MSVC
echo ============================================================================
echo.

REM Determine build type
set BUILD_TYPE=Release
if "%1"=="debug" set BUILD_TYPE=Debug

REM Create build directory
if not exist build mkdir build
cd build

REM Run CMake
echo Running CMake...
cmake .. -G "Visual Studio 17 2022" -A x64
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: CMake configuration failed!
    cd ..
    exit /b 1
)

echo.
echo Building examples (%BUILD_TYPE%)...
cmake --build . --config %BUILD_TYPE%
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Build failed!
    cd ..
    exit /b 1
)

cd ..

echo.
echo ============================================================================
echo Build successful!
echo ============================================================================
echo.
echo Binaries are in: build\bin\%BUILD_TYPE%\
echo.
echo To run an example:
echo   cd build\bin\%BUILD_TYPE%
echo   fluent_08_hello_units.exe
echo.
goto :end

REM ============================================================================
REM Build with MinGW
REM ============================================================================
:build_mingw
echo ============================================================================
echo RobotLib Examples - Building with MinGW
echo ============================================================================
echo.

REM Determine build type
set BUILD_TYPE=Release
if "%1"=="debug" set BUILD_TYPE=Debug

REM Create build directory
if not exist build mkdir build
cd build

REM Run CMake
echo Running CMake...
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: CMake configuration failed!
    cd ..
    exit /b 1
)

echo.
echo Building examples...
cmake --build .
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Build failed!
    cd ..
    exit /b 1
)

cd ..

echo.
echo ============================================================================
echo Build successful!
echo ============================================================================
echo.
echo Binaries are in: build\bin\
echo.
echo To run an example:
echo   cd build\bin
echo   fluent_08_hello_units.exe
echo.
goto :end

REM ============================================================================
REM Clean build directory
REM ============================================================================
:clean
echo Cleaning build directory...
if exist build (
    rmdir /s /q build
    echo Done!
) else (
    echo Nothing to clean.
)
goto :end

REM ============================================================================
REM Show help
REM ============================================================================
:help
echo ============================================================================
echo RobotLib Examples - Windows Build Script
echo ============================================================================
echo.
echo Usage:
echo   build.bat           - Build all examples (Release mode)
echo   build.bat debug     - Build with debug information
echo   build.bat clean     - Remove build directory
echo   build.bat help      - Show this help
echo.
echo Requirements:
echo   - Visual Studio 2017+ with C++ tools
echo     OR
echo   - MinGW-w64 (g++)
echo.
echo After building:
echo   MSVC:   build\bin\Release\fluent_08_hello_units.exe
echo   MinGW:  build\bin\fluent_08_hello_units.exe
echo.
echo For more information, see:
echo   - QUICKSTART.md
echo   - examples/README.md
echo.
goto :end

:end
endlocal
