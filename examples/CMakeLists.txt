# ============================================================================
# RobotLib Examples - CMake Build System
# ============================================================================
# Cross-platform build system for all examples
# Works on: Windows (MSVC, MinGW), Linux, macOS
#
# Usage:
#   mkdir build && cd build
#   cmake ..
#   cmake --build .
#
# Or on Windows with Visual Studio:
#   mkdir build && cd build
#   cmake ..
#   cmake --build . --config Release
# ============================================================================

cmake_minimum_required(VERSION 3.10)
project(RobotLib_Examples LANGUAGES CXX)

# C++11 required
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/../include)

# Compiler warnings
if(MSVC)
    add_compile_options(/W4 /WX-)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

# Optimization for release builds
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Helper function to add an example
function(add_example EXAMPLE_NAME SOURCE_FILE)
    add_executable(${EXAMPLE_NAME} ${SOURCE_FILE})

    # Set output directory
    set_target_properties(${EXAMPLE_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )

    # Platform-specific settings
    if(WIN32)
        # Windows: Enable console for output
        set_target_properties(${EXAMPLE_NAME} PROPERTIES
            WIN32_EXECUTABLE FALSE
        )
    endif()
endfunction()

# ============================================================================
# 01_basics examples
# ============================================================================
file(GLOB BASICS_SOURCES "${CMAKE_SOURCE_DIR}/01_basics/*.cpp")
foreach(SOURCE ${BASICS_SOURCES})
    get_filename_component(EXAMPLE_NAME ${SOURCE} NAME_WE)
    add_example("basics_${EXAMPLE_NAME}" ${SOURCE})
endforeach()

# ============================================================================
# 02_with_feedback examples
# ============================================================================
file(GLOB FEEDBACK_SOURCES "${CMAKE_SOURCE_DIR}/02_with_feedback/*.cpp")
foreach(SOURCE ${FEEDBACK_SOURCES})
    get_filename_component(EXAMPLE_NAME ${SOURCE} NAME_WE)
    add_example("feedback_${EXAMPLE_NAME}" ${SOURCE})
endforeach()

# ============================================================================
# 03_full_systems examples
# ============================================================================
file(GLOB SYSTEMS_SOURCES "${CMAKE_SOURCE_DIR}/03_full_systems/*.cpp")
foreach(SOURCE ${SYSTEMS_SOURCES})
    get_filename_component(EXAMPLE_NAME ${SOURCE} NAME_WE)
    add_example("systems_${EXAMPLE_NAME}" ${SOURCE})
endforeach()

# ============================================================================
# 04_algorithms examples
# ============================================================================
file(GLOB ALGORITHMS_SOURCES "${CMAKE_SOURCE_DIR}/04_algorithms/*.cpp")
foreach(SOURCE ${ALGORITHMS_SOURCES})
    get_filename_component(EXAMPLE_NAME ${SOURCE} NAME_WE)
    add_example("algorithms_${EXAMPLE_NAME}" ${SOURCE})
endforeach()

# ============================================================================
# 05_advanced examples
# ============================================================================
file(GLOB ADVANCED_SOURCES "${CMAKE_SOURCE_DIR}/05_advanced/*.cpp")
foreach(SOURCE ${ADVANCED_SOURCES})
    get_filename_component(EXAMPLE_NAME ${SOURCE} NAME_WE)
    add_example("advanced_${EXAMPLE_NAME}" ${SOURCE})
endforeach()

# ============================================================================
# 06_fluent_api examples
# ============================================================================
file(GLOB FLUENT_SOURCES "${CMAKE_SOURCE_DIR}/06_fluent_api/*.cpp")
foreach(SOURCE ${FLUENT_SOURCES})
    get_filename_component(EXAMPLE_NAME ${SOURCE} NAME_WE)
    add_example("fluent_${EXAMPLE_NAME}" ${SOURCE})
endforeach()

# ============================================================================
# Installation (optional)
# ============================================================================
install(DIRECTORY ${CMAKE_SOURCE_DIR}/../include/
        DESTINATION include/RobotLib
        FILES_MATCHING PATTERN "*.h")

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "============================================================================")
message(STATUS "RobotLib Examples - Build Configuration")
message(STATUS "============================================================================")
message(STATUS "")
message(STATUS "Build type:       ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard:     C++${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler:         ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Install prefix:   ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Binary directory: ${CMAKE_BINARY_DIR}/bin")
message(STATUS "")
message(STATUS "To build:")
message(STATUS "  cmake --build .")
if(WIN32)
    message(STATUS "  cmake --build . --config Release")
endif()
message(STATUS "")
message(STATUS "To install:")
message(STATUS "  cmake --install .")
message(STATUS "")
message(STATUS "To run examples:")
if(WIN32)
    message(STATUS "  bin\\Release\\fluent_08_hello_units.exe")
else()
    message(STATUS "  ./bin/fluent_08_hello_units")
endif()
message(STATUS "")
message(STATUS "============================================================================")
message(STATUS "")

# ============================================================================
# Testing (optional)
# ============================================================================
enable_testing()

# Add a simple test
add_test(NAME hello_test
         COMMAND fluent_08_hello_units
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ============================================================================
# CPack (optional - for creating packages)
# ============================================================================
set(CPACK_PACKAGE_NAME "RobotLib-Examples")
set(CPACK_PACKAGE_VERSION "2.2.0")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RobotLib Examples")
set(CPACK_PACKAGE_VENDOR "Konnor Reynolds")

if(WIN32)
    set(CPACK_GENERATOR "ZIP")
else()
    set(CPACK_GENERATOR "TGZ")
endif()

include(CPack)

# ============================================================================
# End of CMakeLists.txt
# ============================================================================
