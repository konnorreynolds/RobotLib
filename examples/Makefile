# ============================================================================
# RobotLib Examples - Build System
# ============================================================================
# Compiles all example programs for desktop testing
#
# Usage:
#   make              - Build all examples
#   make clean        - Remove all binaries
#   make basics       - Build only basics examples
#   make help         - Show this help
#
# Requirements:
#   - g++ or clang++ with C++11 support
#   - Standard C++ library
#
# Note: These are desktop builds. For embedded targets (Arduino, ESP32, etc),
#       use Arduino IDE or PlatformIO instead.
# ============================================================================

# Compiler settings
CXX := g++
CXXFLAGS := -std=c++11 -Wall -Wextra -pedantic -I../include
LDFLAGS :=

# Optimization (change to -O0 -g for debugging)
OPT := -O2

# Platform detection
UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Darwin)
    # macOS
    CXXFLAGS += -I/usr/local/include
    LDFLAGS += -L/usr/local/lib
endif

ifeq ($(UNAME_S),Linux)
    # Linux
    LDFLAGS += -pthread
endif

# Colors for output (optional)
NO_COLOR := \033[0m
GREEN := \033[0;32m
YELLOW := \033[0;33m
BLUE := \033[0;34m

# Find all .cpp files (excluding Arduino .ino files)
BASICS_SRCS := $(wildcard 01_basics/*.cpp)
FEEDBACK_SRCS := $(wildcard 02_with_feedback/*.cpp)
SYSTEMS_SRCS := $(wildcard 03_full_systems/*.cpp)
ALGORITHMS_SRCS := $(wildcard 04_algorithms/*.cpp)
ADVANCED_SRCS := $(wildcard 05_advanced/*.cpp)
FLUENT_SRCS := $(wildcard 06_fluent_api/*.cpp)

ALL_SRCS := $(BASICS_SRCS) $(FEEDBACK_SRCS) $(SYSTEMS_SRCS) \
            $(ALGORITHMS_SRCS) $(ADVANCED_SRCS) $(FLUENT_SRCS)

# Generate binary names (strip .cpp extension)
ALL_BINS := $(ALL_SRCS:.cpp=)

# Default target
.PHONY: all
all: $(ALL_BINS)
	@echo "$(GREEN)✓ All examples compiled successfully!$(NO_COLOR)"
	@echo ""
	@echo "Run an example:"
	@echo "  ./01_basics/simple_motor"
	@echo "  ./06_fluent_api/08_hello_units"

# Individual category targets
.PHONY: basics feedback systems algorithms advanced fluent
basics: $(BASICS_SRCS:.cpp=)
	@echo "$(GREEN)✓ Basics examples compiled$(NO_COLOR)"

feedback: $(FEEDBACK_SRCS:.cpp=)
	@echo "$(GREEN)✓ Feedback examples compiled$(NO_COLOR)"

systems: $(SYSTEMS_SRCS:.cpp=)
	@echo "$(GREEN)✓ Systems examples compiled$(NO_COLOR)"

algorithms: $(ALGORITHMS_SRCS:.cpp=)
	@echo "$(GREEN)✓ Algorithms examples compiled$(NO_COLOR)"

advanced: $(ADVANCED_SRCS:.cpp=)
	@echo "$(GREEN)✓ Advanced examples compiled$(NO_COLOR)"

fluent: $(FLUENT_SRCS:.cpp=)
	@echo "$(GREEN)✓ Fluent API examples compiled$(NO_COLOR)"

# Generic compilation rule
%: %.cpp
	@echo "$(BLUE)Compiling$(NO_COLOR) $<"
	@$(CXX) $(CXXFLAGS) $(OPT) $< $(LDFLAGS) -o $@

# Clean target
.PHONY: clean
clean:
	@echo "$(YELLOW)Cleaning binaries...$(NO_COLOR)"
	@rm -f $(ALL_BINS)
	@rm -f diff_drive ex[0-9]* fluent_demo hello motor_control mpc mpc_test
	@rm -f omni_robot quad_nav quadcopter
	@echo "$(GREEN)✓ Cleaned$(NO_COLOR)"

# Help target
.PHONY: help
help:
	@echo "============================================================================"
	@echo "RobotLib Examples - Build System"
	@echo "============================================================================"
	@echo ""
	@echo "$(GREEN)Targets:$(NO_COLOR)"
	@echo "  $(BLUE)make$(NO_COLOR)              - Build all examples"
	@echo "  $(BLUE)make basics$(NO_COLOR)       - Build 01_basics/ examples"
	@echo "  $(BLUE)make feedback$(NO_COLOR)     - Build 02_with_feedback/ examples"
	@echo "  $(BLUE)make systems$(NO_COLOR)      - Build 03_full_systems/ examples"
	@echo "  $(BLUE)make algorithms$(NO_COLOR)   - Build 04_algorithms/ examples"
	@echo "  $(BLUE)make advanced$(NO_COLOR)     - Build 05_advanced/ examples"
	@echo "  $(BLUE)make fluent$(NO_COLOR)       - Build 06_fluent_api/ examples"
	@echo "  $(BLUE)make clean$(NO_COLOR)        - Remove all binaries"
	@echo "  $(BLUE)make help$(NO_COLOR)         - Show this message"
	@echo ""
	@echo "$(GREEN)Building a single example:$(NO_COLOR)"
	@echo "  make 01_basics/simple_motor"
	@echo "  make 06_fluent_api/08_hello_units"
	@echo ""
	@echo "$(GREEN)Running examples:$(NO_COLOR)"
	@echo "  ./01_basics/simple_motor"
	@echo "  ./06_fluent_api/08_hello_units"
	@echo ""
	@echo "$(YELLOW)Note:$(NO_COLOR) For Arduino/ESP32, use Arduino IDE or PlatformIO"
	@echo "      See examples/platforms/ for platform-specific examples"
	@echo ""
	@echo "$(GREEN)Requirements:$(NO_COLOR)"
	@echo "  - g++ or clang++ with C++11 support"
	@echo "  - Standard C++ library"
	@echo ""

# Debug target - shows all detected sources
.PHONY: debug
debug:
	@echo "Found sources:"
	@echo "  Basics: $(BASICS_SRCS)"
	@echo "  Feedback: $(FEEDBACK_SRCS)"
	@echo "  Systems: $(SYSTEMS_SRCS)"
	@echo "  Algorithms: $(ALGORITHMS_SRCS)"
	@echo "  Advanced: $(ADVANCED_SRCS)"
	@echo "  Fluent: $(FLUENT_SRCS)"
	@echo ""
	@echo "Will build binaries:"
	@echo "  $(ALL_BINS)"

# Quick test - build and run hello example
.PHONY: test
test: 06_fluent_api/08_hello_units
	@echo "$(YELLOW)Running hello example...$(NO_COLOR)"
	@./06_fluent_api/08_hello_units

# Install target (copies headers to system location)
.PHONY: install
install:
	@echo "$(YELLOW)Installing RobotLib headers...$(NO_COLOR)"
	@sudo mkdir -p /usr/local/include/RobotLib
	@sudo cp -r ../include/* /usr/local/include/RobotLib/
	@echo "$(GREEN)✓ Installed to /usr/local/include/RobotLib/$(NO_COLOR)"
	@echo ""
	@echo "Now you can compile with:"
	@echo "  g++ -std=c++11 myprogram.cpp -I/usr/local/include/RobotLib"

# Uninstall target
.PHONY: uninstall
uninstall:
	@echo "$(YELLOW)Uninstalling RobotLib headers...$(NO_COLOR)"
	@sudo rm -rf /usr/local/include/RobotLib
	@echo "$(GREEN)✓ Uninstalled$(NO_COLOR)"

# Check for required tools
.PHONY: check
check:
	@echo "Checking build environment..."
	@which $(CXX) > /dev/null || (echo "$(YELLOW)✗ Compiler not found: $(CXX)$(NO_COLOR)" && exit 1)
	@$(CXX) --version
	@echo "$(GREEN)✓ Build environment OK$(NO_COLOR)"

# Verbose compilation (shows full commands)
.PHONY: verbose
verbose:
	$(MAKE) all CXXFLAGS="$(CXXFLAGS) -v"

# Fast build (parallel jobs)
.PHONY: fast
fast:
	$(MAKE) -j$(shell nproc || sysctl -n hw.ncpu || echo 4) all

# Build with debug symbols
.PHONY: debug-build
debug-build:
	$(MAKE) clean
	$(MAKE) all OPT="-O0 -g"
	@echo "$(GREEN)✓ Debug build complete$(NO_COLOR)"
	@echo "Run with gdb:"
	@echo "  gdb ./01_basics/simple_motor"

# Count lines of code in examples
.PHONY: stats
stats:
	@echo "$(BLUE)Example Statistics:$(NO_COLOR)"
	@echo "  Total .cpp files: $$(find . -name '*.cpp' | wc -l)"
	@echo "  Total .ino files: $$(find . -name '*.ino' | wc -l)"
	@echo "  Lines of code:    $$(find . -name '*.cpp' -o -name '*.ino' | xargs wc -l | tail -1)"
	@echo ""

# ============================================================================
# End of Makefile
# ============================================================================
