# Justfile for ADS1115 Library
# Provides convenient build and flash commands for Raspberry Pi Pico

# Default recipe - show available commands
default:
    @just --list


# Setup clangd configuration for ARM cross-compilation
setup-clangd:
    @echo "Setting up clangd for ARM cross-compilation..."
    @./scripts/setup-clangd.sh

# Build the project
build:
    @echo "Building ADS1115 library and examples..."
    @mkdir -p build
    @cd build && cmake ..
    @cd build && make -j$(nproc)
    @echo "Build complete! Files in build/ directory:"
    @ls -la build/*.uf2 2>/dev/null || echo "No .uf2 files found"

# Clean build directory
clean:
    @echo "Cleaning build directory..."
    @rm -rf build/
    @echo "Clean complete!"

# Build and flash the basic ADC example
flash: build
    @echo "Flashing basic_adc example to Pico..."
    @if [ ! -f "build/basic_adc.uf2" ]; then \
        echo "Error: basic_adc.uf2 not found in build directory"; \
        exit 1; \
    fi
    @echo "Put your Pico in BOOTSEL mode (hold BOOTSEL while connecting USB)"
    @echo "Press Enter when ready..."
    @read
    @picotool load build/basic_adc.uf2 --force
    @echo "Flash complete! Check serial output for ADC readings."

# Development build with verbose output
dev-build:
    @echo "Development build with verbose output..."
    @mkdir -p build
    @cd build && cmake -DCMAKE_BUILD_TYPE=Debug ..
    @cd build && make -j$(nproc) VERBOSE=1

# Release build (optimized)
release-build:
    @echo "Release build (optimized)..."
    @mkdir -p build
    @cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
    @cd build && make -j$(nproc)

    # Monitor serial output from the Pico
monitor:
    @echo "Monitoring serial output from Pico..."
    @echo "Press Ctrl+A then Ctrl+X to exit"
    @picocom -b 115200 --imap lfcrlf /dev/cu.usbmodem*

# Test build: build, flash, reboot, and monitor
test-build: build
    @echo "Test build: flashing and monitoring basic_adc example..."
    @if [ ! -f "build/basic_adc.uf2" ]; then \
        echo "Error: basic_adc.uf2 not found in build directory"; \
        exit 1; \
    fi
    @echo "Put your Pico in BOOTSEL mode (hold BOOTSEL while connecting USB)"
    @echo "Press Enter when ready..."
    @read
    @picotool load build/basic_adc.uf2 --force
    @echo "Flashing complete! Rebooting Pico..."
    @sleep 2
    @echo "Starting serial monitor..."
    @echo "Press Ctrl+A then Ctrl+X to exit monitoring"
    @picocom -b 115200 --imap lfcrlf /dev/cu.usbmodem*
