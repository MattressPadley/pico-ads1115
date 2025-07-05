# Justfile for ADS1115 Library
# Provides convenient build and flash commands for Raspberry Pi Pico

# Default recipe - show available commands
default:
    @just --list

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