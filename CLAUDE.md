# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**MTO_maze** is a physical maze-solving robot that uses computer vision to navigate. The project has dual-platform support:
- **Windows**: Standalone maze solving with OpenCV visualization (educational/testing)
- **Linux/Raspberry Pi**: Full hardware integration with web-based control, real-time camera feed, and servo motor control

The system performs maze pathfinding using BFS, detects walls and ball position via OpenCV, and controls two servo motors (X/Y) to move the ball through the maze.

## Architecture

### Core Maze Engine (`include/headers/`)
- **Maze class**: BFS-based pathfinding on an 8×8 grid with wall detection. Methods include `bfs()` for path planning, `directionTo()` for movement guidance, and `mmsUntilTurn()` for motor control integration.
- **Cell class**: Represents a single grid cell with wall information (north, south, east, west).
- **BallPosition class**: Tracks the ball's location using vision feedback from camera.
- **Configuration**: Hard-coded constants in `config.h` (maze dimensions, color thresholds for red/orange detection, pixel-to-mm calibration, wall geometry).

### Vision System
- **OpenCV integration**: Uses OpenCV for image processing and camera capture.
- **GStreamer pipeline** (webBasedMain.cpp): Captures video from Raspberry Pi camera using `libcamerasrc` with 640×480 @ 30fps.
- **Color detection**: Red/orange thresholds tuned for physical maze markers.

### Hardware Control (`include/motorcontrol.h`, `include/pi2c.h`, `include/rpi_pwm.h`)
- **Servo control**: PWM-based servo positioning for X and Y axes (servo IDs 2 and 3).
- **I2C communication**: pi2c interface for hardware communication.
- **GPIO PWM**: Low-level PWM signal generation for servo pulses.

### Web Interface (`include/webcontrol.hpp`)
- Simple HTTP server (port 8080) serving an embedded HTML control panel.
- Callback-based Start/Stop buttons tied to application state.
- **Shared state**: `SystemConfig` class provides thread-safe shared state between web control and motor loops.
- Non-blocking operation via background thread.

### Main Entry Points
- **main_jen.cpp**: Windows/Linux maze solver with OpenCV visualization and user input for target selection.
- **webBasedMain.cpp**: Linux-only full-stack: camera capture → maze detection → pathfinding → motor servo control → web dashboard.
- **main_servo.cpp**: Standalone servo motor testing via Makefile (uses gpiod library).

## Build Commands

### CMake (Primary Build System)

**Windows:**
```bash
mkdir build && cd build
cmake .. && cmake --build .
# Outputs: MTO_maze.exe (maze solver with OpenCV)
```

**Linux:**
```bash
mkdir build && cd build
cmake .. && cmake --build .
# Outputs: build/webApp (full stack), build/MTO_maze (maze solver)
```

Set `OpenCV_DIR` if OpenCV discovery fails:
```bash
cmake -DOpenCV_DIR=/path/to/opencv/build ..
```

### Makefile (Servo Motor Control)

```bash
make              # Compile servo control binary
make run          # Compile and run (requires sudo for GPIO)
make clean        # Remove build artifacts
```

## Key Configuration Points

Edit `include/headers/config.h` to adjust:
- **Maze grid size**: `amountCellRows`, `amountCellCols` (currently 8×8)
- **Wall dimensions**: `outerWallLength`, `innerWallThickness` (affects calibration)
- **Color detection**: `redThresholdLow/High`, `orangeThresholdLow/High`
- **Calibration**: `pixelsPerMm` (computed from outer wall pixel width at runtime)
- **Physical dimensions**: `ballDiameter` (used for position accuracy)

## Running the Applications

**Full-stack web app (Linux only):**
```bash
./build/webApp
# Open browser to http://localhost:8080
# Camera feed appears on page load, use Start/Stop buttons to trigger pathfinding
```

**Standalone maze solver (Windows or Linux):**
```bash
./build/MTO_maze
# Prompts for target X, Y coordinates
# Displays maze layout and OpenCV image windows if built on Windows
```

## Development Notes

- **Platform-specific code**: CMake conditionals distinguish Windows (`if(WIN32)`) and Linux (`elseif(UNIX)`). Do not assume Linux features (GPIO, I2C, GStreamer) are available on Windows.
- **Thread safety**: `SystemConfig` and `app_quit` use atomic types and mutexes for shared state between web control and motor threads.
- **Camera pipeline**: GStreamer string in webBasedMain.cpp is Raspberry Pi-specific; will fail on non-Pi Linux without adjustments.
- **Servo pulse calculation**: `angle_to_pulse()` converts degrees to microsecond pulse width (1000 + angle*1000/180); adjust formula if servo specifications differ.
- **Non-blocking keyboard input**: `keyPressed()` in webBasedMain.cpp configures termios for non-blocking stdin; used for testing without web interface.

## Dependencies

- **OpenCV**: 3.x or 4.x, build with cmake installed
- **GStreamer** (Linux only): gstreamer1.0, gstreamer1.0-plugins-base
- **libgpiod** (Raspberry Pi): GPIO daemon library for PWM servo control
- **CMake**: 3.16 or later

## Notes for Future Work

- **Calibration**: `pixelsPerMm` is computed at runtime from outer wall dimensions; if physical maze changes, may need manual adjustment or auto-calibration from image.
- **Servo ranges**: PWM pulse values hardcoded (1000–2000 µs); verify against actual servo specs if servos are replaced.
- **Web hosting**: `index.nginx-debian.html` is present but appears to be example/reference; webcontrol.hpp serves its own embedded HTML panel.
