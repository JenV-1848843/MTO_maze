# MTO Maze — Ball-in-Maze Robot

A Raspberry Pi–based autonomous maze-solving system that uses a camera, OpenCV image recognition, and PID-controlled servo motors to guide a ball through a physical maze. A web interface running on port `8080` lets you switch between control modes in real time.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Prerequisites](#prerequisites)
4. [Install Dependencies](#install-dependencies)
5. [Build OpenCV from Source](#build-opencv-from-source)
6. [Clone the Repository](#clone-the-repository)
7. [Build the Project](#build-the-project)
8. [Run the Application](#run-the-application)
9. [Access the Web Interface](#access-the-web-interface)
10. [Control Modes](#control-modes)
11. [Project Structure](#project-structure)
12. [Troubleshooting](#troubleshooting)

---

## System Overview

```
┌─────────────────────────────────────────────────┐
│                  Raspberry Pi                   │
│                                                 │
│  ┌─────────────┐     ┌─────────────────────┐    │
│  │  webApp     │     │  MTO_maze           │    │
│  │  (port 8080)│     │  (OpenCV + PID)     │    │
│  │             │     │                     │    │
│  │  index.html │     │  Camera → BFS path  │    │
│  │  Start/Stop │────▶│  Servo X / Servo Y  │   │
│  └─────────────┘     └─────────────────────┘    │
│         ▲                      │                │
│         │                      ▼                │
│    Browser on             GPIO (gpiod)          │
│    same network        PWM servo channels       │
└─────────────────────────────────────────────────┘
```

Two executables are built:

| Executable | Description |
|---|---|
| `webApp` | Serves `index.html` on port `8080`, reads IMU via I2C, and drives servos in CONTROLLER or AUTO mode |
| `MTO_maze` | Runs the OpenCV vision pipeline: detects the maze layout, tracks the ball, computes BFS path, and actuates servos via PID |

---

## Hardware Requirements

- Raspberry Pi 4 (or 3B+) running Raspberry Pi OS (Debian-based)
- Pi Camera Module (libcamera-compatible)
- 2× PWM servo motors (channels 2 and 3)
- Physical maze platform with ball

---

## Prerequisites

Make sure your system is up to date:

```bash
sudo apt update && sudo apt upgrade -y
```

Install the required build tools and libraries:

```bash
sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libgpiod-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libcamera \
    python3-pip \
    wget unzip
```

> **Note:** `libgpiod-dev` is required for servo motor control. Without it, `MTO_maze` will still build but servo output will be disabled.
> **Note:** `python` is needed only for testing the camera module.

---

## Build OpenCV from Source

The project requires OpenCV built with GStreamer support (for the Pi Camera pipeline). The prebuilt apt package does **not** include GStreamer — you must build from source.

> [!CAUTION]
> **Danger:** If the steps shown below do not work for you, a tutorial on how to install and build OpenCV on your system can be found at https://docs.opencv.org/3.4/d7/d9f/tutorial_linux_install.html

### 1. Install OpenCV build dependencies

```bash
sudo apt install -y \
    libgtk-3-dev \
    libpng-dev \
    libjpeg-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libatlas-base-dev \
    gfortran
```

### 2. Download OpenCV source

```bash
cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.9.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.9.0.zip
unzip opencv.zip
unzip opencv_contrib.zip
mv opencv-4.9.0 opencv
mv opencv_contrib-4.9.0 opencv_contrib
```

### 3. Configure and build

```bash
cd ~/opencv
mkdir build && cd build

cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D WITH_GSTREAMER=ON \
    -D WITH_GTK=ON \
    -D BUILD_EXAMPLES=OFF \
    ..

make -j$(nproc)
sudo make install
sudo ldconfig
```

> ⏱ This will take **30–60 minutes** on a Raspberry Pi 4. On a Pi 3 it may take longer.

### 4. Verify the build

```bash
python3 -c "import cv2; print(cv2.__version__); print(cv2.getBuildInformation())" | grep GStreamer
```

You should see `GStreamer: YES`.

---

## Clone the Repository

```bash
cd ~
git clone https://github.com/JenV-1848843/MTO_maze.git
cd MTO_maze
```

---

## Build the Project

### 1. Create the build directory

```bash
mkdir build && cd build
```

### 2. Run CMake

The CMakeLists.txt auto-detects Linux and builds both `webApp` and `MTO_maze`. It also detects whether `libgpiod` is available and enables servo control accordingly.

```bash
cmake ..
```

You should see output like:

```
-- Building webApp for Linux/Raspberry Pi
-- Found OpenCV: /usr/local (found version "4.9.0")
-- Found libgpiod - servo control ENABLED
```

If you see `libgpiod not found`, servo output will be compiled out but the rest of the project will still build.

### 3. Compile

```bash
make -j$(nproc)
```

The compiled binaries will be placed in the `build/` directory at the project root:

```
MTO_maze/build/webApp
MTO_maze/build/MTO_maze
```

---

## Run the Application

Both executables should be run from the project root so that relative paths (like `index.html`) resolve correctly.

### Run the web controller

```bash
cd ~/MTO_maze
sudo ./build/webApp
```

> `sudo` is required for GPIO/PWM access on the Raspberry Pi.

You should see:

```
[Hardware] Initializing PWM & I2C...
[Hardware] Ready. Waiting for mode change...
[Main] System running. Press 'x' + Enter to exit.
```

### Run the vision + maze solver

Open a second terminal:

```bash
cd ~/MTO_maze
sudo ./build/MTO_maze
```

You will be prompted to enter the target maze cell:

```
Enter target X: 3
Enter target Y: 4
```

The camera will open, detect the maze layout, find the ball, and begin navigating.

### Stopping the application

Press `x` then `Enter` in the terminal running `webApp`, or press `Ctrl+C`. Both threads shut down cleanly and servos return to neutral.

---

## Access the Web Interface

Once `webApp` is running, open a browser on any device **on the same local network** as the Raspberry Pi and navigate to:

```
http://<raspberry-pi-ip>:8080
```

To find your Pi's IP address:

```bash
hostname -I
```

Example: if your Pi's IP is `192.168.1.42`, go to:

```
http://192.168.1.42:8080
```

The interface serves `index.html` from the project root and exposes two buttons:

| Button | Action |
|---|---|
| **Start** | Switches the system to `AUTO` mode — reads IMU and drives servos based on tilt |
| **Stop** | Switches the system to `CALIBRATING` mode — holds the platform still |

---

## Control Modes

| Mode | Description |
|---|---|
| `CONTROLLER` | Holds the platform flat (zero error PID) |
| `AUTO` | Reads IMU pitch/roll, filters with exponential moving average, drives servos to level |
| `CALIBRATING` | Platform is held still; used after Stop is pressed |
| `MANUAL` | Reserved for future implementation |

---

## Project Structure

```
MTO_maze/
├── src/
│   ├── webBasedMain.cpp      # webApp entry point (IMU, servos, web server)
│   ├── main_jen.cpp          # MTO_maze entry point (OpenCV, BFS, PID)
│   ├── motorcontrol.cpp      # GPIO PWM servo control (Pi only)
│   └── pi2c.cpp              # I2C communication with MPU-6050
├── include/
│   ├── webcontrol.hpp        # Async HTTP server, serves index.html on port 8080
│   ├── SystemConfig.hpp      # Shared state: mode, PID params, offsets (thread-safe)
│   ├── motorcontrol.h
│   ├── rpi_pwm.h
│   ├── pi2c.h
│   ├── headers/
│   │   ├── config.h          # Maze dimensions and constants
│   │   ├── maze.h / maze.cpp # BFS maze solver
│   │   ├── cell.h / cell.cpp # Maze cell representation
│   │   ├── utils.h / utils.cpp
│   │   ├── ballPosition.h    # OpenCV ball tracking
│   │   ├── wall.h
│   │   ├── position.h
│   │   └── direction.h
│   └── sources/              # Implementations for headers above
├── index.html                # Web UI (Start / Stop buttons)
├── CMakeLists.txt
└── README.md
```

---

## Troubleshooting

**Camera fails to open**
```
Failed to open camera
```
Make sure `libcamera` and the GStreamer libcamera plugin are installed and the camera is enabled:
```bash
sudo raspi-config   # Interface Options → Camera → Enable
```
Then verify the pipeline works independently:
```bash
libcamera-hello
```

**OpenCV not found during CMake**

The CMakeLists.txt looks for OpenCV at `~/opencv/build`. If you installed to a different path, update this line in `CMakeLists.txt`:
```cmake
set(OpenCV_DIR ~/opencv/build)
```

**Servo channels not responding**

Confirm `libgpiod` was found during CMake (`SERVO_CONTROL_ENABLED` must be defined). Also verify the servo wiring matches channels 2 and 3, and that you are running with `sudo`.

**Cannot reach the web interface**

- Confirm `webApp` is running and shows no errors.
- Check the Pi's IP with `hostname -I`.
- Make sure your browser device and the Pi are on the same Wi-Fi/network.
- Check for firewall rules blocking port `8080`:
  ```bash
  sudo ufw allow 8080
  ```

**I2C device not found (IMU)**

Check that the MPU-6050 is detected:
```bash
sudo i2cdetect -y 1
```
You should see `68` in the grid. If not, check wiring (SDA → Pin 3, SCL → Pin 5) and enable I2C:
```bash
sudo raspi-config   # Interface Options → I2C → Enable
```
