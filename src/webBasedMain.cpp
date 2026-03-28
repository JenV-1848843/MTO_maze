#include <iostream>
#include <cstdint>
#include <array>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <mutex>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "rpi_pwm.h"
#include "pi2c.h"
#include "webcontrol.hpp"
#include "SystemConfig.hpp" // Your new shared state header

const int LOOPTIME{10}; // ms

// Global shared state & quit signal
SystemConfig shared_state;
std::atomic<bool> app_quit{false};

void signal_handler(int) { 
    app_quit.store(true); 
}

// Helper function: check if a key has been pressed (non-blocking)
bool keyPressed(char target) {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    char ch = 0;
    read(STDIN_FILENO, &ch, 1);

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return ch == target;
}

void PID_control(float error_x, float error_y, RPI_PWM servo_x, RPI_PWM servo_y){
    // Get live PID parameters safely from the shared state
    float kp_x, ki_x, kd_x;
    float kp_y, ki_y, kd_y;
    {
        std::lock_guard<std::mutex> lock(shared_state.mtx);
        kp_x = shared_state.pid_x.kp;
        ki_x = shared_state.pid_x.ki;
        kd_x = shared_state.pid_x.kd;
        
        kp_y = shared_state.pid_y.kp;
        ki_y = shared_state.pid_y.ki;
        kd_y = shared_state.pid_y.kd;
    }

    // Calculate the new required servo angles
    float output_angle_x = shared_state.pid_x.calculate(error_x, LOOPTIME);
    float output_angle_y = shared_state.pid_y.calculate(error_y, LOOPTIME);
    // Write to the servos
    servo_x.setDutyCycle(angleToDutyCycle(output_angle_x, shared_state.offset_x));
    servo_y.setDutyCycle(angleToDutyCycle(output_angle_y, shared_state.offset_y));
}

// ---------------------------------------------------------
// HARDWARE CONTROL THREAD (Runs strictly at 100Hz)
// ---------------------------------------------------------
void hardware_control_loop() {
    std::cout << "[Hardware] Initializing PWM & I2C...\n";
    
    // Setup PWM
    RPI_PWM servo_x, servo_y;
    servo_x.start(2, 50); // Channel 2, 50Hz
    servo_y.start(3, 50); // Channel 3, 50Hz

    // Setup IMU
    Pi2c imu(0x68);
    imu.init();
    usleep(100000); // Give IMU time to settle

    // Filtering variables
    float alpha = 0.1f;
    float roll_filtered = 0.0f;
    float pitch_filtered = 0.0f;
    
    const int BUF_SIZE = 5;
    float roll_buf[BUF_SIZE] = {0};
    float pitch_buf[BUF_SIZE] = {0};
    int buf_index = 0;

    // Get initial offsets (Baseline reading)
    auto accel_0 = imu.readAccel();
    float x_0 = accel_0[0];
    float y_0 = accel_0[1];

    std::cout << "[Hardware] Ready. Waiting for mode change...\n";

    // Timing setup for strictly 100Hz loop (10ms)
    auto next_wake_time = std::chrono::steady_clock::now();

    while (!app_quit.load()) {
        // 1. Calculate next wake time
        next_wake_time += std::chrono::milliseconds(LOOPTIME);

        // 2. Safely read current config
        SystemMode current_mode;
        float offset_x, offset_y;
        {
            std::lock_guard<std::mutex> lock(shared_state.mtx);
            current_mode = shared_state.mode;
            offset_x = shared_state.offset_x;
            offset_y = shared_state.offset_y;
        }

        // 3. State Machine Execution
        switch (current_mode) {
            case SystemMode::CONTROLLER: {
                // Just hold the platform flat
                // servo_x.setDutyCycle(angleToDutyCycle(0, offset_x));
                // servo_y.setDutyCycle(angleToDutyCycle(0, offset_y));
                float error_x = 0.0f;
                float error_y = 0.0f;

                PID_control(error_x, error_y, servo_x, servo_y);
                break;
            }
            case SystemMode::AUTO: {
                // Read and filter IMU
                auto accel_1 = imu.readAccel();
                roll_filtered  = alpha * accel_1[0] + (1.0f - alpha) * roll_filtered;
                pitch_filtered = alpha * accel_1[1] + (1.0f - alpha) * pitch_filtered;

                roll_buf[buf_index]  = roll_filtered;
                pitch_buf[buf_index] = pitch_filtered;
                buf_index = (buf_index + 1) % BUF_SIZE;

                float roll_avg = 0, pitch_avg = 0;
                for (int i = 0; i < BUF_SIZE; i++){
                    roll_avg  += roll_buf[i];
                    pitch_avg += pitch_buf[i];
                }
                roll_avg  /= BUF_SIZE;
                pitch_avg /= BUF_SIZE;

                float x_m = roll_filtered - x_0;
                float y_m = pitch_filtered - y_0;

                // Move servos based on pitch/roll
                if (x_m < 50.0) servo_x.setDutyCycle(angleToDutyCycle(x_m/2, offset_x));
                else if (x_m < -50) servo_x.setDutyCycle(angleToDutyCycle(-25, offset_x));
                else servo_x.setDutyCycle(angleToDutyCycle(25, offset_x));

                if (y_m < 50.0) servo_y.setDutyCycle(angleToDutyCycle(y_m/2, offset_y));
                else if (y_m < -50) servo_y.setDutyCycle(angleToDutyCycle(-25, offset_y));
                else servo_y.setDutyCycle(angleToDutyCycle(25, offset_y));
                
                break;
            }
            
            case SystemMode::CALIBRATING:
            case SystemMode::MANUAL:{

                // Future implementation
                break;
            }
        }

        // 4. Sleep exactly until the next 10ms boundary
        std::this_thread::sleep_until(next_wake_time);
    }

    // Cleanup
    servo_x.stop();
    servo_y.stop();
    std::cout << "[Hardware] Shutting down servos.\n";
}

// ---------------------------------------------------------
// MAIN APPLICATION
// ---------------------------------------------------------
int main() {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 1. Setup Web Control
    WebControl ctrl(8080, "index.html");
    
    // Link the web buttons to the state machine
    ctrl.on_start = [] { 
        std::cout << "[Web] Start pressed -> Mode: TARGETING\n"; 
        shared_state.setMode(SystemMode::AUTO);
    };
    ctrl.on_stop  = [] { 
        std::cout << "[Web] Stop pressed -> Mode: IDLE\n"; 
        shared_state.setMode(SystemMode::CALIBRATING);
    };
    
    ctrl.serve_async();

    // 2. Start hardware loop in a separate thread
    std::thread control_thread(hardware_control_loop);

    std::cout << "[Main] System running. Press 'x' + Enter to exit.\n";

    // 3. Main thread UI/exit loop
    while (!app_quit.load()) {
        if (keyPressed('x')) {
            std::cout << "\n[Main] 'x' pressed. Exiting...\n";
            app_quit.store(true);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 4. Clean shutdown
    ctrl.stop();
    if (control_thread.joinable()) {
        control_thread.join();
    }

    std::cout << "[Main] Goodbye!\n";
    return 0;
}