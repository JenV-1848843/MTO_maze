#pragma once
#include <mutex>

// 1. Define the enum FIRST so the struct knows what it is
enum class SystemMode {
    CONTROLLER,           // Motors off or holding center
    AUTO,       // Auto-balancing or rolling ball to target
    CALIBRATING,    // Finding center, calculating offsets
    MANUAL        // Direct control via web UI 
};

// 2. PID controller
struct PIDController {
    // Tuning parameters
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    // Output limits (e.g., max servo angle in degrees)
    float output_limit = 20.0f; 
    float integral_limit = 20.0f; // Anti-windup limit

    // Memory / State
    float integral = 0.0f;
    float previous_error = 0.0f;

    // Call this when switching to AUTO mode so it starts fresh
    void reset() {
        integral = 0.0f;
        previous_error = 0.0f;
    }

    // The function you call every loop
    float calculate(float error,const int dt) {
        // 1. Calculate current error

        // 2. Proportional term
        float P = kp * error;

        // 3. Integral term (accumulated error)
        integral += error * dt;
        
        // Anti-windup: clamp the integral memory so it doesn't build up forever
        if (integral > integral_limit) integral = integral_limit;
        else if (integral < -integral_limit) integral = -integral_limit;
        
        float I = ki * integral;

        // 4. Derivative term (rate of change of error)
        float derivative = (error - previous_error) / dt;
        float D = kd * derivative;

        // Save current error for the next loop
        previous_error = error;

        // 5. Calculate total output
        float output = P + I + D;

        // 6. Clamp the output to safe servo angles
        if (output > output_limit) output = output_limit;
        else if (output < -output_limit) output = -output_limit;

        return output;
    }
};

struct SystemConfig {
    std::mutex mtx; 
    SystemMode mode = SystemMode::CALIBRATING;
    
    float offset_x = 43.0f;
    float offset_y = -43.0f;

    // Add your two PID controllers here!
    PIDController pid_x;
    PIDController pid_y;
    
    void setMode(SystemMode new_mode) {
        std::lock_guard<std::mutex> lock(mtx);
        mode = new_mode;
    }
    
    SystemMode getMode() {
        std::lock_guard<std::mutex> lock(mtx);
        return mode;
    }
};