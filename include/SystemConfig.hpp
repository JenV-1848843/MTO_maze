#pragma once
#include <mutex>

// 1. Define the enum FIRST so the struct knows what it is
enum class SystemMode {
    IDLE,           // Motors off or holding center
    CALIBRATING,    // Finding center, calculating offsets
    MANUAL,         // Direct control via web UI 
    TARGETING       // Auto-balancing or rolling ball to target
};

// 2. Define the struct SECOND
struct SystemConfig {
    std::mutex mtx; // Protects against simultaneous read/writes from different threads
    
    SystemMode mode = SystemMode::IDLE;
    
    // Servo offsets
    float offset_x = 43.0f;
    float offset_y = -43.0f;
    
    // Thread-safe helper to change modes (called setMode in main.cpp)
    void setMode(SystemMode new_mode) {
        std::lock_guard<std::mutex> lock(mtx);
        mode = new_mode;
    }
    
    // Thread-safe helper to read the current mode
    SystemMode getMode() {
        std::lock_guard<std::mutex> lock(mtx);
        return mode;
    }
};