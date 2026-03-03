#ifndef __PID_BALL
#define __PID_BALL

#include <chrono>
#include <algorithm>

/**
 * PID controller for balancing a ball on a servo-tilted axis.
 * 
 * Convention:
 *   - Positive ball position = ball is to the right
 *   - Positive servo angle   = plate tilts so ball moves right
 *   - To correct a positive error, we tilt negative (opposite direction)
 **/
class PID {
public:
    /**
     * \param kp Proportional gain
     * \param ki Integral gain
     * \param kd Derivative gain
     * \param min_angle Minimum servo angle in degrees (e.g. -90)
     * \param max_angle Maximum servo angle in degrees (e.g.  90)
     * \param integral_limit Clamp for integral term to prevent windup
     */
    PID(float kp, float ki, float kd,
        float min_angle = -45.0f, float max_angle = 45.0f,
        float integral_limit = 30.0f)
        : kp(kp), ki(ki), kd(kd),
          min_angle(min_angle), max_angle(max_angle),
          integral_limit(integral_limit) {}

    /**
     * Call once before starting the control loop.
     */
    void reset() {
        integral    = 0.0f;
        last_error  = 0.0f;
        first_tick  = true;
    }

    /**
     * Compute the servo angle needed to move the ball toward the setpoint.
     * Call this every control loop iteration.
     * 
     * \param setpoint  Desired ball position
     * \param measured  Current ball position
     * \return Servo angle in degrees, clamped to [min_angle, max_angle]
     */
    float compute(float setpoint, float measured) {
        // Time delta
        auto now = std::chrono::steady_clock::now();
        float dt = 0.0f;
        if (!first_tick) {
            dt = std::chrono::duration<float>(now - last_time).count();
        }
        last_time  = now;
        first_tick = false;

        float error = setpoint - measured;

        // Integral with anti-windup clamp
        if (dt > 0.0f) {
            integral += error * dt;
            integral = std::clamp(integral, -integral_limit, integral_limit);
        }

        // Derivative (avoid divide-by-zero on first tick)
        float derivative = (dt > 0.0f) ? (error - last_error) / dt : 0.0f;
        last_error = error;

        // PID output — negate because a positive error means ball is too far
        // right, so we tilt the plate left (negative angle) to correct it
        float output = -(kp * error + ki * integral + kd * derivative);

        return std::clamp(output, min_angle, max_angle);
    }

    // Tune gains at runtime
    void setGains(float p, float i, float d) { kp = p; ki = i; kd = d; }

private:
    float kp, ki, kd;
    float min_angle, max_angle;
    float integral_limit;

    float integral   = 0.0f;
    float last_error = 0.0f;
    bool  first_tick = true;
    std::chrono::steady_clock::time_point last_time;
};

#endif