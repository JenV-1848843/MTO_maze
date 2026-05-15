#pragma once

/* ============================================================
   webserver.hpp — lightweight HTTP server for MTO_maze
   ------------------------------------------------------------
   Include this file once in main_jen.cpp.

   main_jen.cpp must declare the following shared variables
   (webserver.hpp references them as extern):

       std::atomic<bool>  ws_should_start;   // set true → main starts loop
       std::atomic<bool>  ws_should_stop;    // set true → main breaks loop
       std::atomic<bool>  ws_is_running;     // set by main, read by /status
       std::atomic<int>   ws_target_x;       // parsed from /command?action=start
       std::atomic<int>   ws_target_y;
       std::atomic<int>   ws_servo_angle_x;  // tracked by main after servo moves
       std::atomic<int>   ws_servo_angle_y;
       std::mutex         ws_pid_mutex;
       WsPIDParams        ws_pid_params;     // struct defined below, read by main

   Call startWebServer() once from main() before entering any loop.
   The server runs on a detached background thread and never blocks main.

   Endpoints (all GET):
     /status
     /command?action=start&x=N&y=N
     /command?action=stop
     /command?action=servo&id=N&pulse=N
     /api/pid?kpx=f&kix=f&kdx=f&kpy=f&kiy=f&kdy=f
     /api/save_zero
   ============================================================ */

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <fstream>
#include <sstream>
#include <cstring>
#include <iostream>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#ifdef SERVO_CONTROL_ENABLED
#include "motorcontrol.h"
#endif

/* ── Shared state — declared here, defined in main_jen.cpp ── */

// PID parameter struct — accessed by both the web thread and the main loop.
// Always lock ws_pid_mutex before reading or writing.
struct WsPIDParams {
    float kpx = 0.5f, kix = 0.0f, kdx = 0.1f;
    float kpy = 0.5f, kiy = 0.0f, kdy = 0.1f;
};

extern std::atomic<bool>  ws_should_start;
extern std::atomic<bool>  ws_should_stop;
extern std::atomic<bool>  ws_is_running;
extern std::atomic<int>   ws_target_x;
extern std::atomic<int>   ws_target_y;
extern std::atomic<int>   ws_servo_angle_x;
extern std::atomic<int>   ws_servo_angle_y;
extern std::atomic<float> ws_servo_offset_x;
extern std::atomic<float> ws_servo_offset_y;
extern std::mutex         ws_pid_mutex;
extern WsPIDParams        ws_pid_params;

/* ── Port ─────────────────────────────────────────────────── */
static constexpr int WS_PORT = 8080;

/* ── Zero offset save path ────────────────────────────────── */
// nginx serves files from /var/www/html — saving here keeps it accessible.
static constexpr const char* WS_ZERO_OFFSET_PATH = "/var/www/html/zero_offset.cfg";

/* ============================================================
   Internal helpers
   ============================================================ */

// Parse a single named query parameter from a URL string.
// Returns empty string if not found.
static std::string ws_parse_param(const std::string& url, const std::string& key) {
    std::string search = key + "=";
    size_t pos = url.find(search);
    if (pos == std::string::npos) return "";
    pos += search.size();
    size_t end = url.find_first_of("& \t\r\nH", pos); // stop at separator or HTTP version
    return url.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
}

// Send a plain-text HTTP response to the client socket.
static void ws_respond(int fd, int status_code, const std::string& body) {
    std::string status_text = (status_code == 200) ? "OK" :
                              (status_code == 400) ? "Bad Request" :
                              (status_code == 403) ? "Forbidden" : "Internal Server Error";

    std::ostringstream resp;
    resp << "HTTP/1.1 " << status_code << " " << status_text << "\r\n"
     << "Content-Type: text/plain\r\n"
     << "Access-Control-Allow-Origin: *\r\n"
     << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
     << "Access-Control-Allow-Headers: Content-Type\r\n"
     << "Connection: close\r\n"
     << "Content-Length: " << body.size() << "\r\n"
     << "\r\n"
     << body;

    std::string r = resp.str();
    send(fd, r.c_str(), r.size(), MSG_NOSIGNAL);
}

// Convert servo angle (degrees, 0–180) to pulse width (µs).
// Mirrors angle_to_pulse() in main_jen.cpp exactly.
static int ws_angle_to_pulse(int angle) {
    return 1000 + (angle * 1000) / 180;
}

/* ============================================================
   Request handler — called for every incoming HTTP request.
   ============================================================ */
static void ws_handle_request(int client_fd, const std::string& raw) {
    if (raw.find("OPTIONS") == 0) {
        ws_respond(client_fd, 200, "");
        return;
    }
    // Extract the request line (first line only — all we need)
    std::string url;
    {
        size_t start = raw.find(' ');
        size_t end   = raw.find(' ', start + 1);
        if (start == std::string::npos || end == std::string::npos) {
            ws_respond(client_fd, 400, "Malformed request");
            return;
        }
        url = raw.substr(start + 1, end - start - 1);
    }

    /* ── GET /status ─────────────────────────────────────────
       Returns "running" or "idle".
       Browser polls this every 2 s to check backend health
       and sync the running state after a page reload.
    ───────────────────────────────────────────────────────── */
    if (url == "/status") {
        ws_respond(client_fd, 200, ws_is_running.load() ? "running" : "idle");
        return;
    }

    /* ── GET /command?action=start&x=N&y=N ──────────────────
       Validates x and y, sets ws_target_x / ws_target_y,
       then raises ws_should_start for main() to pick up.
       Rejected if the loop is already running.
    ───────────────────────────────────────────────────────── */
    if (url.find("/command") == 0) {
        std::string action = ws_parse_param(url, "action");

        if (action == "start") {
            if (ws_is_running.load()) {
                ws_respond(client_fd, 403, "Already running");
                return;
            }
            std::string sx = ws_parse_param(url, "x");
            std::string sy = ws_parse_param(url, "y");
            if (sx.empty() || sy.empty()) {
                ws_respond(client_fd, 400, "Missing x or y");
                return;
            }
            int x, y;
            try { x = std::stoi(sx); y = std::stoi(sy); }
            catch (...) {
                ws_respond(client_fd, 400, "x and y must be integers");
                return;
            }
            if (x < 0 || x > 7 || y < 0 || y > 7) {
                ws_respond(client_fd, 400, "x and y must be 0-7");
                return;
            }
            ws_target_x.store(x);
            ws_target_y.store(y);
            ws_should_start.store(true);
            ws_respond(client_fd, 200, "OK");
            return;
        }

        /* ── GET /command?action=stop ────────────────────────
           Raises ws_should_stop. main() breaks its inner loop
           on the next iteration when it checks this flag.
        ─────────────────────────────────────────────────── */
        if (action == "stop") {
            ws_should_stop.store(true);
            ws_respond(client_fd, 200, "OK");
            return;
        }

        /* ── GET /command?action=servo&id=N&pulse=N ──────────
           Drives a servo immediately. Only allowed when the
           main loop is NOT running (calibration mode).
           id: 2 (X axis) or 3 (Y axis).
           pulse: microseconds, clamped to [1000, 2000].
        ─────────────────────────────────────────────────── */
        if (action == "servo") {
            if (ws_is_running.load()) {
                ws_respond(client_fd, 403, "Cannot move servo while running");
                return;
            }
            std::string sid    = ws_parse_param(url, "id");
            std::string spulse = ws_parse_param(url, "pulse");
            if (sid.empty() || spulse.empty()) {
                ws_respond(client_fd, 400, "Missing id or pulse");
                return;
            }
            int id, pulse;
            try { id = std::stoi(sid); pulse = std::stoi(spulse); }
            catch (...) {
                ws_respond(client_fd, 400, "id and pulse must be integers");
                return;
            }
            if (id != 2 && id != 3) {
                ws_respond(client_fd, 400, "Servo id must be 2 or 3");
                return;
            }
            // Clamp pulse to safe servo range
            pulse = std::max(1000, std::min(2000, pulse));

            #ifdef SERVO_CONTROL_ENABLED
            ServoMotor_Control(id, pulse);
            #else
            std::cout << "[webserver] ServoMotor_Control(" << id << ", " << pulse << ") [stub]\n";
            #endif

            // Back-calculate angle from pulse and store for save_zero
            int angle = (pulse - 1000) * 180 / 1000;
            if (id == 2) ws_servo_angle_x.store(angle);
            else         ws_servo_angle_y.store(angle);

            ws_respond(client_fd, 200, "OK");
            return;
        }

        ws_respond(client_fd, 400, "Unknown action");
        return;
    }

    /* ── GET /api/pid?kpx=f&kix=f&kdx=f&kpy=f&kiy=f&kdy=f ──
       Parses six floats and writes them into ws_pid_params
       under ws_pid_mutex. main()'s PID loop reads from there
       on each iteration — takes effect immediately.
    ───────────────────────────────────────────────────────── */
    if (url.find("/api/pid") == 0) {
        auto parse_float = [&](const std::string& key, float& out) -> bool {
            std::string v = ws_parse_param(url, key);
            if (v.empty()) return false;
            try { out = std::stof(v); return true; }
            catch (...) { return false; }
        };

        WsPIDParams p;
        if (!parse_float("kpx", p.kpx) || !parse_float("kix", p.kix) || !parse_float("kdx", p.kdx) ||
            !parse_float("kpy", p.kpy) || !parse_float("kiy", p.kiy) || !parse_float("kdy", p.kdy)) {
            ws_respond(client_fd, 400, "Missing or invalid PID parameter");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(ws_pid_mutex);
            ws_pid_params = p;
        }

        ws_respond(client_fd, 200, "OK");
        return;
    }

    /* ── GET /api/offset?id=N&value=f ────────────────────────
       Updates the live servo angle offset (degrees, float).
       The offset is added to every angle_to_pulse() call in the
       main control loop, so changes take effect immediately on
       the next servo write — no restart needed.

       id:    2 (X axis) or 3 (Y axis)
       value: float, the NEW cumulative offset (not a delta)

       If the main loop is idle, the servo is driven to the
       offset position right away so the user sees the effect
       during calibration.
    ───────────────────────────────────────────────────────── */
    if (url.find("/api/offset") == 0) {
        std::string sid    = ws_parse_param(url, "id");
        std::string svalue = ws_parse_param(url, "value");
        if (sid.empty() || svalue.empty()) {
            ws_respond(client_fd, 400, "Missing id or value");
            return;
        }
        int   id;
        float value;
        try { id = std::stoi(sid); value = std::stof(svalue); }
        catch (...) {
            ws_respond(client_fd, 400, "id must be int, value must be float");
            return;
        }
        if (id != 2 && id != 3) {
            ws_respond(client_fd, 400, "Servo id must be 2 or 3");
            return;
        }

        if (id == 2) ws_servo_offset_x.store(value);
        else         ws_servo_offset_y.store(value);

        // When idle, drive the servo so calibration is visible.
        if (!ws_is_running.load()) {
            float a = value;
            if (a < 0.0f)   a = 0.0f;
            if (a > 180.0f) a = 180.0f;
            int pulse = static_cast<int>(1000.0f + (a * 1000.0f) / 180.0f);
            pulse = std::max(1000, std::min(2000, pulse));
            #ifdef SERVO_CONTROL_ENABLED
            ServoMotor_Control(id, pulse);
            #else
            std::cout << "[webserver] offset id=" << id << " value=" << value
                      << " pulse=" << pulse << " [stub]\n";
            #endif
        }

        ws_respond(client_fd, 200, "OK");
        return;
    }

    /* ── GET /api/save_zero ──────────────────────────────────
       Writes current servo angles to WS_ZERO_OFFSET_PATH.
       Format: "angleX angleY\n"  (plain integers, easy to
       parse back with  file >> zeroX >> zeroY).
       Returns "OK {angleX} {angleY}" so the browser can
       confirm the values that were saved.
    ───────────────────────────────────────────────────────── */
    if (url.find("/api/save_zero") == 0) {
        int ax = ws_servo_angle_x.load();
        int ay = ws_servo_angle_y.load();

        std::ofstream f(WS_ZERO_OFFSET_PATH, std::ios::trunc);
        if (!f.is_open()) {
            ws_respond(client_fd, 500, "Failed to open zero offset file");
            return;
        }
        f << ax << " " << ay << "\n";
        f.close();

        ws_respond(client_fd, 200, "OK " + std::to_string(ax) + " " + std::to_string(ay));
        return;
    }

    ws_respond(client_fd, 400, "Unknown endpoint");
}

/* ============================================================
   Server thread — accepts connections in a loop.
   ============================================================ */
static void ws_server_thread() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "[webserver] Failed to create socket\n";
        return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(WS_PORT);

    if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[webserver] Bind failed on port " << WS_PORT << "\n";
        close(server_fd);
        return;
    }

    if (listen(server_fd, 8) < 0) {
        std::cerr << "[webserver] Listen failed\n";
        close(server_fd);
        return;
    }

    std::cout << "[webserver] Listening on port " << WS_PORT << "\n";

    while (true) {
        int client_fd = accept(server_fd, nullptr, nullptr);
        if (client_fd < 0) continue;

        char buf[2048] = {};
        recv(client_fd, buf, sizeof(buf) - 1, 0);

        ws_handle_request(client_fd, std::string(buf));
        close(client_fd);
    }
}

/* ============================================================
   Public API — call once from main()
   ============================================================ */
inline void startWebServer() {
    std::thread(ws_server_thread).detach();
}