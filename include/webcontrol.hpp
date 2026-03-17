#pragma once
// webcontrol.hpp — embeddable HTTP control panel library
//
// Usage:
//   1. #include "webcontrol.hpp"
//   2. Create a WebControl instance with your callbacks
//   3. Call start() to begin serving — it runs in a background thread
//   4. Call stop() to shut down
//
// Example:
//   WebControl ctrl(8080, "index.html");
//   ctrl.on_start = [] { std::cout << "Started!\n"; };
//   ctrl.on_stop  = [] { std::cout << "Stopped!\n"; };
//   ctrl.serve();   // blocks, or use serve_async() for background

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <stdexcept>
#include <cstring>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

class WebControl {
public:
    // ── Callbacks — set these before calling serve() / serve_async() ──────────
    std::function<void()> on_start;   // called when Start button is pressed
    std::function<void()> on_stop;    // called when Stop button is pressed

    // ── Constructor ───────────────────────────────────────────────────────────
    // port      : TCP port to listen on (default 8080)
    // html_path : path to the HTML file to serve at "/" (default "index.html")
    explicit WebControl(int port = 8080, std::string html_path = "index.html")
        : port_(port), html_path_(std::move(html_path)) {}

    ~WebControl() { stop(); }

    // Disable copy
    WebControl(const WebControl&)            = delete;
    WebControl& operator=(const WebControl&) = delete;

    // ── serve() — blocks forever (use in a dedicated thread or main) ──────────
    void serve() {
        init_socket();
        std::cout << "[WebControl] Serving on http://0.0.0.0:" << port_ << "\n";
        accept_loop();
    }

    // ── serve_async() — spins up a background thread, returns immediately ─────
    void serve_async() {
        if (server_thread_.joinable())
            throw std::runtime_error("WebControl: already running");
        init_socket();
        std::cout << "[WebControl] Serving (async) on http://0.0.0.0:" << port_ << "\n";
        server_thread_ = std::thread([this]{ accept_loop(); });
    }

    // ── stop() — signals the accept loop to exit and joins the thread ─────────
    void stop() {
        quit_.store(true);
        if (server_fd_ >= 0) {
            ::shutdown(server_fd_, SHUT_RDWR);
            ::close(server_fd_);
            server_fd_ = -1;
        }
        if (server_thread_.joinable())
            server_thread_.join();
    }

    // ── running() — true if the app logic is in the "started" state ──────────
    bool running() const { return running_.load(); }

private:
    // ── Config ────────────────────────────────────────────────────────────────
    int         port_;
    std::string html_path_;

    // ── State ─────────────────────────────────────────────────────────────────
    std::atomic<bool> running_{false};
    std::atomic<bool> quit_{false};
    int               server_fd_{-1};
    std::thread       server_thread_;

    static constexpr int    BACKLOG     = 8;
    static constexpr size_t BUFFER_SIZE = 4096;

    // ── Socket setup ──────────────────────────────────────────────────────────
    void init_socket() {
        server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) throw std::runtime_error("WebControl: socket() failed");

        int opt = 1;
        ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(static_cast<uint16_t>(port_));

        if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
            throw std::runtime_error("WebControl: bind() failed — port in use?");
        if (::listen(server_fd_, BACKLOG) < 0)
            throw std::runtime_error("WebControl: listen() failed");
    }

    // ── Accept loop ───────────────────────────────────────────────────────────
    void accept_loop() {
        while (!quit_.load()) {
            int client_fd = ::accept(server_fd_, nullptr, nullptr);
            if (client_fd < 0) break;   // socket closed or error
            std::thread([this, client_fd]{ handle_client(client_fd); }).detach();
        }
    }

    // ── Per-connection handler ────────────────────────────────────────────────
    void handle_client(int fd) {
        char buf[BUFFER_SIZE] = {};
        ssize_t n = ::recv(fd, buf, sizeof(buf) - 1, 0);
        if (n <= 0) { ::close(fd); return; }

        std::string raw(buf, static_cast<size_t>(n));
        auto [method, path] = parse_request_line(raw);

        if (path == "/" || path == "/index.html") {
            std::string html = read_file(html_path_);
            if (html.empty())
                send_response(fd, 404, "Not Found", "text/plain", "index.html not found");
            else
                send_response(fd, 200, "OK", "text/html; charset=utf-8", html);

        } else if (path.rfind("/command", 0) == 0) {
            std::string action = query_param(path, "action");

            if (action == "start") {
                if (!running_.exchange(true)) {
                    if (on_start) on_start();
                    send_response(fd, 200, "OK", "text/plain", "Started");
                } else {
                    send_response(fd, 200, "OK", "text/plain", "Already running");
                }
            } else if (action == "stop") {
                if (running_.exchange(false)) {
                    if (on_stop) on_stop();
                    send_response(fd, 200, "OK", "text/plain", "Stopped");
                } else {
                    send_response(fd, 200, "OK", "text/plain", "Already stopped");
                }
            } else {
                send_response(fd, 400, "Bad Request", "text/plain",
                              "Use ?action=start or ?action=stop");
            }
        } else {
            send_response(fd, 404, "Not Found", "text/plain", "Not found");
        }

        ::close(fd);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    static std::string read_file(const std::string& path) {
        std::ifstream f(path);
        if (!f) return "";
        std::ostringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }

    static void send_response(int fd, int status, const std::string& status_text,
                               const std::string& content_type, const std::string& body)
    {
        std::ostringstream r;
        r << "HTTP/1.1 " << status << " " << status_text << "\r\n"
          << "Content-Type: "   << content_type << "\r\n"
          << "Content-Length: " << body.size()  << "\r\n"
          << "Access-Control-Allow-Origin: *\r\n"
          << "Connection: close\r\n\r\n"
          << body;
        std::string s = r.str();
        ::send(fd, s.c_str(), s.size(), 0);
    }

    static std::pair<std::string, std::string> parse_request_line(const std::string& raw) {
        std::istringstream ss(raw);
        std::string method, path;
        ss >> method >> path;
        return {method, path};
    }

    static std::string query_param(const std::string& path, const std::string& key) {
        auto q = path.find('?');
        if (q == std::string::npos) return "";
        std::string qs   = path.substr(q + 1);
        std::string srch = key + "=";
        auto pos = qs.find(srch);
        if (pos == std::string::npos) return "";
        pos += srch.size();
        auto end = qs.find('&', pos);
        return qs.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
    }
};