#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>
#include <thread>

// for color conversions
#include <opencv2/opencv.hpp>

#include "../include/headers/config.h"
#include "../include/headers/utils.h"
#include "../include/headers/cell.h"
#include "../include/headers/maze.h"
#include "../include/headers/position.h"
#include "../include/headers/wall.h"
#include "../include/headers/ballPosition.h"

#include "../include/motorcontrol.h"
#include "../include/SystemConfig.hpp"
#include "../include/Webserver.hpp"

std::atomic<bool>  ws_should_start{false};
std::atomic<bool>  ws_should_stop{false};
std::atomic<bool>  ws_is_running{false};
std::atomic<int>   ws_target_x{0};
std::atomic<int>   ws_target_y{0};
std::atomic<int>   ws_servo_angle_x{0};
std::atomic<int>   ws_servo_angle_y{0};
std::atomic<float> ws_servo_offset_x{0.0f};
std::atomic<float> ws_servo_offset_y{0.0f};
std::mutex         ws_pid_mutex;
WsPIDParams        ws_pid_params;

static int angle_to_pulse(float angle, float offset = 0.0) {
	return 1000 + ((angle - offset) * 1000) / 180;
}

static constexpr float PID_OUTPUT_LIMIT = 30.0f;

int main(int argc, char* argv[]) {
	startWebServer();

	std::cout << "OpenCV version: " << CV_VERSION << std::endl;

	cv::VideoCapture cap(
		"libcamerasrc ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=1 sync=false",
		cv::CAP_GSTREAMER
	);

	if (!cap.isOpened()) {
		std::cerr << "Failed to open camera" << std::endl;
		return -1;
	}

	// -----------------------------
	// Init servo motors
	// -----------------------------
	Init_GPIO_Control();
	Init_ServoMotor_Control();
	std::cout << "Setting servos to 0" << std::endl;
	ServoMotor_Control(2, angle_to_pulse(0.0f, ws_servo_offset_x.load()));
	ServoMotor_Control(3, angle_to_pulse(0.0f, ws_servo_offset_y.load()));

	// -----------------------------
	// Init PID Controllers
	// -----------------------------
	PIDController pid_x, pid_y;
	pid_x.output_limit = pid_y.output_limit = PID_OUTPUT_LIMIT;

	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// -----------------------------
	// Capture initial frame & save snapshot
	// -----------------------------
	cv::Mat frame, tempFrame;
	auto captureStart = std::chrono::steady_clock::now();

	while (true) {
		cap >> tempFrame;
		if (tempFrame.empty()) break;

		frame = tempFrame.clone();

		auto now = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - captureStart).count();
		if (elapsed >= 5000)
			break;
	}

	cv::imwrite("/var/www/html/maze_snapshot.jpg", frame);
	std::cout << "Snapshot saved. Waiting for start command from web interface...\n";

	// -----------------------------
	// Outer loop: run → finish → wait → run again
	// -----------------------------
	while (true) {

		// Wait for start command from web interface
		while (!ws_should_start.load()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		ws_should_start.store(false);
		ws_is_running.store(true);

		// Read target set by web interface
		int targetX = ws_target_x.load();
		int targetY = ws_target_y.load();
		std::cout << "Start received. Target: (" << targetX << ", " << targetY << ")\n";

		// Reset maze and run BFS for new target
		Maze maze;
		readMazeConfig(frame, maze.getConfig());
		maze.bfs(maze.getConfig()[targetX][targetY]);
		maze.print();

		BallPosition pos = trackBall(frame);
		while (!pos.found) {
			cap.grab();
			cap.retrieve(frame);
			pos = trackBall(frame);
		}
		std::cout << "Found ball at (" << pos.x << ", " << pos.y << ")" << std::endl;

		Cell* c;
		auto last_time = std::chrono::steady_clock::now();

		// Inner loop: control until goal, stop command, or unreachable
		while (true) {
			cap.grab();
			cap.retrieve(frame);

			// Check for stop command from web interface
			if (ws_should_stop.load()) {
				std::cout << "Stop command received.\n";
				ws_should_stop.store(false);
				ws_is_running.store(false);
				ServoMotor_Control(2, angle_to_pulse(0.0f, ws_servo_offset_x.load()));
				ServoMotor_Control(3, angle_to_pulse(0.0f, ws_servo_offset_y.load()));
				break;
			}

			pos = trackBall(frame);
			c = &maze.getConfig()[pos.x][pos.y];

			if (c == &maze.getConfig()[targetX][targetY]) {
				std::cout << "Destination reached.\n";
				ws_is_running.store(false);
				ServoMotor_Control(2, angle_to_pulse(0.0f, ws_servo_offset_x.load()));
				ServoMotor_Control(3, angle_to_pulse(0.0f, ws_servo_offset_y.load()));
				break;
			} else if (!c->visited) {
				std::cout << "Unreachable, no path to destination.\n";
				ws_is_running.store(false);
				ServoMotor_Control(2, angle_to_pulse(0.0f, ws_servo_offset_x.load()));
				ServoMotor_Control(3, angle_to_pulse(0.0f, ws_servo_offset_y.load()));
				break;
			}

			double mms = maze.mmsUntilTurn(*c, &pos);
			Direction dir = maze.directionTo(c, c->next);
			std::cout << "moving " << mms << " mms to " << to_string(dir) << std::endl;

			if (pos.found && c->next != nullptr) {
				double error_x;
				double error_y;
				switch (dir) {
				case Direction::UP:
					error_x = -1.0;
					error_y = -mms;
					break;
				case Direction::DOWN:
					error_x = -1.0;
					error_y = mms;
					break;
				case Direction::LEFT:
					error_x = mms;
					error_y = 0.0;
					break;
				case Direction::RIGHT:
					error_x = -mms;
					error_y = 0.0;
					break;
				default:
					error_x = 0.0;
					error_y = 0.0;
					break;
				}

				// Re-read PID constants from web interface on each iteration
				{
					std::lock_guard<std::mutex> lock(ws_pid_mutex);
					pid_x.kp = ws_pid_params.kpx; pid_x.ki = ws_pid_params.kix; pid_x.kd = ws_pid_params.kdx;
					pid_y.kp = ws_pid_params.kpy; pid_y.ki = ws_pid_params.kiy; pid_y.kd = ws_pid_params.kdy;
				}

				auto now = std::chrono::steady_clock::now();
				int dt_ms = static_cast<int>(
					std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count()
				);
				if (dt_ms < 1) dt_ms = 1;
				last_time = now;

				float out_x = pid_x.calculate(error_x, dt_ms);
				float out_y = pid_y.calculate(error_y, dt_ms);

				std::cout << "Py: " << pid_y.kp << "  out: " << out_x << " / " << out_y << std::endl;
				ServoMotor_Control(2, angle_to_pulse(out_x, ws_servo_offset_x.load()));
				ServoMotor_Control(3, angle_to_pulse(out_y, ws_servo_offset_y.load()));
			}
		} // end inner loop

		std::cout << "Run complete. Waiting for next start command...\n";

	} // end outer loop

	cap.release();
	return 0;
}