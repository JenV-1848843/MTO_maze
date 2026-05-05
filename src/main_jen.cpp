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
std::mutex         ws_pid_mutex;
WsPIDParams        ws_pid_params;

static int angle_to_pulse(int angle) {
	return 1000 + (angle * 1000) / 180;
}

// PID tuning — adjust these during testing
static constexpr float PID_KP = 0.5f;
static constexpr float PID_KI = 0.0f;
static constexpr float PID_KD = 0.1f;
static constexpr float PID_OUTPUT_LIMIT = 30.0f;

int main(int argc, char* argv[]) {
	startWebServer();

	int targetX, targetY;

	// targetX and targetY are set by the web interface via /command?action=start
	targetX = ws_target_x.load();
	targetY = ws_target_y.load();

    Maze maze;
    // maze.print();

    //std::string imagePathRelative = "/home/doolhof/repo/MTO_maze/include/images/";
    //std::string imagePath = imagePathRelative + "maze_5.jpg";
    //cv::Mat frame = cv::imread(imagePath, cv::IMREAD_COLOR);
    //cv::resize(frame, frame, cv::Size(), 0.3, 0.3);
	
    
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
	std::cout << "opencv test" << std::endl;
	cv::Mat img = cv::Mat::zeros(100, 100, CV_8UC3);
	std::cout << "mat size: " << img.size() << std::endl;
	// std::cout << cv::getBuildInformation() << std::endl;

	cv::VideoCapture cap(
		"libcamerasrc ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=1 sync=false",
		cv::CAP_GSTREAMER
	);

	if (!cap.isOpened()) {
		std::cerr << "Failed to open camera" << std::endl;
		return -1;
	}

	// cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'C'));
	// cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	// cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);


	// -----------------------------
	// Init servo motoren
	// -----------------------------

	Init_GPIO_Control();
	Init_ServoMotor_Control();
	std::cout << "setting servos to 0" << std::endl;
	ServoMotor_Control(2, angle_to_pulse(0)); // neutral
	ServoMotor_Control(3, angle_to_pulse(0));
	// _____________________________________

	// -----------------------------
	// Init PID Controllers
	// -----------------------------

	PIDController pid_x, pid_y;
	pid_x.output_limit = pid_y.output_limit = PID_OUTPUT_LIMIT;

	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	// ______________________________________

	cv::Mat frame, tempFrame;

	// start timing
	auto start = std::chrono::steady_clock::now();

	while (true)
	{
		cap >> tempFrame;
		if (tempFrame.empty()) break;

		// show the video stream
		cv::imshow("Video", tempFrame);

		// keep updating the last valid frame
		frame = tempFrame.clone();

		// break after 2 seconds
		auto now = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
		if (elapsed >= 5000)
			break;

		// required for imshow to update
		if (cv::waitKey(1) == 27) // optional: press ESC to exit early
			break;
	}
	// if (frame.empty()){
		// std::cerr << "failed to capture frame" << std::endl;
		// return -1;
	// }
	cv::imshow("Camera", frame);
	cv::waitKey(0);

	// Save snapshot for the web interface to display
	cv::imwrite("/var/www/html/maze_snapshot.jpg", frame);
	std::cout << "Snapshot saved. Waiting for start command from web interface...\n";

	// Wait until the web interface sends /command?action=start
	while (!ws_should_start.load()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	ws_should_start.store(false);
	ws_is_running.store(true);

	// Read target set by web interface
	targetX = ws_target_x.load();
	targetY = ws_target_y.load();
	std::cout << "Start received. Target: (" << targetX << ", " << targetY << ")\n";

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

	while (true) {
		cap.grab();
		cap.retrieve(frame);

		// Check for stop command from web interface
		if (ws_should_stop.load()) {
			std::cout << "Stop command received.\n";
			ws_should_stop.store(false);
			ws_is_running.store(false);
			ServoMotor_Control(2, angle_to_pulse(0));
			ServoMotor_Control(3, angle_to_pulse(0));
			break;
		}

		pos = trackBall(frame);

		c = &maze.getConfig()[pos.x][pos.y];

		if (c == &maze.getConfig()[targetX][targetY]) {
			std::cout << "This is the destination\n";
			ws_is_running.store(false);
			ServoMotor_Control(2, angle_to_pulse(0));
			ServoMotor_Control(3, angle_to_pulse(0));
			Exit_Motor_Control();
			return 0;
		} else if (!c->visited) {
			std::cout << "Unreachable, no path to destination\n";
			ws_is_running.store(false);
			ServoMotor_Control(2, angle_to_pulse(0));
			ServoMotor_Control(3, angle_to_pulse(0));
			Exit_Motor_Control();
			return 0;
		}

		std::cout << "Reachable! Following path:\n";
		// std::cout << "(" << current->getX() << "," << current->getY() << ") -> ";
		double mms = maze.mmsUntilTurn(*c, &pos);
		Direction dir = maze.directionTo(c, c->next);
		std::cout << "moving " << mms << " mms to " << to_string(dir) << std::endl;

		if (pos.found && c->next != nullptr) {
			double error_x;
			double error_y;
			switch (dir)
			{
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
			// // Target: centre of the next waypoint cell in mm
			// float next_mm_x = (c->next->getX() + 0.5f) * innerWallLength;
			// float next_mm_y = (c->next->getY() + 0.5f) * innerWallLength;

			// // Error = setpoint - measurement (positive error → tilt toward target)
			// float error_x = next_mm_x - static_cast<float>(pos.mmX);
			// float error_y = next_mm_y - static_cast<float>(pos.mmY);

			// Measure actual elapsed time in ms for correct PID integration
			auto now = std::chrono::steady_clock::now();
			int dt_ms = static_cast<int>(
				std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count()
			);
			if (dt_ms < 1) dt_ms = 1;
			last_time = now;

			float out_x = pid_x.calculate(error_x, dt_ms);
			float out_y = pid_y.calculate(error_y, dt_ms);

			std::cout << "Px: " << pid_y.kp << std::endl;
			std::cout << "Ex: " << out_x <<  " and " << out_y << std::endl;
			ServoMotor_Control(2, angle_to_pulse(static_cast<int>(out_x)));
			ServoMotor_Control(3, angle_to_pulse(static_cast<int>(out_y)));
		}
	}
	// Cell* c = &maze.getConfig()[pos.x][pos.y];




	// while (true) {
		//cap >> frame;
		//if (frame.empty()) break;
		//cv::imshow("Camera", frame);
		//cv::waitKey(5);
		// Flush old frames
		//while (cap.grab()) {
			// keep grabbing to skip buffered frames
			//if (cap.get(cv::CAP_PROP_POS_FRAMES) == -1) break;
		//}
		// cap.grab();
		// cap.retrieve(frame);

		// ---- Your processing ----
		// readMazeConfig(frame, maze.getConfig());
		// BallPosition pos = trackBall(frame);

	// 	maze.print();
	// 	maze.resetWalls();
	// 	cv::imshow("Camera feed", frame);
	// 	cv::waitKey(5);
	// }

	cap.release();
	cv::destroyAllWindows();






    //trackBall(frame);
    


    return 0;
}