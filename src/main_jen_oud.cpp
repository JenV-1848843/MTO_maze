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

#ifdef SERVO_CONTROL_ENABLED
#include "../include/motorcontrol.h"
#include "../include/SystemConfig.hpp"
#endif

static int angle_to_pulse(int angle) {
	return 1000 + (angle * 1000) / 180;
}

// PID tuning — adjust these during testing
static constexpr float PID_KP = 0.5f;
static constexpr float PID_KI = 0.0f;
static constexpr float PID_KD = 0.1f;
static constexpr float PID_OUTPUT_LIMIT = 30.0f;

int main(int argc, char* argv[]) {
	int targetX, targetY;

	std::cout << "Enter target X: ";
	std::cin >> targetX;
	std::cout << "Enter target Y: ";
	std::cin >> targetY;

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

	#ifdef SERVO_CONTROL_ENABLED
	Init_GPIO_Control();
	Init_ServoMotor_Control();
	ServoMotor_Control(2, angle_to_pulse(0)); // neutral
	ServoMotor_Control(3, angle_to_pulse(0));

	PIDController pid_x, pid_y;
	pid_x.kp = pid_y.kp = PID_KP;
	pid_x.ki = pid_y.ki = PID_KI;
	pid_x.kd = pid_y.kd = PID_KD;
	pid_x.output_limit = pid_y.output_limit = PID_OUTPUT_LIMIT;
	#endif

	std::this_thread::sleep_for(std::chrono::milliseconds(500));

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
	// cv::imshow("Camera", frame);
	// cv::waitKey(0);
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

		pos = trackBall(frame);

		c = &maze.getConfig()[pos.x][pos.y];

		if (c == &maze.getConfig()[targetX][targetY]) {
			std::cout << "This is the destination\n";
			#ifdef SERVO_CONTROL_ENABLED
			ServoMotor_Control(2, angle_to_pulse(0));
			ServoMotor_Control(3, angle_to_pulse(0));
			Exit_Motor_Control();
			#endif
			return 0;
		} else if (!c->visited) {
			std::cout << "Unreachable, no path to destination\n";
			#ifdef SERVO_CONTROL_ENABLED
			ServoMotor_Control(2, angle_to_pulse(0));
			ServoMotor_Control(3, angle_to_pulse(0));
			Exit_Motor_Control();
			#endif
			return 0;
		}

		std::cout << "Reachable! Following path:\n";
		// std::cout << "(" << current->getX() << "," << current->getY() << ") -> ";
		double mms = maze.mmsUntilTurn(*c, &pos);
		Direction dir = maze.directionTo(c, c->next);
		std::cout << "moving " << mms << " mms to " << to_string(dir) << std::endl;

		#ifdef SERVO_CONTROL_ENABLED
		if (pos.found && c->next != nullptr) {
			// Target: centre of the next waypoint cell in mm
			float next_mm_x = (c->next->getX() + 0.5f) * innerWallLength;
			float next_mm_y = (c->next->getY() + 0.5f) * innerWallLength;

			// Error = setpoint - measurement (positive error → tilt toward target)
			float error_x = next_mm_x - static_cast<float>(pos.mmX);
			float error_y = next_mm_y - static_cast<float>(pos.mmY);

			// Measure actual elapsed time in ms for correct PID integration
			auto now = std::chrono::steady_clock::now();
			int dt_ms = static_cast<int>(
				std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count()
			);
			if (dt_ms < 1) dt_ms = 1;
			last_time = now;

			float out_x = pid_x.calculate(error_x, dt_ms);
			float out_y = pid_y.calculate(error_y, dt_ms);

			ServoMotor_Control(2, angle_to_pulse(static_cast<int>(out_x)));
			ServoMotor_Control(3, angle_to_pulse(static_cast<int>(out_y)));
		}
		#endif
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
