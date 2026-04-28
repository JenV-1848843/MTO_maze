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
	cv::imshow("Camera", frame);
	cv::waitKey(0);
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

	while (true) {
		cap.grab();
		cap.retrieve(frame);

		pos = trackBall(frame);

		c = &maze.getConfig()[pos.x][pos.y];

		if (c == &maze.getConfig()[targetX][targetY]) {
			std::cout << "This is the destination\n";
			return 0;
		} else if (!c->visited) {
			std::cout << "Unreachable, no path to destination\n";
			return 0;
		}

		std::cout << "Reachable! Following path:\n";
		// std::cout << "(" << current->getX() << "," << current->getY() << ") -> ";
		double mms = maze.mmsUntilTurn(*c, &pos);
		Direction dir = maze.directionTo(c, c->next);
		std::cout << "moving " << mms << " mms to " << to_string(dir) << std::endl;
		//  to " << to_string(dir) << std::endl;
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
