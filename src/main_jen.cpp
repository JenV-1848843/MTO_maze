#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// for color conversions
#include <opencv2/opencv.hpp>

#include "../include/headers/config.h"
#include "../include/headers/utils.h"
#include "../include/headers/cell.h"
#include "../include/headers/maze.h"
#include "../include/headers/position.h"
#include "../include/headers/wall.h"



int main(int argc, char* argv[]) {
    #ifndef RPI_BUILD
        std::cout << "Hello\n";
    #endif

    Maze maze;
    maze.print();

    std::string imagePathRelative = "../include/images/";
    std::string imagePath = imagePathRelative + "maze_5.jpg";
    cv::Mat frame = cv::imread(imagePath, cv::IMREAD_COLOR);
    cv::resize(frame, frame, cv::Size(), 0.3, 0.3);

    // readMazeConfig(frame, maze.getConfig());

    // maze.print();

    // maze.bfs(maze.getConfig()[7][2]);

    // Cell* c = &maze.getConfig()[0][0];

    // #ifndef RPI_BUILD
    //     if (c == &maze.getConfig()[7][2]) {
    //         std::cout << "This is the destination\n";
    //     } else if (!c->visited) {
    //         std::cout << "Unreachable, no path to destination\n";
    //     } else {
    //         std::cout << "Reachable! Following path:\n";
    //         Cell* current = c;
    //         while (current != nullptr) {
    //             std::cout << "(" << current->getX() << "," << current->getY() << ") -> ";
    //             current = current->next;
    //         }
    //         std::cout << "DONE\n";
    //     }
    // #endif

    trackBall(frame);

    return 0;
}