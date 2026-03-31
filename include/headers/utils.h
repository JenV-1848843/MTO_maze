#include <vector>
#include "config.h"
#include "cell.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#pragma once

std::vector<cv::Point2f> orderPoints(const std::vector<cv::Point>& pts);

void readMazeConfig(
    cv::Mat frame,
    std::array<std::array<Cell, amountCellRows>, amountCellCols>& config
);

void detectHorizontalWalls(
    std::array<std::array<Cell, amountCellRows>, amountCellCols>& config,
    int boardWidth,
    int boardHeight,
    int cellWidth,
    int cellHeight,
    cv::Mat innerBoard,
    cv::Mat wallMask
);

void detectVerticalWalls(
    std::array<std::array<Cell, amountCellRows>, amountCellCols>& config,
    int boardWidth,
    int boardHeight,
    int cellWidth,
    int cellHeight,
    cv::Mat innerBoard,
    cv::Mat wallMask
);

void trackBall(cv::Mat frame);