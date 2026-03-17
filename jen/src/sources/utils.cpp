#include "../headers/utils.h"
#include "../headers/config.h"
#include "../headers/maze.h"
#include "../headers/cell.h"


// Order points of a bounding rectangle: top-left, top-right, bottom-right, bottom-left
std::vector<cv::Point2f> orderPoints(const std::vector<cv::Point>& pts)
{
    std::vector<cv::Point2f> rect(4);

    std::vector<cv::Point2f> ptsf;
    for (auto& p : pts)
        ptsf.push_back(cv::Point2f(p.x, p.y));

    // sum and diff
    std::vector<float> sum, diff;
    for (auto& p : ptsf)
    {
        sum.push_back(p.x + p.y);
        diff.push_back(p.y - p.x);
    }

    rect[0] = ptsf[min_element(sum.begin(), sum.end()) - sum.begin()]; // TL
    rect[2] = ptsf[max_element(sum.begin(), sum.end()) - sum.begin()]; // BR
    rect[1] = ptsf[min_element(diff.begin(), diff.end()) - diff.begin()]; // TR
    rect[3] = ptsf[max_element(diff.begin(), diff.end()) - diff.begin()]; // BL

    return rect;
};

void readMazeConfig(
    cv::Mat frame,
    std::array<std::array<Cell, amountCellRows>, amountCellCols>& config
) {
    cv::Mat hsv; // frame
    cv::Mat redMaskLow, redMaskHigh, redMask, orangeMask; // masks

    /*
    HSV = Hue (actual color), Saturation (intensity), Value (brightness)
    Hue is represented by a "color wheel" in which the color of a pixel can be represented by an angle on the wheel
    OpenCV normalizes these angles from 0-360 to 0-180
    */
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(redThresholdLow, 255, 255), redMaskLow); // Only red pixels are kept from the image
    cv::inRange(hsv, cv::Scalar(redThresholdHigh,100,100), cv::Scalar(180,255,255), redMaskHigh); // Only red pixels are kept from the image
    cv::inRange(hsv, cv::Scalar(orangeThresholdLow, 100, 210), cv::Scalar(orangeThresholdHigh, 255, 255), orangeMask); // Only orange pixels are kept from the image


    /*
    Bitwise or --> if pixels are classified as "red" by any of the two masks, add them to the entire mask.
    This is needed because red in the OpenCV Hue "color wheel" is a the beginning and at the end of the wheel.
    */
    redMask = redMaskLow | redMaskHigh;

    cv::Mat redPixels, orangePixels, filteredImage;

    frame.copyTo(redPixels, redMask);
    frame.copyTo(orangePixels, orangeMask);

    frame.copyTo(filteredImage, redMask);
    frame.copyTo(filteredImage, orangeMask);



    // find the contours of the red square
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(redMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
    {
        std::cout << "No contours found\n";
    }

    // assume the largest found contour is the red square
    int largestIdx = 0;
    double maxArea = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            largestIdx = i;
        }
    }

    std::vector<cv::Point> cnt = contours[largestIdx];

    // Approximate polygon
    std::vector<cv::Point> approx;
    double epsilon = 0.02 * arcLength(cnt, true);
    approxPolyDP(cnt, approx, epsilon, true);

    if (approx.size() != 4)
    {
        std::cout << "Did not find 4 corners, found: " << approx.size() << std::endl;
    }

    // Order points
    std::vector<cv::Point2f> srcPts = orderPoints(approx);

    // Destination rectangle
    int width = 400;
    int height = 400;
    std::vector<cv::Point2f> dstPts = {
        cv::Point2f(0,0),
        cv::Point2f(width-1, 0),
        cv::Point2f(width-1, height-1),
        cv::Point2f(0, height-1)
    };

    // Perspective transform
    cv::Mat M = getPerspectiveTransform(srcPts, dstPts);
    cv::Mat warped;
    warpPerspective(filteredImage, warped, M, cv::Size(width, height));

    // crop the red border
    cv::Mat borderlessHsv;
    cv::cvtColor(warped, borderlessHsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1Crop, mask2Crop, redMaskCrop;
    cv::inRange(borderlessHsv, cv::Scalar(0,100,100), cv::Scalar(10,255,255), mask1Crop);
    cv::inRange(borderlessHsv, cv::Scalar(160,100,100), cv::Scalar(180,255,255), mask2Crop);
    redMaskCrop = mask1Crop | mask2Crop;

    // clean up noise
    cv::morphologyEx(redMaskCrop, redMaskCrop, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), 2);


    std::vector<std::vector<cv::Point>> cropContours;
    cv::findContours(redMaskCrop, cropContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int largestIdxCrop = 0;
    double maxAreaCrop = 0;

    for (int i = 0; i < cropContours.size(); i++)
    {
        double cropArea = cv::contourArea(cropContours[i]);
        if (cropArea > maxAreaCrop)
        {
            maxAreaCrop = cropArea;
            largestIdxCrop = i;
        }
    }

    cv::Rect redRectCrop = cv::boundingRect(cropContours[largestIdxCrop]);


    cv::Rect innerROI(
        redRectCrop.x + cropMargin,
        redRectCrop.y + cropMargin,
        redRectCrop.width - 2*cropMargin,
        redRectCrop.height - 2*cropMargin
    );

    // safety clamp
    innerROI &= cv::Rect(0, 0, warped.cols, warped.rows);

    cv::Mat innerBoard = warped(innerROI).clone();

    int innerBoardWidth = innerBoard.size().width;
    int innerBoardHeight = innerBoard.size().height;
    int cellWidth = innerBoardWidth/8;
    int cellHeight = innerBoardHeight/8;
    
    /*
    Wall detection
    */
    cv::Mat wallHsv;
    cv::cvtColor(innerBoard, wallHsv, cv::COLOR_BGR2HSV);

    cv::Mat wallMask;
    cv::inRange(wallHsv, cv::Scalar(orangeThresholdLow, 100, 100), cv::Scalar(orangeThresholdHigh, 255, 255), wallMask);//Only orange pixels are kept from the image

    detectHorizontalWalls(
        config,
        innerBoardWidth,
        innerBoardHeight,
        cellWidth,
        cellHeight,
        innerBoard,
        wallMask
    );

    detectVerticalWalls(
        config,
        innerBoardWidth,
        innerBoardHeight,
        cellWidth,
        cellHeight,
        innerBoard,
        wallMask
    );


    #ifdef DESKTOP_BUILD
        cv::imshow("innerBoard", innerBoard);
        cv::waitKey(0);
    #endif
};

void detectHorizontalWalls(
    std::array<std::array<Cell, amountCellRows>, amountCellCols>& config,
    int boardWidth,
    int boardHeight,
    int cellWidth,
    int cellHeight,
    cv::Mat innerBoard,
    cv::Mat wallMask
) {


    for (int x = cellWidth/2; x < boardWidth; x += cellWidth) {
        for (int y = cellHeight; y < boardHeight - cellHeight; y += cellHeight) {

            int col = (x - cellWidth/2) / cellWidth;
            int row = (y / cellHeight) - 1;

            cv::Rect roi(
                x - wallDetectionRoiDimention / 2, 
                y - wallDetectionRoiDimention / 2, 
                wallDetectionRoiDimention, 
                wallDetectionRoiDimention
            );

            cv::rectangle(
                innerBoard,
                roi,
                cv::Scalar(0, 255, 0),
                1
            );

            roi &= cv::Rect(0, 0, wallMask.cols, wallMask.rows);
            cv::Mat wallMaskROI = wallMask(roi);
            int nonZero = cv::countNonZero(wallMaskROI);
            int totalMatchedPixels = wallMaskROI.rows * wallMaskROI.cols;

            if (nonZero >= 0.1 * totalMatchedPixels) {
                config[col][row].setWall(Wall::BOTTOM, true);
                config[col][row + 1].setWall(Wall::TOP, true);
            };
        }
    }
};

void detectVerticalWalls(
    std::array<std::array<Cell, amountCellRows>, amountCellCols>& config,
    int boardWidth,
    int boardHeight,
    int cellWidth,
    int cellHeight,
    cv::Mat innerBoard,
    cv::Mat wallMask
) {
    for (int x = cellWidth; x < boardWidth - cellWidth; x += cellWidth) {
        for (int y = cellHeight/2; y < boardHeight; y += cellHeight) {
            int col = (x / cellWidth) - 1;
            int row = (y - cellHeight/2) / cellHeight;

            cv::Rect roi(
                x - wallDetectionRoiDimention / 2, 
                y - wallDetectionRoiDimention / 2, 
                wallDetectionRoiDimention, 
                wallDetectionRoiDimention
            );

            cv::rectangle(
                innerBoard,
                roi,
                cv::Scalar(0, 255, 0),
                1
            );

            roi &= cv::Rect(0, 0, wallMask.cols, wallMask.rows);
            cv::Mat wallMaskROI = wallMask(roi);
            int nonZero = cv::countNonZero(wallMaskROI);
            int totalMatchedPixels = wallMaskROI.rows * wallMaskROI.cols;

            if (nonZero >= 0.1 * totalMatchedPixels) {
                config[col][row].setWall(Wall::RIGHT, true);
                config[col + 1][row].setWall(Wall::LEFT, true);
            };
        }
    }
};

void trackBall(cv::Mat frame) {
    cv::Mat hsv; // frame
    cv::Mat redMaskLow, redMaskHigh, redMask, greenMask; // masks

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(redThresholdLow, 255, 255), redMaskLow); // Only red pixels are kept from the image
    cv::inRange(hsv, cv::Scalar(redThresholdHigh,100,100), cv::Scalar(180,255,255), redMaskHigh); // Only red pixels are kept from the image
    cv::inRange(hsv, cv::Scalar(40, 100, 100), cv::Scalar(80, 255, 255), greenMask);

    redMask = redMaskLow | redMaskHigh;

    cv::Rect borderBox, ballBox;

    // Find contours
    std::vector<std::vector<cv::Point>> edgeContours;
    cv::findContours(redMask, edgeContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : edgeContours) {
        // Filter out small noise blobs
        if (cv::contourArea(contour) < 500) continue;

        cv::Rect bbox = cv::boundingRect(contour);
        borderBox = bbox;


        pixelsPerMm = bbox.height / (double)outerWallLength;
        mmPerPixel = (double)outerWallLength / bbox.height;

        // Check if bounding box is roughly square
        float aspectRatio = (float)bbox.width / bbox.height;
        if (aspectRatio > 0.8 && aspectRatio < 1.2) {
            cv::rectangle(frame, bbox, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "278 mm", cv::Point(bbox.x, bbox.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
    }

    // Find contours of the ball
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(greenMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        cv::Rect bbox = cv::boundingRect(contour);
        ballBox = bbox;
        cv::rectangle(frame, bbox, cv::Scalar(0, 255, 0), 2);
    }

    int verticalPixelDistance = std::abs(ballBox.y - borderBox.y);
    double verticalMmDistance = verticalPixelDistance * mmPerPixel;

    int horizontalPixelDistance = std::abs(ballBox.x - borderBox.x);
    double horizontalMmDistance = horizontalPixelDistance * mmPerPixel;

    // Draw a vertical line between the two points
    int x = ballBox.x + ballBox.width / 2;
    cv::line(frame, cv::Point(x, borderBox.y), cv::Point(x, ballBox.y),
        cv::Scalar(255, 0, 0), 2);

    int y = ballBox.y + ballBox.height/ 2;
    cv::line(frame, cv::Point(borderBox.x, y), cv::Point(ballBox.x, y),
        cv::Scalar(255, 0, 0), 2);

    // Label the distance
    std::string verticalLabel = std::to_string((int)verticalMmDistance) + " mm";
    cv::putText(frame, verticalLabel,
        cv::Point(x + 10, borderBox.y + verticalPixelDistance / 2),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

    std::string horizontalLabel = std::to_string((int)horizontalMmDistance) + " mm";
    cv::putText(frame, horizontalLabel,
        cv::Point(borderBox.x + horizontalPixelDistance / 2, y - 10),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

    // cv::Mat hsv;
    // cv::Mat ballMask;

    // cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // cv::inRange(hsv, cv::Scalar(40, 100, 100), cv::Scalar(80, 255, 255), ballMask);

    // // Find contours
    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(ballMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // // Draw bounding box around each contour
    // for (const auto& contour : contours) {
    //     cv::Rect bbox = cv::boundingRect(contour);
    //     cv::rectangle(frame, bbox, cv::Scalar(0, 255, 0), 2);
    // }

    cv::imshow("Result", frame);
    cv::waitKey(0);
};