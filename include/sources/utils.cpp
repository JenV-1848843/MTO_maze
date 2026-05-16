#include "../headers/utils.h"
#include "../headers/config.h"
#include "../headers/maze.h"
#include "../headers/cell.h"
#include "../headers/ballPosition.h"


// Order points of a bounding rectangle: top-left, top-right, bottom-right, bottom-left
std::vector<cv::Point2f> orderPoints(const std::vector<cv::Point>& pts)
{
    std::vector<cv::Point2f> ptsf;
    for (auto& p : pts)
        ptsf.push_back(cv::Point2f((float)p.x, (float)p.y));

    // Find centroid
    cv::Point2f center(0, 0);
    for (auto& p : ptsf) center += p;
    center *= (1.0f / ptsf.size());

    // Sort by angle from centroid: TL, TR, BR, BL (clockwise from top-left)
    std::sort(ptsf.begin(), ptsf.end(), [&center](const cv::Point2f& a, const cv::Point2f& b) {
        return std::atan2(a.y - center.y, a.x - center.x)
             < std::atan2(b.y - center.y, b.x - center.x);
    });

    // atan2 gives angles in [-pi, pi], starting from the right (east), going CCW.
    // Rotate so TL comes first: find the point closest to top-left (min x+y)
    int tlIdx = 0;
    float minSum = std::numeric_limits<float>::max();
    for (int i = 0; i < (int)ptsf.size(); i++) {
        float s = ptsf[i].x + ptsf[i].y;
        if (s < minSum) { minSum = s; tlIdx = i; }
    }

    // Rotate vector so TL is first, then order is TL, TR, BR, BL clockwise
    std::vector<cv::Point2f> rect(4);
    for (int i = 0; i < 4; i++)
        rect[i] = ptsf[(tlIdx + i) % 4];

    return rect;
}

void readMazeConfig(
    cv::Mat frame,
    std::array<std::array<Cell, amountCellRows>, amountCellCols>& config
) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // --- Red mask (for border detection) ---
    cv::Mat redMaskLow, redMaskHigh, redMask;
    cv::inRange(hsv, cv::Scalar(0, 100, 100),           cv::Scalar(redThresholdLow, 255, 255),  redMaskLow);
    cv::inRange(hsv, cv::Scalar(redThresholdHigh,100,100), cv::Scalar(180, 255, 255),           redMaskHigh);
    redMask = redMaskLow | redMaskHigh;

    // --- Orange mask (for walls) ---
    cv::Mat orangeMask;
    cv::inRange(hsv, cv::Scalar(orangeThresholdLow, 100, 210),
                     cv::Scalar(orangeThresholdHigh, 255, 255), orangeMask);

    // Use the ORIGINAL frame (not filtered) for the perspective warp —
    // this gives a much more stable and complete image to warp.
    // We still detect the border using the red mask.

    // Clean up red mask before finding contours
    cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)));

    // Find border contours in red mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(redMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        std::cout << "No contours found\n";
        return;
    }

    // Largest contour = red border
    int largestIdx = 0;
    double maxArea = 0;
    for (int i = 0; i < (int)contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea) { maxArea = area; largestIdx = i; }
    }

    std::vector<cv::Point> cnt = contours[largestIdx];

    // Approximate to polygon — try a few epsilons until we get 4 corners
    std::vector<cv::Point> approx;
    for (double epsFactor = 0.01; epsFactor <= 0.1; epsFactor += 0.005) {
        double epsilon = epsFactor * cv::arcLength(cnt, true);
        cv::approxPolyDP(cnt, approx, epsilon, true);
        if (approx.size() == 4) break;
    }

    if (approx.size() != 4) {
        std::cout << "Could not find 4 corners, found: " << approx.size() << std::endl;
        return;
    }

    std::vector<cv::Point2f> srcPts = orderPoints(approx);

    // Compute output size from the actual contour dimensions to preserve aspect ratio
    float widthA  = cv::norm(srcPts[1] - srcPts[0]);
    float widthB  = cv::norm(srcPts[2] - srcPts[3]);
    float heightA = cv::norm(srcPts[3] - srcPts[0]);
    float heightB = cv::norm(srcPts[2] - srcPts[1]);
    int warpWidth  = (int)std::max(widthA,  widthB);
    int warpHeight = (int)std::max(heightA, heightB);

    std::vector<cv::Point2f> dstPts = {
        cv::Point2f(0,            0),
        cv::Point2f(warpWidth-1,  0),
        cv::Point2f(warpWidth-1,  warpHeight-1),
        cv::Point2f(0,            warpHeight-1)
    };

    // Warp the ORIGINAL frame — avoids gaps from sparse filtered image
    cv::Mat M = cv::getPerspectiveTransform(srcPts, dstPts);
    cv::Mat warped;
    cv::warpPerspective(frame, warped, M, cv::Size(warpWidth, warpHeight));

    // --- Find and crop the red border from the warped original frame ---
    cv::Mat warpedHsv;
    cv::cvtColor(warped, warpedHsv, cv::COLOR_BGR2HSV);

    cv::Mat warpedRedLow, warpedRedHigh, warpedRedMask;
    cv::inRange(warpedHsv, cv::Scalar(0,100,100),            cv::Scalar(redThresholdLow,255,255),  warpedRedLow);
    cv::inRange(warpedHsv, cv::Scalar(redThresholdHigh,100,100), cv::Scalar(180,255,255),          warpedRedHigh);
    warpedRedMask = warpedRedLow | warpedRedHigh;

    cv::morphologyEx(warpedRedMask, warpedRedMask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7)));

    std::vector<std::vector<cv::Point>> cropContours;
    cv::findContours(warpedRedMask, cropContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (cropContours.empty()) {
        std::cout << "Could not find red border in warped image\n";
        return;
    }

    int largestIdxCrop = 0;
    double maxAreaCrop = 0;
    for (int i = 0; i < (int)cropContours.size(); i++) {
        double a = cv::contourArea(cropContours[i]);
        if (a > maxAreaCrop) { maxAreaCrop = a; largestIdxCrop = i; }
    }

    cv::Rect redRectCrop = cv::boundingRect(cropContours[largestIdxCrop]);

    cv::Rect innerROI(
        redRectCrop.x + cropMargin,
        redRectCrop.y + cropMargin,
        redRectCrop.width  - 2 * cropMargin,
        redRectCrop.height - 2 * cropMargin
    );
    innerROI &= cv::Rect(0, 0, warped.cols, warped.rows);

    cv::Mat innerBoard = warped(innerROI).clone();

    // --- Wall detection (unchanged logic, same inputs) ---
    int innerBoardWidth  = innerBoard.cols;
    int innerBoardHeight = innerBoard.rows;
    int cellWidth  = innerBoardWidth  / 8;
    int cellHeight = innerBoardHeight / 8;

    cv::Mat wallHsv;
    cv::cvtColor(innerBoard, wallHsv, cv::COLOR_BGR2HSV);

    cv::Mat wallMask;
    cv::inRange(wallHsv,
                cv::Scalar(orangeThresholdLow,  100, 100),
                cv::Scalar(orangeThresholdHigh, 255, 255),
                wallMask);

    detectHorizontalWalls(config, innerBoardWidth, innerBoardHeight,
                          cellWidth, cellHeight, innerBoard, wallMask);
    detectVerticalWalls  (config, innerBoardWidth, innerBoardHeight,
                          cellWidth, cellHeight, innerBoard, wallMask);

    cv::imshow("innerBoard", innerBoard);
}

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

BallPosition trackBall(cv::Mat frame) {
    BallPosition result{};
    result.found = false;
    cv::Mat hsv;
    cv::Mat borderMaskLow, borderMaskHigh, borderMask, ballMask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(redThresholdLow, 255, 255), borderMaskLow);
    cv::inRange(hsv, cv::Scalar(redThresholdHigh, 100, 100), cv::Scalar(180, 255, 255), borderMaskHigh);
    cv::inRange(hsv,
                cv::Scalar(0, 0, 200),    // low H, low S, high V
                cv::Scalar(180, 50, 255), // any H, low S, max V
                ballMask);
    borderMask = borderMaskLow | borderMaskHigh;

    cv::Rect borderBox, ballBox;
    bool borderFound = false;

    // Find border contours
    std::vector<std::vector<cv::Point>> edgeContours;
    cv::findContours(borderMask, edgeContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : edgeContours) {
        if (cv::contourArea(contour) < 500) continue;
        cv::Rect bbox = cv::boundingRect(contour);
        float aspectRatio = (float)bbox.width / bbox.height;
        if (aspectRatio > 0.8 && aspectRatio < 1.2) {
            borderBox = bbox;
            borderFound = true;
            pixelsPerMm = bbox.height / (double)outerWallLength;
            mmPerPixel = (double)outerWallLength / bbox.height;
            cv::rectangle(frame, bbox, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "278 mm", cv::Point(bbox.x, bbox.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
    }
    if (!borderFound) return result;

    // Crop the white mask to only search inside the border box
    cv::Mat maskedBallRegion = ballMask(borderBox);
    bool ballFound = false;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(maskedBallRegion, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 50 || area > 2000) continue;
        cv::Rect bbox = cv::boundingRect(contour);
        float aspectRatio = (float)bbox.width / bbox.height;
        if (aspectRatio < 0.6 || aspectRatio > 1.4) continue;

        // Offset back to full frame coordinates
        ballBox = cv::Rect(bbox.x + borderBox.x, bbox.y + borderBox.y, bbox.width, bbox.height);
        ballFound = true;
        cv::rectangle(frame, ballBox, cv::Scalar(0, 255, 0), 2);
    }
    if (!ballFound) return result;

    // Pixel position (center of ball)
    result.pixelX = ballBox.x + ballBox.width / 2.0f;
    result.pixelY = ballBox.y + ballBox.height / 2.0f;

    // mm distance from the top-left corner of the border box
    result.mmX = std::abs(result.pixelX - borderBox.x) * mmPerPixel;
    result.mmY = std::abs(result.pixelY - borderBox.y) * mmPerPixel;

    // convert world distances to grid coordinates
    result.x = worldToGrid(result.mmX);
    result.y = worldToGrid(result.mmY);

    result.found = true;

    // Draw measurement lines
    int x = (int)result.pixelX;
    int y = (int)result.pixelY;
    cv::line(frame, cv::Point(x, borderBox.y), cv::Point(x, ballBox.y),
        cv::Scalar(255, 0, 0), 2);
    cv::line(frame, cv::Point(borderBox.x, y), cv::Point(ballBox.x, y),
        cv::Scalar(255, 0, 0), 2);

    // Label distances
    std::string verticalLabel = std::to_string((int)result.mmY) + " mm";
    cv::putText(frame, verticalLabel,
        cv::Point(x + 10, borderBox.y + (int)(result.mmY / mmPerPixel) / 2),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
    std::string horizontalLabel = std::to_string((int)result.mmX) + " mm";
    cv::putText(frame, horizontalLabel,
        cv::Point(borderBox.x + (int)(result.mmX / mmPerPixel) / 2, y - 10),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
    std::string positionLabel = "(" + std::to_string(result.x) + ", " + std::to_string(result.y) + ")";
    cv::putText(frame, positionLabel,
        cv::Point(borderBox.x + 50, borderBox.y + 50),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

    cv::imshow("Result", frame);
    cv::waitKey(5);

    return result;
};

int worldToGrid(double worldDimention) {
    double cellWidth = (double)outerWallLength/8;
    for (int i = 0; i < 8; i++) {
        if (worldDimention <= cellWidth * (i + 1)) return i;
    }
    return 7;
};