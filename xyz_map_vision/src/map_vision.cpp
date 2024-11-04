#include <string>
#include <sstream>
#include "../include/xyz_map_vision/map_vision.hpp"

MapVision::MapVision()
{


}

std::vector<cv::Point> MapVision::initializeROI(const cv::Mat &img)
{

    cv::Mat gray, edges;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::Canny(gray, edges, 50, 150);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double largestArea = 0.0;
    const double aspectRatioThreshold = 0.1;

    for (const auto &contour : contours)
    {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, cv::arcLength(contour, true) * 0.02, true);

        if (approx.size() == 4 && cv::isContourConvex(approx))
        {
            cv::Rect boundingBox = cv::boundingRect(approx);
            double aspectRatio = std::abs(1.0 - (double)boundingBox.width / (double)boundingBox.height);

            if (aspectRatio < aspectRatioThreshold)
            {
                double area = cv::contourArea(approx);

                if (area > largestArea)
                {
                    largestArea = area;
                    largestSquare = approx;
                }
            }
        }
    }

    if (largestSquare.empty())
    {
        std::cerr << "Can not find a square-like ROI!" << std::endl;
        return {};
    }
    else
    {
        return largestSquare;
    }
}

void MapVision::drawCells(cv::Mat &image)
{
    int imgSize = image.cols;
    int BigSquareSize = imgSize * 0.19;
    int remainingSpace = imgSize - 2 * BigSquareSize;
    int middleSquareSize = remainingSpace / 4;
    int squareNumber = 0;

    // 1. Draw right edge (오른쪽 아래에서 위로 올라가는 순서로)
    for (int i = 5; i >= 0; i--)
    {
        int y1 = (i == 0) ? 0 : (i == 5) ? imgSize - BigSquareSize
                                         : BigSquareSize + (i - 1) * middleSquareSize;
        int y2 = y1 + ((i == 0 || i == 5) ? BigSquareSize : middleSquareSize);
        cv::rectangle(image, cv::Point(imgSize - BigSquareSize, y1), cv::Point(imgSize, y2), cv::Scalar(0, 255, 0), 2);
        if (i != 0)
            squares.push_back({imgSize - BigSquareSize, y1, imgSize, y2, squareNumber++});
    }

    // 2. Draw top edge (오른쪽에서 왼쪽으로)
    for (int i = 5; i >= 0; i--)
    {
        int x1 = (i == 0) ? 0 : (i == 5) ? imgSize - BigSquareSize
                                         : BigSquareSize + (i - 1) * middleSquareSize;
        int x2 = x1 + ((i == 0 || i == 5) ? BigSquareSize : middleSquareSize);
        cv::rectangle(image, cv::Point(x1, 0), cv::Point(x2, BigSquareSize), cv::Scalar(0, 255, 0), 2);
        if (i != 0)
            squares.push_back({x1, 0, x2, BigSquareSize, squareNumber++});
    }

    // 3. Draw left edge (위에서 아래로)
    for (int i = 0; i < 6; i++)
    {
        int y1 = (i == 0) ? 0 : (i == 5) ? imgSize - BigSquareSize
                                         : BigSquareSize + (i - 1) * middleSquareSize;
        int y2 = y1 + ((i == 0 || i == 5) ? BigSquareSize : middleSquareSize);
        cv::rectangle(image, cv::Point(0, y1), cv::Point(BigSquareSize, y2), cv::Scalar(0, 255, 0), 2);
        squares.push_back({0, y1, BigSquareSize, y2, squareNumber++});
    }

    // 4. Draw bottom edge (왼쪽에서 오른쪽으로)
    for (int i = 0; i < 6; i++)
    {
        int x1 = (i == 0) ? 0 : (i == 5) ? imgSize - BigSquareSize
                                         : BigSquareSize + (i - 1) * middleSquareSize;
        int x2 = x1 + ((i == 0 || i == 5) ? BigSquareSize : middleSquareSize);
        cv::rectangle(image, cv::Point(x1, imgSize - BigSquareSize), cv::Point(x2, imgSize), cv::Scalar(0, 255, 0), 2);
        if (i != 0 && i != 5)
            squares.push_back({x1, imgSize - BigSquareSize, x2, imgSize, squareNumber++});
    }

    // Draw center square
    int centerX = imgSize / 2;
    int centerY = imgSize / 2;
    cv::rectangle(image, cv::Point(centerX - BigSquareSize / 2, centerY - BigSquareSize / 2),
                  cv::Point(centerX + BigSquareSize / 2, centerY + BigSquareSize / 2), cv::Scalar(0, 255, 0), 2);
    squares.push_back({centerX - BigSquareSize / 2, centerY - BigSquareSize / 2, centerX + BigSquareSize / 2, centerY + BigSquareSize / 2, squareNumber++});

    // Draw diagonal squares
    int diagonalSquareSize = ((imgSize / 2) - (BigSquareSize / 2) - (BigSquareSize)) / 2;
    int directions[4][2] = {
        {1, 1},
        {-1, 1},
        {1, -1},
        {-1, -1}};

    for (int i = 0; i < 4; i++)
    {
        int startX = (directions[i][0] == 1) ? BigSquareSize : imgSize - BigSquareSize - diagonalSquareSize;
        int startY = (directions[i][1] == 1) ? BigSquareSize : imgSize - BigSquareSize - diagonalSquareSize;

        cv::rectangle(image, cv::Point(startX, startY),
                      cv::Point(startX + diagonalSquareSize, startY + diagonalSquareSize), cv::Scalar(0, 255, 0), 2);
        squares.push_back({startX, startY, startX + diagonalSquareSize, startY + diagonalSquareSize, squareNumber++});

        startX += directions[i][0] * diagonalSquareSize;
        startY += directions[i][1] * diagonalSquareSize;

        cv::rectangle(image, cv::Point(startX, startY),
                      cv::Point(startX + diagonalSquareSize, startY + diagonalSquareSize), cv::Scalar(0, 255, 0), 2);
        squares.push_back({startX, startY, startX + diagonalSquareSize, startY + diagonalSquareSize, squareNumber++});
    }

    for (const Square &square : squares)
    {
        int textX = (square.x1 + square.x2) / 2;
        int textY = (square.y1 + square.y2) / 2;
        std::string text = std::to_string(square.number);
        cv::putText(image, text, cv::Point(textX - 10, textY + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }

    cv::imshow("Yutnori Board Cells", image);
    cv::waitKey(1);
}

Player MapVision::processImage(const cv::Mat &raw_img, const cv::Mat &roi_img, const cv::Mat &depth_img,
                               sensor_msgs::msg::CameraInfo::SharedPtr camera_info, const cv::Mat &perspective_matrix, cv::Mat &output_img, const int hsvValues[6], const int player_num_)
{
    original_img = raw_img.clone();
    output_img = roi_img.clone();
    camera_info_ = camera_info;
    perspective_matrix_ = perspective_matrix;
    player_num = player_num_;

    if (player_num == 1)
    {
        p1.positions.clear();
        p1.tokens.clear();
    }
    else if (player_num == 2)
    {
        p2.positions.clear();
        p2.tokens.clear();
    }

    cv::Mat color_mask = extractColorRegion(roi_img, hsvValues);

    processContours(output_img, depth_img, color_mask);
    cv::imshow("Output Image", output_img);



    cv::waitKey(1);

    if (player_num == 1)
    {
        return p1;
    }
    else if (player_num == 2)
    {
        return p2;
    }
    else
    {
        Player default_player;
        default_player.positions = std::vector<int>(3, -1);
        default_player.tokens = std::vector<int>(3, -1);
        return default_player;
    }
}

cv::Mat MapVision::extractColorRegion(const cv::Mat &img, const int hsvValues[6])
{
    cv::Scalar lowerHSV(hsvValues[3], hsvValues[4], hsvValues[5]);
    cv::Scalar upperHSV(hsvValues[0], hsvValues[1], hsvValues[2]);

    cv::Mat hsvImg;
    cv::cvtColor(img, hsvImg, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsvImg, lowerHSV, upperHSV, mask);

    cv::medianBlur(mask, mask, 9);
    cv::GaussianBlur(mask, mask, cv::Size(5, 5), 0);

    cv::imshow("mask", mask);
    cv::waitKey(1);

    return mask;
}

void MapVision::processContours(cv::Mat &output_img, const cv::Mat &depth_img, const cv::Mat &color_mask)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(color_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::set<int> uniquePositions;

    for (const auto &contour : contours)
    {
        //contour size 600
        if (cv::contourArea(contour) > 300)
        {
            cv::Rect boundingBox = cv::boundingRect(contour);
            cv::Point centerPoint(boundingBox.x + boundingBox.width / 2,
                                  boundingBox.y + boundingBox.height / 2);

            int location = findPlayerLocation(centerPoint);
            cv::Rect convertedRect = rectConverter(boundingBox, perspective_matrix_);
            int tokens = depthDetect(depth_img, convertedRect);

            if (location != -1 && uniquePositions.find(location) == uniquePositions.end())
            {
                uniquePositions.insert(location);

                if (player_num == 1)
                {
                    if (std::find(p1.positions.begin(), p1.positions.end(), location) == p1.positions.end())
                    {
                        p1.positions.push_back(location);
                        p1.tokens.push_back(tokens);
                    }
                }
                else if (player_num == 2)
                {
                    if (std::find(p2.positions.begin(), p2.positions.end(), location) == p2.positions.end())
                    {
                        p2.positions.push_back(location);
                        p2.tokens.push_back(tokens);
                    }
                }
                else
                {
                    return;
                }
            }

            cv::rectangle(output_img, boundingBox, cv::Scalar(0, 0, 255), 2);
            cv::circle(output_img, centerPoint, 5, cv::Scalar(255, 0, 0), -1);
        }
    }

    if (player_num == 1 && p1.positions.size() > 0)
    {
        std::cout << "Player 1's positions and tokens:" << std::endl;
        if (!p1.positions.empty())
        {
            for (size_t i = 0; i < p1.positions.size(); ++i)
            {
                std::cout << "Position: " << p1.positions[i] << ", Tokens: " << p1.tokens[i] << std::endl;
            }
            std::cout << std::endl;
        }
    }
    else if (player_num == 2 && p2.positions.size() > 0)
    {
        std::cout << "Player 2's positions and tokens:" << std::endl;
        if (!p2.positions.empty())
        {
            for (size_t i = 0; i < p2.positions.size(); ++i)
            {
                std::cout << "Position: " << p2.positions[i] << ", Tokens: " << p2.tokens[i] << std::endl;
            }
            std::cout << std::endl;
        }
    }
}

int MapVision::findPlayerLocation(const cv::Point &point)
{
    for (const Square &square : squares)
    {
        if (point.x >= square.x1 && point.x <= square.x2 && point.y >= square.y1 && point.y <= square.y2)
        {
            return square.number;
        }
    }
    return -1;
}

int MapVision::depthDetect(const cv::Mat &image, const cv::Rect &rect)
{
    intrinsic_.width = camera_info_->width;
    intrinsic_.height = camera_info_->height;
    intrinsic_.ppx = camera_info_->k[2];
    intrinsic_.ppy = camera_info_->k[5];
    intrinsic_.fx = camera_info_->k[0];
    intrinsic_.fy = camera_info_->k[4];
    intrinsic_.model = RS2_DISTORTION_NONE;

    for (int i = 0; i < 5; ++i)
    {
        intrinsic_.coeffs[i] = camera_info_->d[i];
    }

    if (image.empty())
    {
        std::cout << "Image empty!" << std::endl;
        return -1;
    }

    int min_depth_in_mm = INT_MAX;

    for (int y = rect.y; y < rect.y + rect.height; ++y)
    {
        for (int x = rect.x; x < rect.x + rect.width; ++x)
        {
            if (x >= 0 && x < image.cols && y >= 0 && y < image.rows)
            {
                int depth_in_mm = image.at<short int>(cv::Point(x, y));

                if (depth_in_mm > 0 && depth_in_mm < min_depth_in_mm)
                {
                    min_depth_in_mm = depth_in_mm;
                }
            }
        }
    }

    if (min_depth_in_mm == INT_MAX)
    {
        std::cout << "No valid depth values found!" << std::endl;
        return -1;
    }
    else
    {
        // 675 -> 1
        // 665 -> 2
        // 655 -> 3
        // 642 -> 4
        if (min_depth_in_mm < depth_4)
            return 4;
        else if (min_depth_in_mm < depth_4 + 10)
            return 3;
        else if (min_depth_in_mm < depth_4 + 20)
            return 2;
        else if (min_depth_in_mm < depth_4 + 30)
            return 1;
        else
            return -1;
    }
}

cv::Rect MapVision::rectConverter(const cv::Rect &roi_rect, const cv::Mat &perspective_matrix)
{
    cv::Mat inverse_matrix;
    cv::invert(perspective_matrix, inverse_matrix);

    // 변환된 좌표를 저장할 벡터
    std::vector<cv::Point2f> converted_points;

    // roi_rect의 꼭짓점들을 변환
    std::vector<cv::Point2f> roi_points = {
        cv::Point2f(roi_rect.x, roi_rect.y),                                    // 좌상
        cv::Point2f(roi_rect.x + roi_rect.width, roi_rect.y),                   // 우상
        cv::Point2f(roi_rect.x + roi_rect.width, roi_rect.y + roi_rect.height), // 우하
        cv::Point2f(roi_rect.x, roi_rect.y + roi_rect.height)                   // 좌하
    };

    for (const auto &point : roi_points)
    {
        cv::Mat roi_point_mat = (cv::Mat_<double>(3, 1) << point.x, point.y, 1.0);
        cv::Mat original_point_mat = inverse_matrix * roi_point_mat;

        float x = original_point_mat.at<double>(0, 0) / original_point_mat.at<double>(2, 0);
        float y = original_point_mat.at<double>(1, 0) / original_point_mat.at<double>(2, 0);

        converted_points.emplace_back(x, y);
    }

    // 변환된 좌표로 새로운 Rect 생성
    int x_min = static_cast<int>(std::floor(std::min({converted_points[0].x, converted_points[1].x, converted_points[2].x, converted_points[3].x})));
    int y_min = static_cast<int>(std::floor(std::min({converted_points[0].y, converted_points[1].y, converted_points[2].y, converted_points[3].y})));
    int x_max = static_cast<int>(std::ceil(std::max({converted_points[0].x, converted_points[1].x, converted_points[2].x, converted_points[3].x})));
    int y_max = static_cast<int>(std::ceil(std::max({converted_points[0].y, converted_points[1].y, converted_points[2].y, converted_points[3].y})));

    return cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
}
