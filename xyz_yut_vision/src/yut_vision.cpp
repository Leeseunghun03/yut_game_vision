#include <string>
#include <sstream>
#include "../include/xyz_yut_vision/yut_vision.hpp"

using namespace cv;
using namespace std;

#include <opencv2/opencv.hpp>
#include <vector>

#include <opencv2/opencv.hpp>
#include <vector>

#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

void YutVision::convertToWorldCoordinates(const Point2f &markerCenterPixel, const Vec3d &markerCenterWorld,
                                          const Point2f &targetPixel, Vec3d &targetWorld,
                                          float markerLengthInWorld, double yaw_angle, float markerLengthInPixels)
{
  float pixelDistanceX = targetPixel.x - markerCenterPixel.x;

  // Y Axis: Camera Frame Marker Frame Diffrent
  float pixelDistanceY = -(targetPixel.y - markerCenterPixel.y);

  Mat R_yaw = (Mat_<double>(2, 2) << cos(yaw_angle), -sin(yaw_angle),
               sin(yaw_angle), cos(yaw_angle));

  Mat point_cam = (Mat_<double>(2, 1) << pixelDistanceX, pixelDistanceY);
  Mat rotated_point_cam = R_yaw * point_cam;

  float scaleX = markerLengthInWorld / markerLengthInPixels;
  float scaleY = markerLengthInWorld / markerLengthInPixels;

  targetWorld[0] = markerCenterWorld[0] + rotated_point_cam.at<double>(0, 0) * scaleX;
  targetWorld[1] = markerCenterWorld[1] + rotated_point_cam.at<double>(1, 0) * scaleY;
  targetWorld[2] = markerCenterWorld[2]; // z축은 고정 (평면 상의 좌표 변환만 고려)

  // cout << "Target pixel: (" << targetPixel.x << ", " << targetPixel.y << ") -> "
  //      << "World coordinates (mm): (" << targetWorld[0] << ", " << targetWorld[1] << ", " << targetWorld[2] << ")" << endl;
}

void YutVision::calculateArucoMarkerProperties(Mat &image, Point2f &markerCenter, float &markerSideLength)
{
  // Aruco 딕셔너리 초기화
  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
  vector<int> markerIds;
  vector<vector<Point2f>> markerCorners;

  // 아루코 마커 검출
  aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

  if (markerIds.empty())
  {
    // cout << "Aruco marker not found!" << endl;
    return;
  }

  // 첫 번째 아루코 마커를 기준으로 중심 좌표와 한 변의 길이를 계산
  vector<Point2f> corners = markerCorners[0];

  // 마커의 중심 좌표 계산
  markerCenter = (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;

  // 마커 한 변의 길이 계산 (평균값)
  float side1 = norm(corners[0] - corners[1]);
  float side2 = norm(corners[1] - corners[2]);
  float side3 = norm(corners[2] - corners[3]);
  float side4 = norm(corners[3] - corners[0]);

  markerSideLength = (side1 + side2 + side3 + side4) / 4.0f;

  cout << "Marker Center: " << markerCenter << endl;
  cout << "Marker Side Length: " << markerSideLength << " pixels" << endl;

  // 아루코 마커와 축 시각화
  aruco::drawDetectedMarkers(image, markerCorners, markerIds);
  imshow("Aruco Marker Detection", image);
  waitKey(1);
}

void YutVision::detectArucoAndTransformPoints(Mat &image, std::vector<Point2f> &imagePoints, const Vec3d &markerActualPosition,
                                              const Mat &cameraMatrix, const Mat &distCoeffs, float markerLength)
{
  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
  vector<int> markerIds;
  vector<vector<Point2f>> markerCorners;
  aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

  if (markerIds.empty())
  {
    cout << "Aruco marker not found!" << endl;
    return;
  }

  vector<Vec3d> rvecs, tvecs;
  aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

  Vec3d rvec = rvecs[0];

  // 여기서 tvec을 추정된 값이 아니라 실제 값으로 설정
  Vec3d tvec = markerActualPosition; // 실제 마커의 위치 (mm 단위로 이미 알고 있는 값)

  // 회전 행렬 계산
  Mat R;
  Rodrigues(rvec, R);
  double yaw_angle = 0.0349;

  Mat R_yaw = (Mat_<double>(2, 2) << cos(yaw_angle), -sin(yaw_angle),
               sin(yaw_angle), cos(yaw_angle));

  for (auto &point : imagePoints)
  {
    vector<Point2f> imagePointsVec{point};
    vector<Point2f> undistortedPoints;
    undistortPoints(imagePointsVec, undistortedPoints, cameraMatrix, distCoeffs);

    Mat point_cam = (Mat_<double>(2, 1) << undistortedPoints[0].x, undistortedPoints[0].y);

    Mat point_world = R_yaw * point_cam;

    point.x = tvec[0] + point_world.at<double>(0, 0);
    point.y = tvec[1] + point_world.at<double>(1, 0);

    cout << "Image point: " << imagePointsVec[0] << " -> World point (mm): " << point.x << ", " << point.y << endl;
  }

  aruco::drawAxis(image, cameraMatrix, distCoeffs, rvec, tvec, markerLength);
  imshow("Aruco Detection", image);
  waitKey(1);
}

void YutVision::transformToOriginal(float x_in_resized, float y_in_resized, float &x_out, float &y_out, const cv::Mat &perspectiveMatrix, const cv::Size &original_roi_size)
{
  // Step 1: 300x300으로 리사이즈된 이미지에서의 좌표를 원래 ROI 이미지 내의 좌표로 변환
  float scale_x = static_cast<float>(original_roi_size.width) / 300.0;
  float scale_y = static_cast<float>(original_roi_size.height) / 300.0;

  // 원래 ROI 이미지 내의 좌표
  cv::Point2f point_in_roi;
  point_in_roi.x = x_in_resized * scale_x;
  point_in_roi.y = y_in_resized * scale_y;

  // Step 2: ROI 이미지 내의 좌표를 원본 이미지 내의 좌표로 변환하기 위해 변환 행렬의 역행렬 적용
  cv::Mat inverse_perspective_matrix;
  cv::invert(perspectiveMatrix, inverse_perspective_matrix);

  // 1x3 행렬로 변환 후 역행렬 적용
  std::vector<cv::Point2f> point_in_vector = {point_in_roi};
  std::vector<cv::Point2f> point_in_original(1);
  cv::perspectiveTransform(point_in_vector, point_in_original, inverse_perspective_matrix);

  // 원본 이미지 내의 좌표를 출력 매개변수에 저장
  x_out = point_in_original[0].x;
  y_out = point_in_original[0].y;
}

void YutVision::changeToBinary(cv::Mat &input_img, cv::Mat &output_img, int hsv_low[], int hsv_high[])
{
  if (input_img.empty())
  {
    return;
  }

  cv::Mat hsv_img;
  cv::cvtColor(input_img, hsv_img, cv::COLOR_BGR2HSV);
  cv::medianBlur(hsv_img, hsv_img, 9);
  cv::GaussianBlur(hsv_img, hsv_img, cv::Size(15, 15), 2.0);

  cv::Scalar lower_bound(hsv_low[0], hsv_low[1], hsv_low[2]);
  cv::Scalar upper_bound(hsv_high[0], hsv_high[1], hsv_high[2]);
  cv::inRange(hsv_img, lower_bound, upper_bound, output_img);
  cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
  cv::morphologyEx(output_img, output_img, cv::MORPH_OPEN, element);
  cv::morphologyEx(output_img, output_img, cv::MORPH_CLOSE, element);
  cv::resize(output_img, output_img, cv::Size(320, 240), 0, 0, cv::INTER_LINEAR);
}

cv::Mat YutVision::changeToBinary(cv::Mat &input_img)
{
  cv::Mat gray_img, thresh, edge;
  cv::cvtColor(input_img, gray_img, cv::COLOR_BGR2GRAY);
  cv::medianBlur(gray_img, gray_img, 9);
  cv::GaussianBlur(gray_img, gray_img, cv::Size(15, 15), 0);
  imshow("gray", gray_img);
  cv::waitKey(1);
  threshold(gray_img, thresh, 120, 255, THRESH_BINARY);
  imshow("binary", thresh);
  cv::waitKey(1);

  vector<Vec4i> lines;

  HoughLinesP(edge, lines, 1, CV_PI / 180, 50, 50, 10);

  for (size_t i = 0; i < lines.size(); i++)
  {
    Vec4i l = lines[i];
    line(input_img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 2, LINE_AA);
  }

  imshow("input_img", input_img);
  cv::waitKey(1);

  return thresh;
}

cv::Mat YutVision::extractColorRegion(const cv::Mat &img, cv::Mat &output_mask, const int hsvValues[6])
{
  cv::Scalar lowerHSV(hsvValues[3], hsvValues[4], hsvValues[5]);
  cv::Scalar upperHSV(hsvValues[0], hsvValues[1], hsvValues[2]);

  cv::Mat hsvImg;
  cv::cvtColor(img, hsvImg, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsvImg, lowerHSV, upperHSV, mask);
  cv::medianBlur(mask, mask, 9);
  cv::GaussianBlur(mask, mask, cv::Size(5, 5), 0);
  output_mask = mask.clone();

  cv::Mat result;
  cv::bitwise_and(img, img, result, mask);

  return result;
}

bool YutVision::isRegionEnough(const cv::Mat &colorRegion, int thresholdArea, bool screen)
{
  cv::Mat gray;
  if (colorRegion.channels() == 3)
  {
    cv::cvtColor(colorRegion, gray, cv::COLOR_BGR2GRAY);
  }
  else
  {
    gray = colorRegion;
  }

  cv::Mat binary;
  cv::threshold(gray, binary, 1, 255, cv::THRESH_BINARY);
  int nonZeroCount = cv::countNonZero(binary);

  if (screen)
  {
    cout << "Size: " << nonZeroCount << endl;
  }

  return nonZeroCount >= thresholdArea;
}

bool YutVision::JudgeProcess(const cv::Mat &img, cv::Mat &output_img, cv::Mat &output_mask, const int hsvValues[6], int threshold)
{
  output_img = img.clone();
  Mat color_region = extractColorRegion(img, output_mask, hsvValues);
  bool isRegion = isRegionEnough(color_region, threshold, false);
  output_img = color_region.clone();
  return isRegion;
}

int YutVision::CalProcess(const cv::Mat &img)
{
  cv::Mat gray;
  if (img.channels() == 3)
  {
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  }
  else
  {
    gray = img;
  }

  cv::Mat binary;
  cv::threshold(gray, binary, 1, 255, cv::THRESH_BINARY);

  // cv::Mat connected_regions = labelAndConnectComponents(binary, 200);
  // imshow("Connected Regions", connected_regions);
  // cv::waitKey(1);

  return 1;
}

int YutVision::countFrontYutNum(const cv::Mat &img, const int hsvValues[6], int thresholdArea)
{
  Mat hsv_img;
  cv::medianBlur(img, hsv_img, 9);
  cv::GaussianBlur(hsv_img, hsv_img, cv::Size(15, 15), 2.0);

  cv::Scalar lower_bound(hsvValues[3], hsvValues[4], hsvValues[5]);
  cv::Scalar upper_bound(hsvValues[0], hsvValues[1], hsvValues[2]);
  Mat output_mask;
  cv::inRange(hsv_img, lower_bound, upper_bound, output_mask);

  cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(output_mask, output_mask, cv::MORPH_OPEN, element);
  cv::morphologyEx(output_mask, output_mask, cv::MORPH_CLOSE, element);

  Mat front_yut_region = extractColorRegion(img, output_mask, hsvValues);

  cv::Mat gray;
  if (front_yut_region.channels() == 3)
  {
    cv::cvtColor(front_yut_region, gray, cv::COLOR_BGR2GRAY);
  }
  else
  {
    gray = front_yut_region;
  }

  cv::Mat binary;
  cv::threshold(gray, binary, 1, 255, cv::THRESH_BINARY);
  int nonZeroCount = cv::countNonZero(binary);

  int front_yut_num = nonZeroCount / thresholdArea;

  return front_yut_num;
}

std::vector<std::array<float, 4>> YutVision::detectYutFront(const cv::Mat &img, const int hsvValues[6], int back_flip, int yut_pixel_size, bool &isRegion, int current_yut_num)
{
  Mat hsv_img;
  cv::medianBlur(img, hsv_img, 9);
  cv::GaussianBlur(hsv_img, hsv_img, cv::Size(15, 15), 2.0);

  cv::Scalar lower_bound(hsvValues[3], hsvValues[4], hsvValues[5]);
  cv::Scalar upper_bound(hsvValues[0], hsvValues[1], hsvValues[2]);
  Mat output_mask;
  cv::inRange(hsv_img, lower_bound, upper_bound, output_mask);

  cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(output_mask, output_mask, cv::MORPH_OPEN, element);
  cv::morphologyEx(output_mask, output_mask, cv::MORPH_CLOSE, element);

  Mat front_yut_region = extractColorRegion(img, output_mask, hsvValues);

  int threshold = (4 - back_flip) * yut_pixel_size;
  isRegion = isRegionEnough(front_yut_region, threshold, false);

  Mat gray_front_yut_region;
  cv::cvtColor(front_yut_region, gray_front_yut_region, cv::COLOR_BGR2GRAY);
  Mat binary_front_yut_region;
  cv::threshold(gray_front_yut_region, binary_front_yut_region, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  int targetCount = current_yut_num - back_flip;

  if (targetCount > 0)
  {
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(binary_front_yut_region, lines, 1, CV_PI / 180, 50, 50, 10);

    std::vector<std::array<float, 4>> pos_vector;
    if (lines.size() < targetCount)
    {
      std::cerr << "Error: Could not find enough objects. Found " << lines.size() << " instead of " << targetCount << "." << std::endl;
      return pos_vector;
    }
    else
    {
      Mat labeled_image;
      cv::cvtColor(binary_front_yut_region, labeled_image, cv::COLOR_GRAY2BGR);

      std::vector<cv::Vec4i> unique_lines;
      for (const auto &line : lines)
      {
        bool isUnique = true;
        for (const auto &unique_line : unique_lines)
        {
          double distance = std::sqrt(std::pow(line[0] - unique_line[0], 2) + std::pow(line[1] - unique_line[1], 2));
          if (distance < 20)
          {
            isUnique = false;
            break;
          }
        }
        if (isUnique)
        {
          unique_lines.push_back(line);
        }
      }

      std::sort(unique_lines.begin(), unique_lines.end(), [](const cv::Vec4i &a, const cv::Vec4i &b)
                {
    double length_a = std::sqrt(std::pow(a[2] - a[0], 2) + std::pow(a[3] - a[1], 2));
    double length_b = std::sqrt(std::pow(b[2] - b[0], 2) + std::pow(b[3] - b[1], 2));
    return length_a > length_b; });

      for (size_t i = 0; i < targetCount && i < unique_lines.size(); ++i)
      {
        cv::Vec4i l = unique_lines[i];
        cv::line(labeled_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 0), 2);

        cv::Point2f center((l[0] + l[2]) / 2.0f, (l[1] + l[3]) / 2.0f);

        double angle = calAng_Vertical(cv::Point(l[0], l[1]), cv::Point(l[2], l[3]));

        double depth = depthDetect(img, center);
        pos_vector.push_back({center.x, center.y, static_cast<float>(depth), static_cast<float>(angle)});
        // std::cout << "Object " << i + 1 << ": Center = (" << center.x << ", " << center.y << "), Angle = " << angle << " degrees" << std::endl;
      }

      imshow("labeled_front_yut", labeled_image);
      cv::waitKey(1);
      // cout << "pos_vector_size: " << pos_vector.size() << endl;
      return pos_vector;
    }
  }
  else
  {
    cout << "More than 4!" << endl;
    return {};
  }
}

std::array<float, 4> YutVision::labelAndConnectComponents(const cv::Mat &binaryImg, cv::Mat &output_img, int minSize, bool &priority)
{
  output_img = binaryImg.clone();

  cv::Mat labels;
  int numLabels = cv::connectedComponents(binaryImg, labels);

  std::vector<cv::Point> centroids(numLabels, cv::Point(0, 0));
  std::vector<int> counts(numLabels, 0);

  for (int y = 0; y < binaryImg.rows; ++y)
  {
    for (int x = 0; x < binaryImg.cols; ++x)
    {
      int label = labels.at<int>(y, x);
      if (label > 0)
      {
        centroids[label] += cv::Point(x, y);
        counts[label]++;
      }
    }
  }

  std::vector<std::pair<int, cv::Point>> validCentroids;
  for (int i = 1; i < numLabels; ++i)
  {
    if (counts[i] >= minSize)
    {
      centroids[i] /= counts[i];
      validCentroids.emplace_back(counts[i], centroids[i]);
    }
  }

  float line_degree = 0.0f;
  std::array<float, 4> result = {0.0f, 0.0f, 0.0f, 0.0f};

  if (validCentroids.size() > 1)
  {
    std::sort(validCentroids.begin(), validCentroids.end(), [](const auto &a, const auto &b)
              { return a.first > b.first; });

    const cv::Point &pt1 = validCentroids[0].second;
    const cv::Point &pt2 = validCentroids[1].second;
    cv::Point midPoint = (pt1 + pt2) * 0.5;

    if (cv::Rect(0, 0, binaryImg.cols, binaryImg.rows).contains(pt1) &&
        cv::Rect(0, 0, binaryImg.cols, binaryImg.rows).contains(pt2))
    {
      line(output_img, pt1, pt2, cv::Scalar(255, 255, 255), 2);
      line_degree = static_cast<float>(calAng_Vertical(pt1, pt2));
      result = {static_cast<float>(midPoint.x), static_cast<float>(midPoint.y), 0.0f, line_degree};
      priority = false;
    }
  }
  else if (validCentroids.size() == 1)
  {
    int label = labels.at<int>(validCentroids[0].second.y, validCentroids[0].second.x);
    std::vector<cv::Point> points;

    for (int y = 0; y < binaryImg.rows; ++y)
    {
      for (int x = 0; x < binaryImg.cols; ++x)
      {
        if (labels.at<int>(y, x) == label)
        {
          points.emplace_back(x, y);
        }
      }
    }

    if (points.size() >= 2)
    {
      line_degree = static_cast<float>(calAng_Vertical(points));
      result = {static_cast<float>(validCentroids[0].second.x), static_cast<float>(validCentroids[0].second.y), 0.0f, line_degree};
      priority = true;
    }
  }

  return result;
}

double YutVision::calAng_Vertical(const cv::Point &pt1, const cv::Point &pt2)
{
  if (pt1 == pt2)
  {
    return 0.0;
  }

  float dx = pt2.x - pt1.x;
  float dy = pt2.y - pt1.y;

  float angle = atan2(dy, dx) * 180 / CV_PI;

  double adjustedAngle = 90 - angle;

  if (adjustedAngle > 90)
  {
    adjustedAngle = adjustedAngle - 180.0;
  }

  return adjustedAngle;
}

double YutVision::calAng_Vertical(const std::vector<cv::Point> &points)
{
  if (points.size() < 2)
  {
    return 0.0; // 점이 너무 적으면 각도 계산 불가
  }

  cv::Vec4f lineParams;
  cv::fitLine(points, lineParams, cv::DIST_L2, 0, 0.01, 0.01);

  // 수평선(x축)을 기준으로 각도를 계산
  float angle = atan2(lineParams[1], lineParams[0]) * 180 / CV_PI;

  double adjustedAngle = 90 - angle;

  if (adjustedAngle > 90)
  {
    adjustedAngle = adjustedAngle - 180.0;
  }

  return adjustedAngle;
}

int YutVision::depthDetect(const cv::Mat &image, const cv::Point &point)
{
  if (image.empty())
  {
    std::cout << "Image empty!" << std::endl;
    return -1;
  }

  if (point.x < 0 || point.x >= image.cols || point.y < 0 || point.y >= image.rows)
  {
    std::cout << "Point out of image bounds!" << std::endl;
    return -1;
  }

  uint16_t depth_value = image.at<uint16_t>(point);

  // 520에서 depth_value를 뺀 값을 반환
  return 520 - static_cast<int>(depth_value);
}

void YutVision::do_labeling(cv::Mat &img_binary, int pixel_threshold)
{
  // blobs 벡터 초기화
  blobs.clear();

  cv::Mat img_labels, stats, centroids;
  int num_of_labels = cv::connectedComponentsWithStats(img_binary, img_labels, stats, centroids, 8, CV_32S);

  for (int i = 1; i < num_of_labels; i++)
  {
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    int left = stats.at<int>(i, cv::CC_STAT_LEFT);
    int top = stats.at<int>(i, cv::CC_STAT_TOP);
    int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

    if (area >= pixel_threshold)
    {
      blobs.push_back(cv::Rect(left, top, width, height));
    }
  }
}
