#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <librealsense2/rsutil.h>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>

// Camera Image Size
#define RAW_X 320
#define RAW_Y 240

using namespace cv;
using namespace std;

class YutVision
{
public:
    void changeToBinary(cv::Mat &input_img, cv::Mat &output_img, int hsv_low[], int hsv_high[]);
    cv::Mat addBinaryImage(const Mat &img1, const Mat &img2);
    cv::Mat closeGaps(const cv::Mat &binary_img, int kernel_size);
    cv::Mat changeToBinary(cv::Mat &input_img);
    void calculateArucoMarkerProperties(Mat &image, Point2f &markerCenter, float &markerSideLength);
    void convertToWorldCoordinates(const Point2f &markerCenterPixel, const Vec3d &markerCenterWorld,
                                   const Point2f &targetPixel, Vec3d &targetWorld,
                                   float markerLengthInWorld, double yaw_angle, float markerLengthInPixels);
    cv::Point3f imagePointToWorld(cv::Point2f imagePoint, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                  const cv::Vec3d &rvec, const cv::Vec3d &tvec, float z_world = 0.0);
    cv::Mat extractColorRegion(const cv::Mat &img, cv::Mat &output_mask, const int hsvValues[6]);
    std::array<float, 4> labelAndConnectComponents(const cv::Mat &binaryImg, cv::Mat &output_img, int minSize, bool &priority);
    void detectArucoAndTransformPoints(Mat &image, std::vector<Point2f> &imagePoints, const Vec3d &markerActualPosition,
                                       const Mat &cameraMatrix, const Mat &distCoeffs, float markerLength);
    int CalProcess(const cv::Mat &img);
    int countFrontYutNum(const cv::Mat &img, const int hsvValues[6], int thresholdArea);
    void do_labeling(Mat &img_binary, int pixel_threshold);
    bool JudgeProcess(const cv::Mat &img, cv::Mat &output_img, cv::Mat &output_mask, const int hsvValues[6], int threshold);
    bool isRegionEnough(const cv::Mat &colorRegion, int thresholdArea, bool screen);
    std::vector<std::array<float, 4>> detectYutFront(const cv::Mat &img, const int hsvValues[6], int back_flip, int yut_pixel_size, bool &isRegion, int current_yut_num);
    double calAng_Vertical(const cv::Point &pt1, const cv::Point &pt2);
    double calAng_Vertical(const std::vector<cv::Point> &points);
    std::vector<cv::Point> initializeROI(const cv::Mat &img);
    int depthDetect(const cv::Mat &image, const cv::Point &point);
    void transformToOriginal(float x_in_resized, float y_in_resized, float &x_out, float &y_out, const cv::Mat &perspectiveMatrix, const cv::Size &original_roi_size);
    cv::Mat extractROI(const cv::Mat &image, const std::vector<cv::Point2f> &markerCorners, float markerSizeMM, float roiSizeMM, std::vector<cv::Point2f> &points);
    vector<cv::Rect> blobs;

    int max_num = 0;
    int max_pixel = 0;
    double scaleFactor = 0.0;

private:
};
