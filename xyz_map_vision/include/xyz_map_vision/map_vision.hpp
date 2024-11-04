#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <librealsense2/rsutil.h>
#include "sensor_msgs/msg/camera_info.hpp"
#include "xyz_interfaces/srv/yutnori_board_state.hpp"
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

struct Square
{
    int x1, y1, x2, y2; // Coordinates of the square
    int number;         // Number of the square
};

struct Player
{
    std::vector<int> positions;
    std::vector<int> tokens;
};

class MapVision
{
public:
    MapVision();
    std::vector<cv::Point> initializeROI(const cv::Mat &img);
    void drawCells(cv::Mat &img);
    int findPlayerLocation(const cv::Point &point);
    Player processImage(const cv::Mat &raw_img, const cv::Mat &roi_img, const cv::Mat &depth_img,
                        sensor_msgs::msg::CameraInfo::SharedPtr camera_info, const cv::Mat &perspective_matrix, cv::Mat &output_img, const int hsvValues[6], const int player_num_);
    void processContours(cv::Mat &output_img, const cv::Mat &depth_img, const cv::Mat &color_mask);
    cv::Mat extractColorRegion(const cv::Mat &img, const int hsvValues[6]);
    cv::Rect rectConverter(const cv::Rect &roi_rect, const cv::Mat &perspective_matrix);
    std::vector<cv::Point> largestSquare;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    cv::Mat original_img;
    int player_num = 0;
    cv::Mat perspective_matrix_;
    xyz_interfaces::srv::YutnoriBoardState player_data_msg;
    Player p1;
    Player p2;

private:
    int depthDetect(const cv::Mat &image, const cv::Rect &rect);
    std::vector<Square> squares;
    rs2_intrinsics intrinsic_;
    int depth_4 = 650;






};
