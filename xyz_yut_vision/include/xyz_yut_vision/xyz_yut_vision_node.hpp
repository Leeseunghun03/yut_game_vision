#ifndef YUT_VISION_MASTER_HPP
#define YUT_VISION_MASTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "yut_vision.hpp"
#include "xyz_interfaces/srv/yutnori_yut_state.hpp"
#include "xyz_interfaces/srv/yut_position.hpp"

class YutMaster : public rclcpp::Node
{
public:
    YutMaster();

private:
    // Callback functions
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_raw);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void depthimageCallback(const sensor_msgs::msg::Image::SharedPtr img_raw);

    // Core processing
    void run(const cv::Mat &roi_img);
    int defineStatus(int back_flip, bool backdo);
    void pubMsg();
    void printStatus(int status);

    // Service handlers
    void handleStateService(const std::shared_ptr<xyz_interfaces::srv::YutnoriYutState::Request> request,
                            const std::shared_ptr<xyz_interfaces::srv::YutnoriYutState::Response> response);
    void handlePositionService(const std::shared_ptr<xyz_interfaces::srv::YutPosition::Request> request,
                               const std::shared_ptr<xyz_interfaces::srv::YutPosition::Response> response);

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;

    // Services
    rclcpp::Service<xyz_interfaces::srv::YutnoriYutState>::SharedPtr yut_state_service;
    rclcpp::Service<xyz_interfaces::srv::YutPosition>::SharedPtr yut_position_service;

    // Image data and camera info
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    cv::Rect roi_;
    cv::Mat depth_img;
    cv::Mat color_img;

    // Library class
    YutVision yv;

    // Transformation-related
    cv::Mat perspectiveMatrix;
    std::vector<cv::Point2f> rotated_corners;
    std::vector<cv::Point> roiPoint;

    // Processed data
    std::vector<std::array<float, 4>> combined_vector;

    // Status and control
    int status = 0;
    int current_yut_num = 4;
    bool isInit = false;
    float rotationAngle = 0.0;

    // Color detection flags
    bool isBlue = false;
    bool isPink = false;
    bool isRed = false;
    bool isBlack = false;
    bool isOrange = false;

    // HSV color detection values
    int Blue_value[6] = {130, 255, 255, 90, 50, 50};
    int Pink_value[6] = {170, 255, 255, 150, 50, 50};
    int Red_value[6] = {10, 255, 255, 0, 50, 50};
    int Black_value[6] = {180, 40, 200, 0, 0, 0};
    int Yut_value[6] = {50, 80, 255, 0, 0, 180};
    int Orange_value[6] = {25, 255, 255, 10, 50, 50};

    // Aruco Marker Parameter
    cv::Vec3d markerCenterOffset = cv::Vec3d(0.0, 0.0, 0.0); // mm
    Vec3d targetWorld;
    float markerLength = 36.5;
    cv::Point2f markerCenter, objectCenter;
    float markerLengthInPixels = 0.0;
    double yaw_angle = 0.0349; // 2degree

    bool arucoInit = false;

    void publishImage(const cv::Mat &image) {
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    // 메시지 발행
    image_publisher_->publish(*img_msg);

    //RCLCPP_INFO(this->get_logger(), "Image published");
    }
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

#endif // YUT_VISION_MASTER_HPP
