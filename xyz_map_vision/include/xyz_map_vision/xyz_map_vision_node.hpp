#ifndef MAP_VISION_MASTER_HPP
#define MAP_VISION_MASTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "map_vision.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include "xyz_interfaces/srv/yutnori_board_state.hpp"
#include <vector>

class MapMaster : public rclcpp::Node
{
public:
    MapMaster();
    cv::Mat depth_img;
    cv::Mat perspective_matrix;
    xyz_interfaces::srv::YutnoriBoardState player_msg;
    Player p1_;
    Player p2_;

private:
    cv::Rect roi_;
    MapVision mv;
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void depthimageCallback(const sensor_msgs::msg::Image::SharedPtr img_raw);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr img_raw);
    void commandCallback(const std_msgs::msg::String::SharedPtr msg);
    void run(const cv::Mat &raw_img, const cv::Mat &roi_img, const cv::Mat &depth_img);
    void handleService(const std::shared_ptr<xyz_interfaces::srv::YutnoriBoardState::Request> request,
                       const std::shared_ptr<xyz_interfaces::srv::YutnoriBoardState::Response> response);

    cv::Point center;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Service<xyz_interfaces::srv::YutnoriBoardState>::SharedPtr map_service;

    int Red_value[6] = {40, 255, 255, 0, 40, 0};
    int Blue_value[6] = {130, 255, 255, 90, 50, 50};

    bool isInit = false;
    std::vector<cv::Point> roiPoint;


    void publishImage(const cv::Mat &image) {
    // cv::Mat을 sensor_msgs::msg::Image로 변환
    // std_msgs::msg::Header header;  // 메시지의 헤더
    // header.stamp = this->get_clock()->now();  // 시간 설정
    // header.frame_id = "camera_frame";  // 프레임 ID 설정

    // cv::Mat을 ROS2 sensor_msgs::msg::Image로 변환
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();


    // 메시지 발행
    image_publisher_->publish(*img_msg);

    //RCLCPP_INFO(this->get_logger(), "Image published");
    }
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

#endif // MAP_VISION_MASTER_HPP
