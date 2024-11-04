#include "../include/xyz_map_vision/xyz_map_vision_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

MapMaster::MapMaster() : Node("xyz_map_vision")
{
    std::string depth_camera_topic, camera_info_topic, camera_topic, command_topic;
    depth_camera_topic = this->declare_parameter<std::string>("depth_camera_topic", "");
    camera_info_topic = this->declare_parameter<std::string>("camera_info_topic", "");
    camera_topic = this->declare_parameter<std::string>("camera_topic", "");
    command_topic = this->declare_parameter<std::string>("command_topic", "");

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("map_topic", 10);



    //std::cout<<depth_camera_topic<<std::endl<<camera_info_topic<<std::endl<<camera_topic<<std::endl<<command_topic<<std::endl;

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&MapMaster::cameraInfoCallback, this, std::placeholders::_1));

    depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_camera_topic, 10,
        std::bind(&MapMaster::depthimageCallback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, 10,
        std::bind(&MapMaster::imageCallback, this, std::placeholders::_1));

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        command_topic, 1,
        std::bind(&MapMaster::commandCallback, this, std::placeholders::_1));

    map_service = this->create_service<xyz_interfaces::srv::YutnoriBoardState>(
        "board_state",
        std::bind(&MapMaster::handleService, this, _1, _2));
}

void MapMaster::handleService(const std::shared_ptr<xyz_interfaces::srv::YutnoriBoardState::Request> request,
                              const std::shared_ptr<xyz_interfaces::srv::YutnoriBoardState::Response> response)
{
    response->positions_p1 = p1_.positions;
    response->tokens_p1 = p1_.tokens;
    response->positions_p2 = p2_.positions;
    response->tokens_p2 = p2_.tokens;

    RCLCPP_INFO(this->get_logger(), "Player positions and tokens sent");
}

void MapMaster::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    camera_info_ = msg;
}

void MapMaster::imageCallback(const sensor_msgs::msg::Image::SharedPtr img_raw)
{
    if (!img_raw)
        return;

    cv::Mat color_img = cv_bridge::toCvCopy(img_raw, sensor_msgs::image_encodings::BGR8)->image;

    if (!isInit)
    {
        roiPoint = mv.initializeROI(color_img);
        isInit = true;
    }

    if (!roiPoint.empty() && !depth_img.empty())
    {
        cv::Point2f center(0, 0);
        std::vector<cv::Point2f> src_points;
        for (const auto &point : roiPoint)
        {
            center += cv::Point2f(point.x, point.y);
        }
        center *= (1.0 / roiPoint.size());

        std::vector<std::pair<cv::Point2f, float>> sorted_points;
        for (const auto &point : roiPoint)
        {
            float angle = atan2(point.y - center.y, point.x - center.x);
            sorted_points.push_back({point, angle});
        }
        std::sort(sorted_points.begin(), sorted_points.end(),
                  [](const std::pair<cv::Point2f, float> &a, const std::pair<cv::Point2f, float> &b)
                  { return a.second < b.second; });

        for (const auto &p : sorted_points)
            src_points.push_back(p.first);

        std::vector<cv::Point2f> dst_points = {cv::Point2f(0, 0), cv::Point2f(300, 0), cv::Point2f(300, 300), cv::Point2f(0, 300)};
        perspective_matrix = cv::getPerspectiveTransform(src_points, dst_points);

        cv::Mat roi_image;
        cv::warpPerspective(color_img, roi_image, perspective_matrix, cv::Size(300, 300));

        for (const auto &point : src_points)
        {
            cv::circle(color_img, point, 5, cv::Scalar(0, 0, 255), -1);
        }

        mv.drawCells(roi_image);
        run(color_img, roi_image, depth_img);
    }
    cv::imshow("Color Image", color_img);
    cv::waitKey(1);
}

void MapMaster::depthimageCallback(const sensor_msgs::msg::Image::SharedPtr img_raw)
{
    depth_img = cv_bridge::toCvCopy(img_raw, sensor_msgs::image_encodings::TYPE_16UC1)->image;
}

void MapMaster::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "init")
    {
        isInit = false;
    }
}

void MapMaster::run(const cv::Mat &raw_img, const cv::Mat &roi_img, const cv::Mat &depth_img)
{
    if (!raw_img.empty() && !roi_img.empty() && !depth_img.empty() && camera_info_ != nullptr)
    {
        cv::Mat output_img;
        p1_ = mv.processImage(raw_img, roi_img, depth_img, camera_info_, perspective_matrix, output_img, Red_value, 1);
        p2_ = mv.processImage(raw_img, roi_img, depth_img, camera_info_, perspective_matrix, output_img, Blue_value, 2);

        publishImage(roi_img);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMaster>());
    rclcpp::shutdown();
    return 0;
}
