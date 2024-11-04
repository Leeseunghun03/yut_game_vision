#include "../include/xyz_yut_vision/xyz_yut_vision_node.hpp"

using namespace cv;
using namespace std;

using std::placeholders::_1;
using std::placeholders::_2;

cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1280, 0, 640, // fx, 0, cx
                        0, 720, 360,                            // 0, fy, cy
                        0, 0, 1);                               // 0, 0, 1
cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

YutMaster::YutMaster() : Node("xyz_yut_vision")
{
    std::string depth_camera_topic, camera_info_topic, camera_topic;
    depth_camera_topic = this->declare_parameter<std::string>("depth_camera_topic", "");
    camera_info_topic = this->declare_parameter<std::string>("camera_info_topic", "");
    camera_topic = this->declare_parameter<std::string>("camera_topic", "");

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("yut_topic", 10);

    img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, 1, std::bind(&YutMaster::imageCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&YutMaster::cameraInfoCallback, this, std::placeholders::_1));

    depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_camera_topic, 10,
        std::bind(&YutMaster::depthimageCallback, this, std::placeholders::_1));

    int x, y, width, height;

    x = this->declare_parameter<int>("roi_value.x", 0);
    y = this->declare_parameter<int>("roi_value.y", 0);
    width = this->declare_parameter<int>("roi_value.width", 0);
    height = this->declare_parameter<int>("roi_value.height", 0);
    rotationAngle = this->declare_parameter<float>("roi_value.rotationAngle", 0);
    yaw_angle = static_cast<double>(rotationAngle) * M_PI / 180.0;

    roi_ = cv::Rect(x, y, width, height);
    RCLCPP_INFO(this->get_logger(), "ROI set to: (%d, %d), width: %d, height: %d, rotationAngle: %f", x, y, width, height, rotationAngle);
    RCLCPP_INFO(this->get_logger(), "Degree: %f -> Radian: %lf", rotationAngle, yaw_angle);

    std::vector<long int> temp_blue_value = this->declare_parameter<std::vector<long int>>("Blue_value", std::vector<long int>{130, 255, 255, 90, 50, 50});
    std::vector<long int> temp_pink_value = this->declare_parameter<std::vector<long int>>("Pink_value", std::vector<long int>{170, 255, 255, 150, 50, 50});
    std::vector<long int> temp_red_value = this->declare_parameter<std::vector<long int>>("Red_value", std::vector<long int>{10, 255, 255, 0, 50, 50});
    std::vector<long int> temp_black_value = this->declare_parameter<std::vector<long int>>("Black_value", std::vector<long int>{180, 40, 200, 0, 0, 0});
    std::vector<long int> temp_yut_value = this->declare_parameter<std::vector<long int>>("Yut_value", std::vector<long int>{50, 80, 255, 0, 0, 180});
    std::vector<long int> temp_orange_value = this->declare_parameter<std::vector<long int>>("Orange_value", std::vector<long int>{25, 255, 255, 10, 50, 50});

    std::transform(temp_blue_value.begin(), temp_blue_value.end(), Blue_value, [](long int val)
                   { return static_cast<int>(val); });

    std::transform(temp_pink_value.begin(), temp_pink_value.end(), Pink_value, [](long int val)
                   { return static_cast<int>(val); });

    std::transform(temp_red_value.begin(), temp_red_value.end(), Red_value, [](long int val)
                   { return static_cast<int>(val); });

    std::transform(temp_black_value.begin(), temp_black_value.end(), Black_value, [](long int val)
                   { return static_cast<int>(val); });

    std::transform(temp_yut_value.begin(), temp_yut_value.end(), Yut_value, [](long int val)
                   { return static_cast<int>(val); });

    std::transform(temp_orange_value.begin(), temp_orange_value.end(), Orange_value, [](long int val)
                   { return static_cast<int>(val); });

    double marker_offset_x = this->declare_parameter("markerCenterOffset.x", 0.0);
    double marker_offset_y = this->declare_parameter("markerCenterOffset.y", 0.0);
    double marker_offset_z = this->declare_parameter("markerCenterOffset.z", 0.0);

    markerCenterOffset = cv::Vec3d(marker_offset_x, marker_offset_y, marker_offset_z);

    RCLCPP_INFO(this->get_logger(), "Marker center offset: [%f, %f, %f]",
                markerCenterOffset[0], markerCenterOffset[1], markerCenterOffset[2]);

    yut_state_service = this->create_service<xyz_interfaces::srv::YutnoriYutState>(
        "yut_state",
        std::bind(&YutMaster::handleStateService, this, _1, _2));

    yut_position_service = this->create_service<xyz_interfaces::srv::YutPosition>(
        "yut_position",
        std::bind(&YutMaster::handlePositionService, this, _1, _2));

    rotated_corners.resize(4);
}

void YutMaster::handleStateService(const std::shared_ptr<xyz_interfaces::srv::YutnoriYutState::Request> request,
                                   const std::shared_ptr<xyz_interfaces::srv::YutnoriYutState::Response> response)
{
    response->yut_info = status;
}

void YutMaster::handlePositionService(const std::shared_ptr<xyz_interfaces::srv::YutPosition::Request> request,
                                      const std::shared_ptr<xyz_interfaces::srv::YutPosition::Response> response)
{
    if (!combined_vector.empty())
    {
        // RCLCPP_WARN(get_logger(), "%d %d %d %d", combined_vector[0][0], combined_vector[0][1], combined_vector[0][2], combined_vector[0][3]);
        response->pos = {combined_vector[0][0], combined_vector[0][1], combined_vector[0][2], combined_vector[0][3]};
        // cout << "combined_vector:" << combined_vector[0][0] << "," << combined_vector[0][1] << "," << combined_vector[0][2] << "," << combined_vector[0][3] << endl;
    }
    else
    {
        response->pos = {0.0, 0.0, 0.0, 0.0};
        std::cerr << "Warning: combined_vector is empty, sending default response." << std::endl;
    }

    // current_yut_num을 항상 응답
    response->left_num = current_yut_num;
}

void YutMaster::depthimageCallback(const sensor_msgs::msg::Image::SharedPtr img_raw)
{
    depth_img = cv_bridge::toCvCopy(img_raw, sensor_msgs::image_encodings::TYPE_16UC1)->image;
}

void YutMaster::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    camera_info_ = msg;
}

void YutMaster::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_raw)
{
    color_img = cv_bridge::toCvCopy(img_raw, sensor_msgs::image_encodings::BGR8)->image;

    if (!isInit)
    {
        // ROI의 왼쪽 위 점을 기준으로 회전 각도 설정
        cv::Point2f top_left(roi_.x, roi_.y);
        cv::Point2f top_right(roi_.x + roi_.width, roi_.y);
        cv::Point2f bottom_left(roi_.x, roi_.y + roi_.height);
        cv::Point2f bottom_right(roi_.x + roi_.width, roi_.y + roi_.height);

        // 왼쪽 위 점을 기준으로 회전 변환을 위한 변환 행렬 계산
        cv::Mat rotationMatrix = cv::getRotationMatrix2D(top_left, rotationAngle, 1.0);

        // 회전 변환을 적용하여 새로운 ROI 점 계산
        std::vector<cv::Point2f> roi_corners = {top_left, top_right, bottom_right, bottom_left};
        cv::transform(roi_corners, rotated_corners, rotationMatrix);

        // 회전된 ROI 점을 사용하여 ROI 이미지를 추출하기 위한 변환 행렬 계산
        std::vector<cv::Point2f> dst_points = {cv::Point2f(0, 0), cv::Point2f(roi_.width, 0),
                                               cv::Point2f(roi_.width, roi_.height),
                                               cv::Point2f(0, roi_.height)};
        perspectiveMatrix = cv::getPerspectiveTransform(rotated_corners, dst_points);
        isInit = true;
    }

    // 회전된 ROI 추출
    cv::Mat roi_img;
    cv::warpPerspective(color_img, roi_img, perspectiveMatrix, cv::Size(roi_.width, roi_.height));

    // ROI 유효성 검사
    if (!roi_img.empty())
    {
        // 회전된 ROI 꼭짓점에 점 표시
        for (const auto &point : rotated_corners)
        {
            cv::circle(color_img, point, 5, cv::Scalar(0, 0, 255), -1); // 빨간색 원 표시
        }

        cv::imshow("roi", roi_img);
        publishImage(roi_img); //!
        cv::waitKey(1);

        cv::resize(roi_img, roi_img, cv::Size(300, 300), 0, 0, cv::INTER_LINEAR);
        run(roi_img);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "ROI is out of image bounds.");
    }
}

void YutMaster::run(const cv::Mat &roi_img)
{
    int back_flip = 0;
    bool backdo = false;

    std::vector<std::array<float, 4>> back_flip_vector;
    std::vector<std::array<float, 4>> yut_front_vector;

    cv::Mat resize_img, color_mask, output_img, cal_img;
    cv::resize(roi_img, resize_img, cv::Size(300, 300), 0, 0, cv::INTER_LINEAR);
    cv::Mat sum_mask = cv::Mat::zeros(resize_img.size(), CV_8UC1);

    if (!arucoInit)
    {
        yv.calculateArucoMarkerProperties(color_img, markerCenter, markerLengthInPixels);
        arucoInit = true;
    }
    else
    {
        // Function to process each color detection (black, red, pink, blue)
        auto processColorDetection = [&](int *color_value, std::vector<std::array<float, 4>> &flip_vector, int &flip_count)
        {
            bool priority = false;
            if (yv.JudgeProcess(resize_img, output_img, color_mask, color_value, 400))
            {
                flip_count++;
                sum_mask += color_mask;

                std::array<float, 4> pos = yv.labelAndConnectComponents(color_mask, cal_img, 400, priority);
                float original_x, original_y;
                yv.transformToOriginal(pos[0], pos[1], original_x, original_y, perspectiveMatrix, cv::Size(roi_.width, roi_.height));
                objectCenter = cv::Point2f(original_x, original_y);

                // cout << "markerCenter:" << markerCenter << endl;
                // cout << "objectCenter:" << objectCenter << endl;

                yv.convertToWorldCoordinates(markerCenter, markerCenterOffset,
                                             objectCenter, targetWorld,
                                             markerLength, -yaw_angle, markerLengthInPixels);

                // cout << "Coordinate: " << targetWorld[0] << "," << targetWorld[0] << endl;

                pos[0] = targetWorld[0];
                pos[1] = targetWorld[1];

                cv::Point depth_point(static_cast<int>(original_x), static_cast<int>(original_y));
                int depth_value = yv.depthDetect(depth_img, depth_point);
                pos[2] = (depth_value != -1) ? static_cast<float>(depth_value) : 0.0f;

                flip_vector.push_back(pos);
            }
        };

        // Process each color: black, red, pink, blue
        processColorDetection(Orange_value, back_flip_vector, back_flip);
        if (back_flip == 1)
            backdo = true;

        processColorDetection(Red_value, back_flip_vector, back_flip);
        processColorDetection(Pink_value, back_flip_vector, back_flip);
        processColorDetection(Blue_value, back_flip_vector, back_flip);

        std::sort(back_flip_vector.begin(), back_flip_vector.end(),
                  [](const std::array<float, 4> &a, const std::array<float, 4> &b)
                  {
                      return a[1] > b[1];
                  });

        if (!sum_mask.empty())
        {
            cv::bitwise_and(resize_img, resize_img, output_img, sum_mask);
            cv::imshow("output", output_img);
            cv::waitKey(1);
        }

        bool isAll = false;

        int front_yut_num = yv.countFrontYutNum(resize_img, Yut_value, 2000);
        current_yut_num = front_yut_num + back_flip;
        if (current_yut_num > 4)
            current_yut_num = 4;
        cout << "Current_yut_num: " << current_yut_num << endl;

        if (current_yut_num != 0 && back_flip != current_yut_num)
        {
            // One yut 2500
            yut_front_vector = yv.detectYutFront(resize_img, Yut_value, back_flip, 2000, isAll, current_yut_num);

            for (auto &pos : yut_front_vector)
            {
                float original_x, original_y;
                yv.transformToOriginal(pos[0], pos[1], original_x, original_y, perspectiveMatrix, cv::Size(roi_.width, roi_.height));
                objectCenter = cv::Point2f(original_x, original_y);
                yv.convertToWorldCoordinates(markerCenter, markerCenterOffset,
                                             objectCenter, targetWorld,
                                             markerLength, -yaw_angle, markerLengthInPixels);

                pos[0] = targetWorld[0];
                pos[1] = targetWorld[1];

                cv::Point depth_point(static_cast<int>(original_x), static_cast<int>(original_y));
                int depth_value = yv.depthDetect(depth_img, depth_point);
                pos[2] = (depth_value != -1) ? static_cast<float>(depth_value) : 0.0f;
            }

            std::sort(yut_front_vector.begin(), yut_front_vector.end(),
                      [](const std::array<float, 4> &a, const std::array<float, 4> &b)
                      {
                          return a[1] > b[1];
                      });
        }
        else if (current_yut_num == 4 && back_flip == 4)
        {
            // Yut
            isAll = true;
        }
        else
        {
            isAll = false;
        }

        // Determine status based on detection results
        status = (isAll) ? defineStatus(back_flip, backdo) : 0;

        // Combine and draw detected points if all Yut pieces are detected
        if (current_yut_num != 0)
        {
            combined_vector = back_flip_vector;
            combined_vector.insert(combined_vector.end(), yut_front_vector.begin(), yut_front_vector.end());

            printStatus(status);
            std::cout << "Combined Vector Contents:" << std::endl;

            size_t index = 0;
            for (const auto &pos : combined_vector)
            {
                if (index == combined_vector.size() - 1)
                    std::cout << "[" << pos[0] << ", " << pos[1] << ", " << pos[2] << ", " << pos[3] << "]" << std::endl
                              << std::endl;
                else
                    std::cout << "[" << pos[0] << ", " << pos[1] << ", " << pos[2] << ", " << pos[3] << "]" << std::endl;

                cv::Point point(static_cast<int>(pos[0]), static_cast<int>(pos[1]));
                cv::circle(color_img, point, 5, cv::Scalar(255, 0, 0), -1);
                ++index;
            }
        }
    }

    // Display the original image with drawn points
    cv::imshow("original", color_img);
    cv::waitKey(1);

    // Clear vectors for the next iteration
    back_flip_vector.clear();
    yut_front_vector.clear();
}

int YutMaster::defineStatus(int back_flip, bool backdo)
{
    if (back_flip == 1 && backdo)
        return -1;
    else if (back_flip == 1 && !backdo)
        return 1;
    else if (back_flip == 2)
        return 2;
    else if (back_flip == 3)
        return 3;
    else if (back_flip == 4)
        return 4;
    else if (back_flip == 0)
        return 5;
    else
        return 0;
}

void YutMaster::printStatus(int status)
{
    cout << "Status: ";
    if (status == -1)
        cout << "Back do" << endl;
    else if (status == 0)
        cout << "Nak" << endl;
    else if (status == 1)
        cout << "Do" << endl;
    else if (status == 2)
        cout << "Gae" << endl;
    else if (status == 3)
        cout << "Gul" << endl;
    else if (status == 4)
        cout << "Yut" << endl;
    else if (status == 5)
        cout << "Mo" << endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YutMaster>());
    rclcpp::shutdown();
    return 0;
}
