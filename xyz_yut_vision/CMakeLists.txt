cmake_minimum_required(VERSION 3.8)
project(xyz_yut_vision)

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(xyz_interfaces REQUIRED)   
find_package(realsense2 REQUIRED)


# include 디렉토리 설정
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

add_library(yut_vision src/yut_vision.cpp)

ament_target_dependencies(yut_vision
  rclcpp 
  sensor_msgs
  cv_bridge 
  geometry_msgs 
  std_msgs
  image_transport
  xyz_interfaces
  realsense2
)

# 실행 파일(노드) 생성
add_executable(xyz_yut_vision_node src/xyz_yut_vision_node.cpp)

ament_target_dependencies(xyz_yut_vision_node
  rclcpp 
  sensor_msgs 
  cv_bridge 
  geometry_msgs 
  std_msgs
  image_transport
  xyz_interfaces
)

target_link_libraries(yut_vision
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
)

target_link_libraries(xyz_yut_vision_node
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
  yut_vision
)

# 설치 경로 설정
install(TARGETS
  xyz_yut_vision_node
  yut_vision
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# ament 패키징 설정
ament_package()
