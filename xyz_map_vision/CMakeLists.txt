cmake_minimum_required(VERSION 3.8)
project(xyz_map_vision)

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(realsense2 REQUIRED)
find_package(xyz_interfaces REQUIRED)

# include 디렉토리 설정
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

# map_vision 라이브러리 생성
add_library(map_vision src/map_vision.cpp)

ament_target_dependencies(map_vision
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  std_msgs
  image_transport
  realsense2
  xyz_interfaces
)

# 실행 파일(노드) 생성
add_executable(xyz_map_vision_node src/xyz_map_vision_node.cpp)

ament_target_dependencies(xyz_map_vision_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  std_msgs
  image_transport
  xyz_interfaces
)

target_link_libraries(map_vision
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
)

target_link_libraries(xyz_map_vision_node
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
  map_vision
)

# 설치 설정
install(TARGETS
  xyz_map_vision_node
  map_vision
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
