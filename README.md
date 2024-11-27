# Autonomous Yut game robot vision package
This package is a vision module for a robot Yut game on ROS2.

## Info
 o 대회 : 엑스와이지 인간-로봇 상호작용 기술 구현 챌린지 
         (XYZ Human-Robot Interaction Technology Implementation Challenge)
 
 o 주최 : 산업통상자원부
 
 o 주관 : 한국로봇산업진흥원, 한국로봇산업협회
 
 o 세부주관 : (주)엑스와이지
 
 o 후원 : 푸드테크로봇협의회
 
 o 결과 : 한국로봇산업기술진흥원 원장상

## Env

1. Ros2 humble
2. Ubuntu 22.04

## Usage

```
ros2 launch realsense2_camera rs_align_depth_launch.py camera_name:=my_d455 device_type:=d455
ros2 launch realsense2_camera rs_align_depth_launch.py camera_name:=my_d455 device_type:=d455
ros2 launch xyz_yut_vision start.py
ros2 launch xyz_map_vision start.py
```

## Feature
1. Detect and Extract game map, game piece.
   
![image](https://github.com/user-attachments/assets/7792feb2-df37-4a4e-95ed-13a4ef4285c3)

2. Numbering yut piece and extract position and degree. (Back flip pieces)
   
![image](https://github.com/user-attachments/assets/c85a1625-d5e6-49e2-ab60-e38377e14de8)

3. Numbering yut piece and extract position and degree. (Non back flip pieces)
   
![image](https://github.com/user-attachments/assets/9c90b4c8-1efd-4fff-ab05-0676596aa052)
