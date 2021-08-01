cmake_minimum_required(VERSION 3.0.2)
project(camera-direction-determinator)

find_package(catkin REQUIRED COMPONENTS
  color_detector_msgs
  color_detector_params
  color_detector_srvs
  dynamixel_angle_msgs
  kalman-filter
  roscpp
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES camera-direction-deteminator
#  CATKIN_DEPENDS color_detector_msgs color_detector_params dynamixel_angle_msgs kalman-filter roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/camera_direction_determinator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
