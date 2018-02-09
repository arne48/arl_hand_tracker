cmake_minimum_required(VERSION 3.5)
project(arl_hand_tracker_msgs)

#list(APPEND CMAKE_MODULE_PATH "/home/user/libraries/")
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -g -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_filters
  arl_hw_msgs
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

find_package(PCL 1.7.2 EXACT REQUIRED)

find_package(OpenCV 3 REQUIRED)

add_message_files(
        FILES
        IMUInfo.msg
        Extrinsics.msg
        TrainingData.msg
)

generate_dynamic_reconfigure_options(
        cfg/MarkerFilter.cfg
)

generate_messages(
        DEPENDENCIES
        sensor_msgs
        std_msgs
        arl_hw_msgs
        geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arl_hand_tracker
#  CATKIN_DEPENDS realsense_ros_camera roscpp rospy std_msgs
#  DEPENDS system_lib
)
