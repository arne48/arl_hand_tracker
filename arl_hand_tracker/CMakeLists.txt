cmake_minimum_required(VERSION 3.5)
project(arl_hand_tracker)

#list(APPEND CMAKE_MODULE_PATH "/home/user/libraries/")
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -g -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_filters
  arl_hw_msgs
  arl_hand_tracker_msgs
  dynamic_reconfigure
  realsense_ros_camera
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  image_transport
# librealsense
  tf
)

find_package(PCL 1.7.2 EXACT REQUIRED)

find_package(OpenCV 3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arl_hand_tracker
#  CATKIN_DEPENDS realsense_ros_camera roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

add_library(arl_marker_tracker src/marker_tracker.cpp)
add_dependencies(arl_marker_tracker arl_hand_tracker_msgs_gencfg)
target_link_libraries(arl_marker_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


add_executable(arl_marker_tracker_node src/arl_marker_tracker_node.cpp src/realsense_zr300.cpp)
add_dependencies(arl_marker_tracker_node arl_hand_tracker_msgs_generate_messages_cpp)
target_link_libraries(arl_marker_tracker_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense arl_marker_tracker)
