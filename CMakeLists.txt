cmake_minimum_required(VERSION 3.5)
project(arl_hand_tracker)

#list(APPEND CMAKE_MODULE_PATH "/home/user/libraries/")
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -g -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_filters
  arl_hw_msgs
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
  librealsense
  tf
)

find_package(PCL 1.8.1 EXACT REQUIRED)

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
add_dependencies(arl_marker_tracker ${PROJECT_NAME}_gencfg)
target_link_libraries(arl_marker_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


add_executable(arl_marker_tracker_node src/arl_marker_tracker_node.cpp src/realsense_zr300.cpp)
add_dependencies(arl_marker_tracker_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(arl_marker_tracker_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense arl_marker_tracker)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/arl_hand_tracker.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/arl_hand_tracker_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_arl_hand_tracker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
