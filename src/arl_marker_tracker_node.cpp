#include <arl_hand_tracker/realsense_zr300.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arl_marker_tracker_node");
  RealsenseZR300 zr300;
  if (!zr300.cameraStarted_)
  {
    ros::shutdown();
    return 0;
  }
  ros::spin();
  return 0;
}