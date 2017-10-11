#include <arl_hand_tracker/realsense_zr300.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arl_marker_tracker_node");
  RealsenseZR300 zr300;
  ros::spin();
  return 0;
}