#ifndef REALSENSE_ZR300_H
#define REALSENSE_ZR300_H

#include <omp.h>
#include <iostream>
#include <functional>
#include <iomanip>
#include <map>
#include <atomic>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <librealsense/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <arl_hand_tracker/Extrinsics.h>
#include <arl_hand_tracker/IMUInfo.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <arl_hand_tracker/constants.h>
#include <arl_hand_tracker/marker_tracker.h>

class RealsenseZR300
{
public:
  RealsenseZR300();
  ~RealsenseZR300();

  bool cameraStarted_;

  void disableRealsense();
  void getParameters();
  bool setupDevice();
  void setupPublishers();
  void initUndistortionParams(rs::stream stream);
  void setupStreams();
  void registerVisualLambdaCallback(rs::stream stream);
  std::function<void(rs::motion_data)> getImuCallback();
  void getStreamCalibData(rs::stream stream);
  void publishStaticTransforms();
  float rgbFromTexCoord(cv::Mat tex, struct rs::float2 coord, rs::intrinsics tex_intrinsics);
  void publishPCTopic(ros::Time t);
  void getImuInfo(rs::device* device, arl_hand_tracker::IMUInfo &accelInfo, arl_hand_tracker::IMUInfo &gyroInfo);
  arl_hand_tracker::Extrinsics rsExtrinsicsToMsg(rs::extrinsics rsExtrinsics);
  arl_hand_tracker::Extrinsics getFisheye2ImuExtrinsicsMsg(rs::device* device);
  arl_hand_tracker::Extrinsics getFisheye2DepthExtrinsicsMsg(rs::device* device);

private:
  ros::NodeHandle node_handle, pnh_;
  std::unique_ptr< rs::context > ctx_;
  rs::device *device_;

  std::string serial_no_;
  std::string usb_port_id_;
  std::string camera_type_;

  MarkerTracker *tracker;

  std::map<rs::stream, int> width_;
  std::map<rs::stream, int> height_;
  std::map<rs::stream, int> fps_;
  std::map<rs::stream, bool> enable_;
  std::map<rs::stream, std::string> stream_name_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  bool publish_undistorted_color_;
  image_transport::Publisher undist_color_publisher_;

  // R200 and ZR300 types
  std::map<rs::stream, image_transport::Publisher> image_publishers_;
  std::map<rs::stream, int> image_format_;
  std::map<rs::stream, rs::format> format_;
  std::map<rs::stream, ros::Publisher> info_publisher_;
  std::map<rs::stream, cv::Mat> image_;
  std::map<rs::stream, std::string> encoding_;
  std::string base_frame_id_;
  std::map<rs::stream, std::string> frame_id_;
  std::map<rs::stream, std::string> optical_frame_id_;
  std::string imu_frame_id_, optical_imu_frame_id_;
  std::map<rs::stream, unsigned int> seq_;
  std::map<rs::stream, unsigned int> unit_step_size_;
  std::map<rs::stream, std::function<void(rs::frame)>> stream_callback_per_stream;
  std::map<rs::stream, sensor_msgs::CameraInfo> camera_info_;
  std::map<rs::stream, cv::Mat[2]> remap_parameter_;
  ros::Publisher pointcloud_publisher_;
  bool intialize_time_base_;
  double camera_time_base_;
  ros::Time ros_time_base_;

  ros::Publisher accelInfo_publisher_, gyroInfo_publisher_, fe2imu_publisher_, fe2depth_publisher_;
  ros::Publisher imu_publishers_[2];
  unsigned int seq_motion[2];
  std::string optical_imu_id_[2];
};

#endif //REALSENSE_ZR300_H
