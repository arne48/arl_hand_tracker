#ifndef ARL_HAND_TRACKER_MARKER_TRACKER_H
#define ARL_HAND_TRACKER_MARKER_TRACKER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <librealsense/rs.hpp>
#include <opencv2/core/core.hpp>
#include <dynamic_reconfigure/server.h>
#include <arl_hand_tracker_msgs/MarkerFilterConfig.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <arl_hand_tracker/constants.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <visualization_msgs/Marker.h>

class MarkerTracker{
public:
  MarkerTracker(rs::device *device, ros::NodeHandle nh);
  ~MarkerTracker();

  void publishMarker(cv::Mat frame, cv::Mat depth_frame);

private:
  struct filter_config_t {
    int red_h_max;
    int red_h_min;
    int red_s_max;
    int red_s_min;
    int red_v_max;
    int red_v_min;

    int blue_h_max;
    int blue_h_min;
    int blue_s_max;
    int blue_s_min;
    int blue_v_max;
    int blue_v_min;

    int yellow_h_max;
    int yellow_h_min;
    int yellow_s_max;
    int yellow_s_min;
    int yellow_v_max;
    int yellow_v_min;
  } current_filter_setting;

  struct marker_pose_t {
    geometry_msgs::Point position;
    tf::Quaternion orientation;
  };

  rs::device *device_;
  rs::intrinsics depth_intrinsic_;
  rs::intrinsics color_intrinsic_;
  rs::extrinsics depth_extrinsic_;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  ros::Publisher red_marker_cloud_pub_;
  ros::Publisher blue_marker_cloud_pub_;
  ros::Publisher yellow_marker_cloud_pub_;
  ros::Publisher marker_pub_;
  tf::TransformBroadcaster transform_broadcaster_;

  dynamic_reconfigure::Server<arl_hand_tracker_msgs::MarkerFilterConfig> server;
  void filterCallback(arl_hand_tracker_msgs::MarkerFilterConfig &config, uint32_t level);
  void publishTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name);
  struct marker_pose_t getMarkerPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  uchar getFromTexCoord(cv::Mat tex, struct rs::float2 coord, rs::intrinsics tex_intrinsics);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getBiggestCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

};

#endif //ARL_HAND_TRACKER_MARKER_TRACKER_H
