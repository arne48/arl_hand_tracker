#include <arl_hand_tracker/marker_tracker.h>

MarkerTracker::MarkerTracker(rs::device *device, ros::NodeHandle nh)
    : it_(nh)
{
  nh_ = nh;
  device_ = device;
  depth_intrinsic_ = device_->get_stream_intrinsics(rs::stream::depth);
  color_intrinsic_ = device_->get_stream_intrinsics(rs::stream::color);
  depth_extrinsic_ = device_->get_extrinsics(rs::stream::depth, rs::stream::color);

  dynamic_reconfigure::Server<arl_hand_tracker::MarkerFilterConfig>::CallbackType f;

  f = boost::bind(&MarkerTracker::filterCallback, this, _1, _2);
  server.setCallback(f);

  current_filter_setting = {255, 0, 255, 0, 255, 0,
                            255, 0, 255, 0, 255, 0};

  image_pub_ = it_.advertise("/arl_marker_tracker/debug/filtered_image", 1);
  red_marker_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/arl_marker_tracker/debug/red_marker_cloud", 1);
  blue_marker_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/arl_marker_tracker/debug/blue_marker_cloud", 1);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
};

MarkerTracker::~MarkerTracker()
{};

void MarkerTracker::filterCallback(arl_hand_tracker::MarkerFilterConfig &config, uint32_t level)
{
  current_filter_setting = {config.red_h_max, config.red_h_min, config.red_s_max, config.red_s_min,
                            config.red_v_max, config.red_v_min, config.blue_h_max, config.blue_h_min,
                            config.blue_s_max, config.blue_s_min, config.blue_v_max, config.blue_v_min};
}

uchar MarkerTracker::getFromTexCoord(cv::Mat tex, struct rs::float2 coord, rs::intrinsics tex_intrinsics)
{
  auto pixel_x = (int)(coord.x * tex_intrinsics.width);
  if (pixel_x < 0) pixel_x = 0;
  if (pixel_x >= tex_intrinsics.width) pixel_x = tex_intrinsics.width - 1;

  auto pixel_y = (int)(coord.y * tex_intrinsics.height);
  if (pixel_y < 0) pixel_y = 0;
  if (pixel_y >= tex_intrinsics.height) pixel_y = tex_intrinsics.height -1;

  return tex.at<uchar>(pixel_y, pixel_x);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MarkerTracker::getBiggestCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);



  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr biggest_cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  size_t biggest_cluster = 0;
  if (cluster_indices.size() > 1)
  {
    for (size_t i = 1; i < cluster_indices.size(); i++)
    {
      if (cluster_indices.at(biggest_cluster).indices.size() < cluster_indices.at(i).indices.size())
      {
        biggest_cluster = i;
      }
    }
  }
  else if (cluster_indices.size() == 1)
  {
    biggest_cluster = 0;
  }

  if (cluster_indices.size() > 0)
  {
    for (std::vector<int>::const_iterator pit = cluster_indices.at(biggest_cluster).indices.begin();
         pit != cluster_indices.at(biggest_cluster).indices.end ();
         ++pit)
    {
      biggest_cluster_cloud->points.push_back (cloud_filtered->points[*pit]);
    }
    biggest_cluster_cloud->width = biggest_cluster_cloud->points.size ();
    biggest_cluster_cloud->height = 1;
    biggest_cluster_cloud->is_dense = true;
  }

  return biggest_cluster_cloud;
}

struct MarkerTracker::marker_pose_t MarkerTracker::getMarkerPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

  // Avg Normal Marker
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setRadiusSearch (0.03);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.compute (*cloud_normals);


  visualization_msgs::Marker line_list;
  line_list.header.stamp = ros::Time::now();
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 2;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.1;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  line_list.header.frame_id = "camera_depth_frame";

  geometry_msgs::Point p, po;
  for(size_t idx = 0; idx < cloud_normals->size(); idx++)
  {
    p.x += cloud->points[idx].x + cloud_normals->points[idx].normal[0];
    p.y += cloud->points[idx].y + cloud_normals->points[idx].normal[1];
    p.z += cloud->points[idx].z + cloud_normals->points[idx].normal[2];

    po.x += cloud->points[idx].x;
    po.y += cloud->points[idx].y;
    po.z += cloud->points[idx].z;
  }

  p.x /= cloud_normals->size();
  p.y /= cloud_normals->size();
  p.z /= cloud_normals->size();

  po.x /= cloud_normals->size();
  po.y /= cloud_normals->size();
  po.z /= cloud_normals->size();

  geometry_msgs::Point line_origin, line_end;
  line_origin.x = po.z;
  line_origin.y = -po.x;
  line_origin.z = -po.y;

  line_end.x = p.z;
  line_end.y = -p.x;
  line_end.z = -p.y;

  line_list.points.push_back(line_origin);
  line_list.points.push_back(line_end);
  marker_pub_.publish(line_list);


  tf::Quaternion q;
  q.setValue(0, 0, 0, 1);

  return {line_origin, q};

}

void MarkerTracker::publishTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name)
{
  if (cloud->size() > 0)
  {

    struct marker_pose_t pose = getMarkerPose(cloud);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    transform.setRotation(pose.orientation);
    transform_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_frame", name));
  }

}

void MarkerTracker::publishMarker(cv::Mat color_frame, cv::Mat depth_frame)
{

  cv::Mat filtered, hsv;
  cvtColor(color_frame, hsv, CV_BGR2HSV);

  cv::Mat red_mask, blue_mask, combined_mask;
  inRange(hsv, cv::Scalar(current_filter_setting.red_h_min, current_filter_setting.red_s_min, current_filter_setting.red_v_min),
          cv::Scalar(current_filter_setting.red_h_max, current_filter_setting.red_s_max, current_filter_setting.red_v_max), red_mask);

  inRange(hsv, cv::Scalar(current_filter_setting.blue_h_min, current_filter_setting.blue_s_min, current_filter_setting.blue_v_min),
          cv::Scalar(current_filter_setting.blue_h_max, current_filter_setting.blue_s_max, current_filter_setting.blue_v_max), blue_mask);

  cv::Mat msg_img;
  msg_img = blue_mask + red_mask;

  if (image_pub_.getNumSubscribers())
  {
    sensor_msgs::ImagePtr img;
    img = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, msg_img).toImageMsg();
    img->width = (uint32_t)msg_img.cols;
    img->height = (uint32_t)msg_img.rows;
    img->is_bigendian = 0;
    img->step = msg_img.cols * sizeof(unsigned char);
    img->header.frame_id = "0";
    img->header.stamp = ros::Time::now();

    image_pub_.publish(img);
  }


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr red_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr blue_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  float depth_scale_meters = device_->get_depth_scale();
  for (int v = 0; v < depth_intrinsic_.height; v++)
  {
    for (int u = 0; u < depth_intrinsic_.width; u++)
    {
      float depth_point[3], scaled_depth;
      uint16_t depth_value;
      int depth_offset;

      depth_offset = (u * sizeof(uint16_t)) + (v * sizeof(uint16_t) * depth_intrinsic_.width);
      memcpy(&depth_value, &depth_frame.data[depth_offset], sizeof(uint16_t));
      scaled_depth = static_cast<float>(depth_value) * depth_scale_meters;

      if (scaled_depth > 0.0f && scaled_depth <= realsense_ros_camera::MAX_Z) {
        float depth_pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
        rs_deproject_pixel_to_point(depth_point, &depth_intrinsic_, depth_pixel, scaled_depth);

        struct rs::float2 tex_coord = color_intrinsic_.project_to_texcoord(depth_extrinsic_.transform({depth_point[0],
                                                                                                      depth_point[1],
                                                                                                      depth_point[2]}));
        if(getFromTexCoord(red_mask, tex_coord, color_intrinsic_))
        {
          pcl::PointXYZRGB point;
          point.x = depth_point[0];
          point.y = depth_point[1];
          point.z = depth_point[2];
          point.r = 255;

          red_cloud_ptr->push_back(point);
        }
        else if(getFromTexCoord(blue_mask, tex_coord, color_intrinsic_))
        {
          pcl::PointXYZRGB point;
          point.x = depth_point[0];
          point.y = depth_point[1];
          point.z = depth_point[2];
          point.b = 255;

          blue_cloud_ptr->push_back(point);
        }
      }
    }
  }


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_red_cloud_ptr = getBiggestCluster(red_cloud_ptr);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_blue_cloud_ptr = getBiggestCluster(blue_cloud_ptr);

  publishTransform(debug_blue_cloud_ptr, "blue_marker_frame");
  publishTransform(debug_red_cloud_ptr, "red_marker_frame");


  sensor_msgs::PointCloud2 output;

  pcl::PCLPointCloud2 pcl_pc;
  pcl::toPCLPointCloud2(*debug_red_cloud_ptr, pcl_pc);
  pcl_conversions::fromPCL(pcl_pc, output);
  output.header.frame_id = "camera_depth_optical_frame";
  output.header.stamp = ros::Time::now();
  red_marker_cloud_pub_.publish(output);

  pcl::toPCLPointCloud2(*debug_blue_cloud_ptr, pcl_pc);
  pcl_conversions::fromPCL(pcl_pc, output);
  output.header.frame_id = "camera_depth_optical_frame";
  output.header.stamp = ros::Time::now();
  blue_marker_cloud_pub_.publish(output);
};
