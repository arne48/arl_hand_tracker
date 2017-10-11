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
  marker_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/arl_marker_tracker/marker_cloud", 1);
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

cv::Vec3b MarkerTracker::rgbFromTexCoord(cv::Mat tex, struct rs::float2 coord, rs::intrinsics tex_intrinsics)
{
  auto pixel_x = (int)(coord.x * tex_intrinsics.width);
  if (pixel_x < 0) pixel_x = 0;
  if (pixel_x >= tex_intrinsics.width) pixel_x = tex_intrinsics.width - 1;

  auto pixel_y = (int)(coord.y * tex_intrinsics.height);
  if (pixel_y < 0) pixel_y = 0;
  if (pixel_y >= tex_intrinsics.height) pixel_y = tex_intrinsics.height -1;

  return tex.at<cv::Vec3b>(pixel_y, pixel_x);
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


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

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
        if(getFromTexCoord(msg_img, tex_coord, color_intrinsic_))
        {
          pcl::PointXYZRGB point;
          point.x = depth_point[0];
          point.y = depth_point[1];
          point.z = depth_point[2];

          cv::Vec3b pixel_vec = rgbFromTexCoord(color_frame, tex_coord, color_intrinsic_);
          point.r = pixel_vec[0];
          point.g = pixel_vec[1];
          point.b = pixel_vec[2];

          cloud_ptr->push_back(point);
        }
      }
    }
  }

  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 pcl_pc;
  pcl::toPCLPointCloud2(*cloud_ptr, pcl_pc);
  pcl_conversions::fromPCL(pcl_pc, output);
  output.header.frame_id = "camera_depth_optical_frame";
  output.header.stamp = ros::Time::now();
  marker_cloud_pub_.publish(output);
};