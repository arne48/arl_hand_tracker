// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved
// modified by Arne
#include <arl_hand_tracker/realsense_zr300.h>

RealsenseZR300::RealsenseZR300() :
cameraStarted_(false),
intialize_time_base_(false)
{
  ros::NodeHandle n("arl_marker_tracker_node");
  pnh_ = n;

  // Types for depth stream
  format_[rs::stream::depth] = rs::format::z16;   // libRS type
  image_format_[rs::stream::depth] = CV_16UC1;    // CVBridge type
  encoding_[rs::stream::depth] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
  unit_step_size_[rs::stream::depth] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
  stream_name_[rs::stream::depth] = "depth";

  // Types for color stream
  format_[rs::stream::color] = rs::format::rgb8;  // libRS type
  image_format_[rs::stream::color] = CV_8UC3;     // CVBridge type
  encoding_[rs::stream::color] = sensor_msgs::image_encodings::RGB8; // ROS message type
  unit_step_size_[rs::stream::color] = sizeof(unsigned char) * 3; // sensor_msgs::ImagePtr row step size
  stream_name_[rs::stream::color] = "color";

  // Types for fisheye stream
  format_[rs::stream::fisheye] = rs::format::raw8;  // libRS type
  image_format_[rs::stream::fisheye] = CV_8UC1;     // CVBridge type
  encoding_[rs::stream::fisheye] = sensor_msgs::image_encodings::MONO8; // ROS message type
  unit_step_size_[rs::stream::fisheye] = sizeof(unsigned char); // sensor_msgs::ImagePtr row step size
  stream_name_[rs::stream::fisheye] = "fisheye";

  publishMarkers_ = false;

  getParameters();

  if (!setupDevice())
    return;

  setupPublishers();
  setupStreams();
  publishStaticTransforms();

  tracker = new MarkerTracker(device_, node_handle);
  cameraStarted_ = true;
}

RealsenseZR300::~RealsenseZR300()
{}

void RealsenseZR300::disableRealsense()
{
  device_->stop(rs::source::all_sources);
  sleep(3);
  ctx_.reset();
  delete tracker;
}

void RealsenseZR300::getParameters()
{
  pnh_.param("serial_no", serial_no_, realsense_ros_camera::DEFAULT_SERIAL_NO);

  pnh_.param("depth_width", width_[rs::stream::depth], realsense_ros_camera::DEPTH_WIDTH);
  pnh_.param("depth_height", height_[rs::stream::depth], realsense_ros_camera::DEPTH_HEIGHT);
  pnh_.param("depth_fps", fps_[rs::stream::depth], realsense_ros_camera::DEPTH_FPS);
  pnh_.param("enable_depth", enable_[rs::stream::depth], true);

  pnh_.param("color_width", width_[rs::stream::color], realsense_ros_camera::COLOR_WIDTH);
  pnh_.param("color_height", height_[rs::stream::color], realsense_ros_camera::COLOR_HEIGHT);
  pnh_.param("color_fps", fps_[rs::stream::color], realsense_ros_camera::COLOR_FPS);
  pnh_.param("enable_color", enable_[rs::stream::color], true);
  pnh_.param("publish_undistorted_color", publish_undistorted_color_, true);

  pnh_.param("fisheye_width", width_[rs::stream::fisheye], realsense_ros_camera::FISHEYE_WIDTH);
  pnh_.param("fisheye_height", height_[rs::stream::fisheye], realsense_ros_camera::FISHEYE_HEIGHT);
  pnh_.param("fisheye_fps", fps_[rs::stream::fisheye], realsense_ros_camera::FISHEYE_FPS);
  pnh_.param("enable_fisheye", enable_[rs::stream::fisheye], true);

  pnh_.param("base_frame_id", base_frame_id_, realsense_ros_camera::DEFAULT_BASE_FRAME_ID);
  pnh_.param("depth_frame_id", frame_id_[rs::stream::depth], realsense_ros_camera::DEFAULT_DEPTH_FRAME_ID);
  pnh_.param("color_frame_id", frame_id_[rs::stream::color], realsense_ros_camera::DEFAULT_COLOR_FRAME_ID);
  pnh_.param("fisheye_frame_id", frame_id_[rs::stream::fisheye], realsense_ros_camera::DEFAULT_FISHEYE_FRAME_ID);
  pnh_.param("imu_frame_id", imu_frame_id_, realsense_ros_camera::DEFAULT_IMU_FRAME_ID);
  pnh_.param("depth_optical_frame_id", optical_frame_id_[rs::stream::depth], realsense_ros_camera::DEFAULT_DEPTH_OPTICAL_FRAME_ID);
  pnh_.param("color_optical_frame_id", optical_frame_id_[rs::stream::color], realsense_ros_camera::DEFAULT_COLOR_OPTICAL_FRAME_ID);
  pnh_.param("fisheye_optical_frame_id", optical_frame_id_[rs::stream::fisheye], realsense_ros_camera::DEFAULT_FISHEYE_OPTICAL_FRAME_ID);
  pnh_.param("imu_optical_frame_id", optical_imu_frame_id_, realsense_ros_camera::DEFAULT_IMU_OPTICAL_FRAME_ID);

  optical_imu_id_[0] = "imu_accel_frame_id";
  optical_imu_id_[1] = "imu_gyro_frame_id";
}//end getParameters


bool RealsenseZR300::setupDevice()
{
  ctx_.reset(new rs::context());

  sleep(3);

  int num_of_cams = ctx_ -> get_device_count();
  if (num_of_cams == 0)
  {
    ROS_ERROR("error : no RealSense ZR300 devices found.");
    ctx_.reset();
    return false;
  }

  rs::device *detected_dev;
  for (int i = 0; i < num_of_cams; i++)
  {
    detected_dev = ctx_->get_device(i);
    detected_dev->get_serial();
    if (serial_no_.empty() || (serial_no_ == std::string(detected_dev->get_serial())))
    {
      device_ = detected_dev;
      break;
    }
  }

  if (device_ == nullptr)
  {
    ROS_ERROR_STREAM("error: No RealSense device with serial_no = " << serial_no_ << " found.");
    ctx_.reset();
    return false;
  }

  auto device_name = device_ -> get_name();
  ROS_INFO_STREAM(device_name << ", serial_no: " << std::string(device_ ->get_serial()));
  if (std::string(device_name).find("ZR300") == std::string::npos)
  {
      ROS_ERROR_STREAM("error: This ROS node supports ZR300 only.");
      ctx_.reset();
      return false;
  }

  return true;
}//end setupDevice


void RealsenseZR300::setupPublishers()
{
  image_transport::ImageTransport image_transport(node_handle);

  // Stream publishers and latched topics
  if (true == enable_[rs::stream::color])
  {
    image_publishers_[rs::stream::color] = image_transport.advertise("camera/color/image_raw", 1);
    info_publisher_[rs::stream::color] =
        node_handle.advertise< sensor_msgs::CameraInfo >("camera/color/camera_info", 1);

    if (publish_undistorted_color_)
    {
      undist_color_publisher_ = image_transport.advertise("camera/color/image_rect", 1);
    }
  }

  if (true == enable_[rs::stream::depth])
  {
    image_publishers_[rs::stream::depth] = image_transport.advertise("camera/depth/image_raw", 1);
    info_publisher_[rs::stream::depth] =
        node_handle.advertise< sensor_msgs::CameraInfo >("camera/depth/camera_info", 1);

    pointcloud_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/camera/points", 1);
  }

  if (true == enable_[rs::stream::fisheye])
  {
    image_publishers_[rs::stream::fisheye] = image_transport.advertise("camera/fisheye/image_raw", 1);
    info_publisher_[rs::stream::fisheye] =
        node_handle.advertise<sensor_msgs::CameraInfo>("camera/fisheye/camera_info", 1);
  }

  imu_publishers_[RS_EVENT_IMU_GYRO] = node_handle.advertise< sensor_msgs::Imu >("camera/gyro/sample", 100);
  imu_publishers_[RS_EVENT_IMU_ACCEL] = node_handle.advertise< sensor_msgs::Imu >("camera/accel/sample", 100);

  // Latched topics
  if (true == enable_[rs::stream::fisheye])
  {
    fe2imu_publisher_ = node_handle.advertise<arl_hand_tracker::Extrinsics>("camera/extrinsics/fisheye2imu", 1, true);
    fe2depth_publisher_ = node_handle.advertise<arl_hand_tracker::Extrinsics>("camera/extrinsics/fisheye2depth", 1, true);
  }
  accelInfo_publisher_ = node_handle.advertise<arl_hand_tracker::IMUInfo>("camera/accel/imu_info", 1, true);
  gyroInfo_publisher_ = node_handle.advertise<arl_hand_tracker::IMUInfo>("camera/gyro/imu_info", 1, true);

}//end setupPublishers

void RealsenseZR300::initUndistortionParams(rs::stream stream)
{
  cv::Matx33d K;
  K(0, 0) = camera_info_[stream].K[0];
  K(0, 1) = camera_info_[stream].K[1];
  K(0, 2) = camera_info_[stream].K[2];
  K(1, 0) = camera_info_[stream].K[3];
  K(1, 1) = camera_info_[stream].K[4];
  K(1, 2) = camera_info_[stream].K[5];
  K(2, 2) = 1.0;

  cv::Mat map_x, map_y, R;
  R = cv::Mat::eye(3, 3, CV_32S);

  cv::initUndistortRectifyMap(K, camera_info_[stream].D, R, K,
                              cv::Size(width_[stream], height_[stream]),
                              CV_32FC1, map_x, map_y);

  remap_parameter_[stream][0] = map_x;
  remap_parameter_[stream][1] = map_y;
}

void RealsenseZR300::registerVisualLambdaCallback(rs::stream stream)
{
  // Define lambda callback for receiving stream data
  device_->set_frame_callback(stream, [this,stream](rs::frame frame)
  {
    image_[stream].data = (unsigned char *) frame.get_data();

    if (stream == rs::stream::color && publishMarkers_)
    {
      tracker->publishMarker(image_[stream], image_[rs::stream::depth]);
    }

    if (rs::timestamp_domain::microcontroller != frame.get_frame_timestamp_domain())
    {
      ROS_ERROR_STREAM("error: Junk time stamp in stream:" << (int) (stream) <<
                                                           "\twith frame counter:" << frame.get_frame_number());
      return;
    }

    // We compute a ROS timestamp which is based on an initial ROS time at point of first frame,
    // and the incremental timestamp from the camera.
    if (!intialize_time_base_)
    {
      intialize_time_base_ = true;
      ros_time_base_ = ros::Time::now();
      camera_time_base_ = frame.get_timestamp();
    }
    double elapsed_camera_ms = (/*ms*/ frame.get_timestamp() - /*ms*/ camera_time_base_) / /*ms to seconds*/ 1000;
    ros::Time t(ros_time_base_.toSec() + elapsed_camera_ms);

    // If this stream is associated with depth and we have at least one point cloud subscriber,
    // service it here.
    if ((stream == rs::stream::depth) && (0 != pointcloud_publisher_.getNumSubscribers()))
      publishPCTopic(t);

    seq_[stream] += 1;
    if (info_publisher_[stream].getNumSubscribers() ||
        image_publishers_[stream].getNumSubscribers())
    {
      sensor_msgs::ImagePtr img;
      img = cv_bridge::CvImage(std_msgs::Header(), encoding_[stream], image_[stream]).toImageMsg();
      img->width = (uint32_t)image_[stream].cols;
      img->height = (uint32_t)image_[stream].rows;
      img->is_bigendian = 0;
      img->step = image_[stream].cols * unit_step_size_[stream];
      img->header.frame_id = optical_frame_id_[stream];
      img->header.stamp = t;
      img->header.seq = seq_[stream];

      image_publishers_[stream].publish(img);

      camera_info_[stream].header.stamp = t;
      camera_info_[stream].header.seq = seq_[stream];
      info_publisher_[stream].publish(camera_info_[stream]);
    }

    // Publishing undistorted color image
    if (stream == rs::stream::color && publish_undistorted_color_ &&
        0 != undist_color_publisher_.getNumSubscribers())
    {
      sensor_msgs::ImagePtr img;
      cv::Mat dst;
      const cv::Mat src = image_[stream];

      cv::remap(src, dst, remap_parameter_[rs::stream::color][0], remap_parameter_[rs::stream::color][1],
                cv::INTER_LINEAR, cv::BORDER_CONSTANT);

      img = cv_bridge::CvImage(std_msgs::Header(), encoding_[stream], dst).toImageMsg();
      img->width = (uint32_t)image_[stream].cols;
      img->height = (uint32_t)image_[stream].rows;
      img->is_bigendian = 0;
      img->step = image_[stream].cols * unit_step_size_[stream];
      img->header.frame_id = optical_frame_id_[stream];
      img->header.stamp = t;
      img->header.seq = seq_[stream];

      undist_color_publisher_.publish(img);
    }
  });
}

std::function<void(rs::motion_data)> RealsenseZR300::getImuCallback()
{
  return [this](rs::motion_data entry)
  {
    if ((entry.timestamp_data.source_id != RS_EVENT_IMU_GYRO) &&
        (entry.timestamp_data.source_id != RS_EVENT_IMU_ACCEL))
      return;

    rs_event_source motionType = entry.timestamp_data.source_id;

    // If there is nobody subscribed to the stream, do no further
    // processing
    if(!imu_publishers_[motionType].getNumSubscribers() || !intialize_time_base_)
    {
      return;
    }

    double elapsed_camera_ms = (/*ms*/ entry.timestamp_data.timestamp - /*ms*/ camera_time_base_) / /*ms to seconds*/ 1000;
    ros::Time t(ros_time_base_.toSec() + elapsed_camera_ms);

    sensor_msgs::Imu imu_msg = sensor_msgs::Imu();
    imu_msg.header.frame_id = optical_imu_id_[motionType];
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;
    imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (motionType == RS_EVENT_IMU_GYRO)
    {
      imu_msg.angular_velocity.x = entry.axes[0];
      imu_msg.angular_velocity.y = entry.axes[1];
      imu_msg.angular_velocity.z = entry.axes[2];
    }
    if (motionType == RS_EVENT_IMU_ACCEL)
    {
      imu_msg.linear_acceleration.x = entry.axes[0];
      imu_msg.linear_acceleration.y = entry.axes[1];
      imu_msg.linear_acceleration.z = entry.axes[2];
    }
    seq_motion[motionType] += 1;
    imu_msg.header.seq = seq_motion[motionType];
    imu_msg.header.stamp = t;
    imu_publishers_[motionType].publish(imu_msg);
  };
}

void RealsenseZR300::setupStreams()
{
  device_->set_option(rs::option::r200_lr_auto_exposure_enabled, 1);

  const rs::stream All[] = { rs::stream::depth, rs::stream::color, rs::stream::fisheye };
  for (const auto stream : All)
  {

    if (false == enable_[stream])
      continue;

    // Enable the stream
    device_->enable_stream(stream, width_[stream], height_[stream], format_[stream], fps_[stream]);

    // Publish info about the stream
    getStreamCalibData(stream);

    // Initialize Parameters to undistort the camera image via remapping
    if (stream == rs::stream::color)
    {
      initUndistortionParams(stream);
    }


    // Setup stream callback for stream
    image_[stream] = cv::Mat(camera_info_[stream].height, camera_info_[stream].width,
                             image_format_[stream], cv::Scalar(0, 0, 0));

    registerVisualLambdaCallback(stream);

    ROS_INFO_STREAM("  enabled " << stream_name_[stream] << " stream, width: "
                                 << camera_info_[stream].width << " height: " << camera_info_[stream].height
                                 << " fps: " << fps_[stream]);
  }//end for


  // Needed to align image timestamps to common clock-domain with the motion events
  device_->set_option(rs::option::fisheye_strobe, 1);

  // This option causes the fisheye image to be aquired in-sync with the depth image.
  device_->set_option(rs::option::fisheye_external_trigger, 1);
  device_->set_option(rs::option::fisheye_color_auto_exposure, 1);
  seq_motion[RS_EVENT_IMU_GYRO] = 0;
  seq_motion[RS_EVENT_IMU_ACCEL] = 0;


  device_->enable_motion_tracking(getImuCallback(), [](rs::timestamp_data entry) {});
  ROS_INFO_STREAM("  enabled accel and gyro stream");

  // publish ZR300-specific intrinsics/extrinsics
  arl_hand_tracker::IMUInfo accelInfo, gyroInfo;
  getImuInfo(device_, accelInfo, gyroInfo);

  fe2imu_publisher_.publish(getFisheye2ImuExtrinsicsMsg(device_));
  fe2depth_publisher_.publish(getFisheye2DepthExtrinsicsMsg(device_));
  accelInfo_publisher_.publish(accelInfo);
  gyroInfo_publisher_.publish(gyroInfo);


  device_->start(rs::source::all_sources);
  cameraStarted_ = true;
}//end setupStreams

void RealsenseZR300::getStreamCalibData(rs::stream stream)
{
  rs::intrinsics intrinsic = device_->get_stream_intrinsics(stream);

  camera_info_[stream].header.frame_id = optical_frame_id_[stream];
  camera_info_[stream].width = (uint32_t)intrinsic.width;
  camera_info_[stream].height = (uint32_t)intrinsic.height;

  camera_info_[stream].K.at(0) = intrinsic.fx;
  camera_info_[stream].K.at(2) = intrinsic.ppx;
  camera_info_[stream].K.at(4) = intrinsic.fy;
  camera_info_[stream].K.at(5) = intrinsic.ppy;
  camera_info_[stream].K.at(8) = 1;

  camera_info_[stream].P.at(0) = camera_info_[stream].K.at(0);
  camera_info_[stream].P.at(1) = 0;
  camera_info_[stream].P.at(2) = camera_info_[stream].K.at(2);
  camera_info_[stream].P.at(3) = 0;
  camera_info_[stream].P.at(4) = 0;
  camera_info_[stream].P.at(5) = camera_info_[stream].K.at(4);
  camera_info_[stream].P.at(6) = camera_info_[stream].K.at(5);
  camera_info_[stream].P.at(7) = 0;
  camera_info_[stream].P.at(8) = 0;
  camera_info_[stream].P.at(9) = 0;
  camera_info_[stream].P.at(10) = 1;
  camera_info_[stream].P.at(11) = 0;

  if (stream == rs::stream::depth)
  {
    // set depth to color translation values in Projection matrix (P)
    rs::extrinsics extrinsic = device_->get_extrinsics(rs::stream::depth, rs::stream::color);
    camera_info_[stream].P.at(3) = extrinsic.translation[0];     // Tx
    camera_info_[stream].P.at(7) = extrinsic.translation[1];     // Ty
    camera_info_[stream].P.at(11) = extrinsic.translation[2];    // Tz

    for (int i = 0; i < 9; i++)
      camera_info_[stream].R.at(i) = extrinsic.rotation[i];
  }

  switch ((int32_t)intrinsic.model())
  {
    case 0:
      camera_info_[stream].distortion_model = "plumb_bob";
      break;
    case 1:
      // This is the same as "modified_brown_conrady", but used by ROS
      camera_info_[stream].distortion_model = "plumb_bob";
      break;
    case 2:
      camera_info_[stream].distortion_model = "inverse_brown_conrady";
      break;
    case 3:
      camera_info_[stream].distortion_model = "distortion_ftheta";
      break;
    default:
      camera_info_[stream].distortion_model = "others";
      break;
  }

  // set R (rotation matrix) values to identity matrix
  if (stream != rs::stream::depth)
  {
    camera_info_[stream].R.at(0) = 1.0;
    camera_info_[stream].R.at(1) = 0.0;
    camera_info_[stream].R.at(2) = 0.0;
    camera_info_[stream].R.at(3) = 0.0;
    camera_info_[stream].R.at(4) = 1.0;
    camera_info_[stream].R.at(5) = 0.0;
    camera_info_[stream].R.at(6) = 0.0;
    camera_info_[stream].R.at(7) = 0.0;
    camera_info_[stream].R.at(8) = 1.0;
  }

  for (int i = 0; i < 5; i++)
  {
    camera_info_[stream].D.push_back(intrinsic.coeffs[i]);
  }
}//end getStreamCalibData


void RealsenseZR300::publishStaticTransforms()
{
  // Publish transforms for the cameras
  tf::Quaternion q_c2co;
  tf::Quaternion q_d2do;
  tf::Quaternion q_i2io;
  geometry_msgs::TransformStamped b2d_msg;
  geometry_msgs::TransformStamped d2do_msg;
  geometry_msgs::TransformStamped b2c_msg;
  geometry_msgs::TransformStamped c2co_msg;
  geometry_msgs::TransformStamped b2i_msg;
  geometry_msgs::TransformStamped i2io_msg;

  // Get the current timestamp for all static transforms
  ros::Time transform_ts_ = ros::Time::now();

  // The color frame is used as the base frame.
  // Hence no additional transformation is done from base frame to color frame.
  b2c_msg.header.stamp = transform_ts_;
  b2c_msg.header.frame_id = base_frame_id_;
  b2c_msg.child_frame_id = frame_id_[rs::stream::color];
  b2c_msg.transform.translation.x = 0;
  b2c_msg.transform.translation.y = 0;
  b2c_msg.transform.translation.z = 0;
  b2c_msg.transform.rotation.x = 0;
  b2c_msg.transform.rotation.y = 0;
  b2c_msg.transform.rotation.z = 0;
  b2c_msg.transform.rotation.w = 1;
  static_tf_broadcaster_.sendTransform(b2c_msg);

  // Transform color frame to color optical frame
  q_c2co.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  c2co_msg.header.stamp = transform_ts_;
  c2co_msg.header.frame_id = frame_id_[rs::stream::color];
  c2co_msg.child_frame_id = optical_frame_id_[rs::stream::color];
  c2co_msg.transform.translation.x = 0;
  c2co_msg.transform.translation.y = 0;
  c2co_msg.transform.translation.z = 0;
  c2co_msg.transform.rotation.x = q_c2co.getX();
  c2co_msg.transform.rotation.y = q_c2co.getY();
  c2co_msg.transform.rotation.z = q_c2co.getZ();
  c2co_msg.transform.rotation.w = q_c2co.getW();
  static_tf_broadcaster_.sendTransform(c2co_msg);

  // Transform base frame to depth frame
  rs::extrinsics color2depth_extrinsic = device_->get_extrinsics(rs::stream::color, rs::stream::depth);
  b2d_msg.header.stamp = transform_ts_;
  b2d_msg.header.frame_id = base_frame_id_;
  b2d_msg.child_frame_id = frame_id_[rs::stream::depth];
  b2d_msg.transform.translation.x =  color2depth_extrinsic.translation[2];
  b2d_msg.transform.translation.y = -color2depth_extrinsic.translation[0];
  b2d_msg.transform.translation.z = -color2depth_extrinsic.translation[1];
  b2d_msg.transform.rotation.x = 0;
  b2d_msg.transform.rotation.y = 0;
  b2d_msg.transform.rotation.z = 0;
  b2d_msg.transform.rotation.w = 1;
  static_tf_broadcaster_.sendTransform(b2d_msg);

  // Transform depth frame to depth optical frame
  q_d2do.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  d2do_msg.header.stamp = transform_ts_;
  d2do_msg.header.frame_id = frame_id_[rs::stream::depth];
  d2do_msg.child_frame_id = optical_frame_id_[rs::stream::depth];
  d2do_msg.transform.translation.x = 0;
  d2do_msg.transform.translation.y = 0;
  d2do_msg.transform.translation.z = 0;
  d2do_msg.transform.rotation.x = q_d2do.getX();
  d2do_msg.transform.rotation.y = q_d2do.getY();
  d2do_msg.transform.rotation.z = q_d2do.getZ();
  d2do_msg.transform.rotation.w = q_d2do.getW();
  static_tf_broadcaster_.sendTransform(d2do_msg);


  tf::Quaternion q_f2fo, q_imu2imuo;
  geometry_msgs::TransformStamped b2f_msg;
  geometry_msgs::TransformStamped f2fo_msg;
  geometry_msgs::TransformStamped b2imu_msg;
  geometry_msgs::TransformStamped imu2imuo_msg;

  // Transform base frame to fisheye frame
  b2f_msg.header.stamp = transform_ts_;
  b2f_msg.header.frame_id = base_frame_id_;
  b2f_msg.child_frame_id = frame_id_[rs::stream::fisheye];
  rs::extrinsics color2fisheye_extrinsic = device_->get_extrinsics(rs::stream::color, rs::stream::fisheye);
  b2f_msg.transform.translation.x =  color2fisheye_extrinsic.translation[2];
  b2f_msg.transform.translation.y = -color2fisheye_extrinsic.translation[0];
  b2f_msg.transform.translation.z = -color2fisheye_extrinsic.translation[1];
  b2f_msg.transform.rotation.x = 0;
  b2f_msg.transform.rotation.y = 0;
  b2f_msg.transform.rotation.z = 0;
  b2f_msg.transform.rotation.w = 1;
  static_tf_broadcaster_.sendTransform(b2f_msg);

  // Transform fisheye frame to fisheye optical frame
  q_f2fo.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  f2fo_msg.header.stamp = transform_ts_;
  f2fo_msg.header.frame_id = frame_id_[rs::stream::fisheye];
  f2fo_msg.child_frame_id = optical_frame_id_[rs::stream::fisheye];
  f2fo_msg.transform.translation.x = 0;
  f2fo_msg.transform.translation.y = 0;
  f2fo_msg.transform.translation.z = 0;
  f2fo_msg.transform.rotation.x = q_f2fo.getX();
  f2fo_msg.transform.rotation.y = q_f2fo.getY();
  f2fo_msg.transform.rotation.z = q_f2fo.getZ();
  f2fo_msg.transform.rotation.w = q_f2fo.getW();
  static_tf_broadcaster_.sendTransform(f2fo_msg);

}

float RealsenseZR300::rgbFromTexCoord(cv::Mat tex, struct rs::float2 coord, rs::intrinsics tex_intrinsics)
{
  auto pixel_x = (int)(coord.x * tex_intrinsics.width);
  if (pixel_x < 0) pixel_x = 0;
  if (pixel_x >= tex_intrinsics.width) pixel_x = tex_intrinsics.width - 1;

  auto pixel_y = (int)(coord.y * tex_intrinsics.height);
  if (pixel_y < 0) pixel_y = 0;
  if (pixel_y >= tex_intrinsics.height) pixel_y = tex_intrinsics.height -1;

  cv::Vec3b pixel_vec = tex.at<cv::Vec3b>(pixel_y, pixel_x);

  return pixel_vec.val[2] | pixel_vec.val[1] << 8 | pixel_vec.val[0] << 16;
}

void RealsenseZR300::publishPCTopic(ros::Time t)
{
  rs::intrinsics depth_intrinsic = device_->get_stream_intrinsics(rs::stream::depth);
  rs::intrinsics color_intrinsic = device_->get_stream_intrinsics(rs::stream::color);
  rs::extrinsics depth_extrinsics = device_->get_extrinsics(rs::stream::depth, rs::stream::color);
  float depth_scale_meters = device_->get_depth_scale();
  cv::Mat latest_color_frame = image_[rs::stream::color];
  cv::Mat latest_depth_frame = image_[rs::stream::depth];

  sensor_msgs::PointCloud2 msg_pointcloud;
  msg_pointcloud.header.stamp = t;
  msg_pointcloud.header.frame_id = optical_frame_id_[rs::stream::depth];
  msg_pointcloud.width = (uint32_t)depth_intrinsic.width;
  msg_pointcloud.height = (uint32_t)depth_intrinsic.height;
  msg_pointcloud.is_dense = 1;

  sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);
  modifier.setPointCloud2Fields(4,
                                "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32,
                                "rgb", 1, sensor_msgs::PointField::FLOAT32);

  for (int v = 0; v < depth_intrinsic.height; v++)
  {
    for (int u = 0; u < depth_intrinsic.width; u++)
    {
      float depth_point[3], scaled_depth;
      uint32_t pixel_color_value;
      uint16_t depth_value;
      int depth_offset, cloud_offset;

      // Offset into point cloud data, for point at u, v
      cloud_offset = (v * msg_pointcloud.row_step) + (u * msg_pointcloud.point_step);

      // Retrieve depth value, and scale it in terms of meters
      depth_offset = (u * sizeof(uint16_t)) + (v * sizeof(uint16_t) * depth_intrinsic.width);
      memcpy(&depth_value, &latest_depth_frame.data[depth_offset], sizeof(uint16_t));
      scaled_depth = static_cast<float>(depth_value) * depth_scale_meters;
      if (scaled_depth <= 0.0f || scaled_depth > realsense_ros_camera::MAX_Z)
      {
        // Depth value is invalid, so zero it out.
        depth_point[0] = 0.0f;
        depth_point[1] = 0.0f;
        depth_point[2] = 0.0f;
        pixel_color_value = 0;
      } else
      {
        // Convert depth image to points in 3D space
        float depth_pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
        rs_deproject_pixel_to_point(depth_point, &depth_intrinsic, depth_pixel, scaled_depth);

        struct rs::float2 tex_coord = color_intrinsic.project_to_texcoord(depth_extrinsics.transform({depth_point[0],
                                                                                                      depth_point[1],
                                                                                                      depth_point[2]}));
        pixel_color_value = rgbFromTexCoord(latest_color_frame, tex_coord, color_intrinsic);
      }

      // Assign 3d point and rgb color
      memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[0].offset], &depth_point[0], sizeof(float)); // X
      memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[1].offset], &depth_point[1], sizeof(float)); // Y
      memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[2].offset], &depth_point[2], sizeof(float)); // Z
      memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[3].offset], &pixel_color_value,
             sizeof(float)); // RGB
    }
  }

  pointcloud_publisher_.publish(msg_pointcloud);
}

void RealsenseZR300::getImuInfo(rs::device* device, arl_hand_tracker::IMUInfo &accelInfo, arl_hand_tracker::IMUInfo &gyroInfo)
{
  rs::motion_intrinsics imuIntrinsics = device->get_motion_intrinsics();

  accelInfo.header.frame_id = "imu_accel";
  int index = 0;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      accelInfo.data[index] = imuIntrinsics.acc.data[i][j];
      ++index;
    }
    accelInfo.noise_variances[i] = imuIntrinsics.acc.noise_variances[i];
    accelInfo.bias_variances[i] = imuIntrinsics.acc.bias_variances[i];
  }

  gyroInfo.header.frame_id = "imu_gyro";
  index = 0;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      gyroInfo.data[index] = imuIntrinsics.gyro.data[i][j];
      ++index;
    }
    gyroInfo.noise_variances[i] = imuIntrinsics.gyro.noise_variances[i];
    gyroInfo.bias_variances[i] = imuIntrinsics.gyro.bias_variances[i];
  }
}

arl_hand_tracker::Extrinsics RealsenseZR300::rsExtrinsicsToMsg(rs::extrinsics rsExtrinsics)
{
  arl_hand_tracker::Extrinsics extrinsicsMsg;

  for (int i = 0; i < 9; ++i)
  {
    extrinsicsMsg.rotation[i] = rsExtrinsics.rotation[i];
    if (i < 3) extrinsicsMsg.translation[i] = rsExtrinsics.translation[i];
  }

  return extrinsicsMsg;
}

arl_hand_tracker::Extrinsics RealsenseZR300::getFisheye2ImuExtrinsicsMsg(rs::device* device)
{
  arl_hand_tracker::Extrinsics extrinsicsMsg = rsExtrinsicsToMsg(device->get_motion_extrinsics_from(rs::stream::fisheye));
  extrinsicsMsg.header.frame_id = "fisheye2imu_extrinsics";
  return extrinsicsMsg;
}

arl_hand_tracker::Extrinsics RealsenseZR300::getFisheye2DepthExtrinsicsMsg(rs::device* device)
{
  arl_hand_tracker::Extrinsics extrinsicsMsg =  rsExtrinsicsToMsg(device->get_extrinsics(rs::stream::depth, rs::stream::fisheye));
  extrinsicsMsg.header.frame_id = "fisheye2depth_extrinsics";
  return extrinsicsMsg;
}