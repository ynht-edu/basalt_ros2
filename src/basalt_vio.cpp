#include "basalt_ros/basalt_vio.hpp"

#include <basalt/serialization/headers_serialization.h>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace basalt_ros
{

namespace
{

int64_t to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

builtin_interfaces::msg::Time to_stamp(int64_t t_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(t_ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(t_ns % 1000000000LL);
  return stamp;
}

std::string frame_prefix(const std::string & ns)
{
  const auto name = ns.substr(ns.find_last_of('/') + 1);
  return name.empty() ? "" : name + "/";
}

}  // namespace

BasaltVio::BasaltVio(const rclcpp::NodeOptions & options)
: Node("basalt_vio", options)
{
  const auto prefix = frame_prefix(get_namespace());
  const auto calibration_file = declare_parameter<std::string>("calibration.file", "");
  const auto config_file = declare_parameter<std::string>("config.file", "");
  const auto use_double = declare_parameter<bool>("estimator.use_double", false);
  const auto imu_topic = declare_parameter<std::string>("input.imu_topic", "camera/imu");
  const auto left_topic = declare_parameter<std::string>("input.left_topic", "camera/infra1/image_rect_raw");
  const auto right_topic = declare_parameter<std::string>("input.right_topic", "camera/infra2/image_rect_raw");
  const auto odom_topic = declare_parameter<std::string>("output.odom_topic", "basalt/odom_imu");
  const auto points_topic = declare_parameter<std::string>("output.points_topic", "basalt/points");
  const auto depth_topic = declare_parameter<std::string>("output.depth_topic", "basalt/loop_depth");
  const auto depth_colored_topic =
    declare_parameter<std::string>("output.depth_colored_topic", "basalt/loop_depth_colored");
  world_frame_ = declare_parameter<std::string>("frame.world", prefix + "vio_world");
  imu_frame_ = declare_parameter<std::string>("frame.imu", prefix + "camera_imu_optical_frame");
  camera_frame_ = declare_parameter<std::string>("frame.camera", prefix + "camera_infra1_optical_frame");
  init_window_ = declare_parameter<double>("init.window", 1.0);
  init_max_accel_std_ = declare_parameter<double>("init.max_accel_std", 0.2);
  init_max_gyro_std_ = declare_parameter<double>("init.max_gyro_std", 0.05);

  load_calibration(calibration_file);
  config_.load(config_file);

  flow_ = basalt::OpticalFlowFactory::getOpticalFlow(config_, calib_);
  vio_ = basalt::VioEstimatorFactory::getVioEstimator(config_, calib_, basalt::constants::g, true, use_double);
  flow_->output_queue = &vio_->vision_data_queue;
  state_queue_.set_capacity(100);
  vis_queue_.set_capacity(10);
  vio_->out_state_queue = &state_queue_;
  vio_->out_vis_queue = &vis_queue_;

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(points_topic, rclcpp::SensorDataQoS());
  depth_pub_ = create_publisher<Image>(depth_topic, 2);
  depth_colored_pub_ = create_publisher<Image>(depth_colored_topic, 2);

  state_thread_ = std::thread([this]() {state_loop();});
  points_thread_ = std::thread([this]() {points_loop();});

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS().keep_last(400),
    [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {imu_cb(msg);});
  left_sub_.subscribe(this, left_topic, rmw_qos_profile_sensor_data);
  right_sub_.subscribe(this, right_topic, rmw_qos_profile_sensor_data);
  stereo_sync_ = std::make_shared<StereoSync>(left_sub_, right_sub_, 10);
  stereo_sync_->registerCallback(&BasaltVio::stereo_cb, this);

  RCLCPP_INFO(
    get_logger(), "basalt vio (%s) on %s + %s, imu %s -> %s",
    use_double ? "double" : "float", left_topic.c_str(), right_topic.c_str(), imu_topic.c_str(),
    odom_topic.c_str());
}

BasaltVio::~BasaltVio()
{
  stop();
}

void BasaltVio::load_calibration(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("could not open calibration " + path);
  }
  cereal::JSONInputArchive archive(file);
  archive(calib_);
  if (calib_.intrinsics.size() != 2) {
    throw std::runtime_error("calibration needs 2 cameras, got " + std::to_string(calib_.intrinsics.size()));
  }
}

void BasaltVio::imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const auto t_ns = to_ns(msg->header.stamp);
  if (t_ns <= last_imu_ns_) {
    ++imu_dropped_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "dropped %zu imu samples so far, last one %s by %.4f s",
      imu_dropped_, t_ns == last_imu_ns_ ? "duplicate" : "back in time", (last_imu_ns_ - t_ns) * 1e-9);
    return;
  }
  last_imu_ns_ = t_ns;

  auto data = std::make_shared<basalt::ImuData<double>>();
  data->t_ns = t_ns;
  data->accel << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  data->gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  {
    std::lock_guard<std::mutex> lock(gyro_mutex_);
    last_gyro_ = data->gyro;
  }

  if (!initialized_) {
    imu_window_.push_back(data);
    const auto keep_ns = static_cast<int64_t>((init_window_ + 0.5) * 1e9);
    while (!imu_window_.empty() && t_ns - imu_window_.front()->t_ns > keep_ns) {
      imu_window_.pop_front();
    }
    return;
  }

  if (!vio_->imu_data_queue.try_push(data)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "imu queue full, estimator is behind");
  }
}

void BasaltVio::stereo_cb(const Image::ConstSharedPtr & left, const Image::ConstSharedPtr & right)
{
  // basalt vio does not apply the calibrated camera to imu time offset itself
  const auto t_ns = to_ns(left->header.stamp) + calib_.cam_time_offset_ns;
  if (t_ns <= last_image_ns_) {
    return;
  }

  auto input = std::make_shared<basalt::OpticalFlowInput>();
  input->t_ns = t_ns;
  input->img_data.resize(2);
  if (!to_basalt(*left, input->img_data[0]) || !to_basalt(*right, input->img_data[1])) {
    return;
  }
  last_image_ns_ = t_ns;

  if (!initialized_ && !try_initialize(t_ns)) {
    return;
  }
  if (!flow_->input_queue.try_push(input)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "image queue full, frame dropped");
  }
}

bool BasaltVio::try_initialize(int64_t frame_ns)
{
  // like a static initializer: start only after the imu has been still for a whole window,
  // so gravity and the gyro bias come from an average instead of a single noisy sample
  const auto window_ns = static_cast<int64_t>(init_window_ * 1e9);
  std::vector<basalt::ImuData<double>::Ptr> still;
  for (const auto & sample : imu_window_) {
    if (sample->t_ns > frame_ns - window_ns && sample->t_ns <= frame_ns) {
      still.push_back(sample);
    }
  }
  if (still.size() < 10 || still.front()->t_ns > frame_ns - 0.9 * window_ns) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for %.1f s of imu data", init_window_);
    return false;
  }

  Eigen::Vector3d accel_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
  for (const auto & sample : still) {
    accel_mean += sample->accel;
    gyro_mean += sample->gyro;
  }
  accel_mean /= still.size();
  gyro_mean /= still.size();

  Eigen::Vector3d accel_var = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_var = Eigen::Vector3d::Zero();
  for (const auto & sample : still) {
    accel_var += (sample->accel - accel_mean).cwiseAbs2();
    gyro_var += (sample->gyro - gyro_mean).cwiseAbs2();
  }
  const double accel_std = std::sqrt(accel_var.sum() / still.size());
  const double gyro_std = std::sqrt(gyro_var.sum() / still.size());
  if (accel_std > init_max_accel_std_ || gyro_std > init_max_gyro_std_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "waiting for the vehicle to be still: accel std %.3f, gyro std %.4f",
      accel_std, gyro_std);
    return false;
  }

  vio_->initialize(gyro_mean, Eigen::Vector3d::Zero());

  auto average = std::make_shared<basalt::ImuData<double>>();
  average->t_ns = frame_ns;
  average->accel = accel_mean;
  average->gyro = gyro_mean;
  vio_->imu_data_queue.push(average);
  for (const auto & sample : imu_window_) {
    if (sample->t_ns > frame_ns) {
      vio_->imu_data_queue.push(sample);
    }
  }
  imu_window_.clear();
  initialized_ = true;

  RCLCPP_INFO(
    get_logger(), "initialized from %zu still imu samples: |g| %.3f, gyro bias [%.4f %.4f %.4f]",
    still.size(), accel_mean.norm(), gyro_mean.x(), gyro_mean.y(), gyro_mean.z());
  return true;
}

bool BasaltVio::to_basalt(const Image & msg, basalt::ImageData & out) const
{
  namespace enc = sensor_msgs::image_encodings;
  const bool is_8bit = msg.encoding == enc::MONO8 || msg.encoding == enc::TYPE_8UC1;
  const bool is_16bit = msg.encoding == enc::MONO16 || msg.encoding == enc::TYPE_16UC1;
  if (!is_8bit && !is_16bit) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "image encoding %s not supported, use mono8 or mono16",
      msg.encoding.c_str());
    return false;
  }

  out.img = std::make_shared<basalt::ManagedImage<uint16_t>>(msg.width, msg.height);
  for (uint32_t y = 0; y < msg.height; ++y) {
    const uint8_t * row = msg.data.data() + y * msg.step;
    uint16_t * dst = out.img->ptr + y * msg.width;
    for (uint32_t x = 0; x < msg.width; ++x) {
      if (is_8bit) {
        dst[x] = static_cast<uint16_t>(row[x]) << 8;
      } else {
        dst[x] = reinterpret_cast<const uint16_t *>(row)[x];
      }
    }
  }
  return true;
}

void BasaltVio::state_loop()
{
  while (running_) {
    basalt::PoseVelBiasState<double>::Ptr state;
    state_queue_.pop(state);
    if (!state) {
      break;
    }

    Eigen::Vector3d gyro;
    {
      std::lock_guard<std::mutex> lock(gyro_mutex_);
      gyro = last_gyro_;
    }
    const auto & T_w_i = state->T_w_i;
    const Eigen::Vector3d v_imu = T_w_i.so3().inverse() * state->vel_w_i;
    const Eigen::Vector3d w_imu = calib_.calib_gyro_bias.getCalibrated(gyro) - state->bias_gyro;
    const Eigen::Quaterniond q = T_w_i.unit_quaternion();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = to_stamp(state->t_ns);
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = imu_frame_;
    odom.pose.pose.position.x = T_w_i.translation().x();
    odom.pose.pose.position.y = T_w_i.translation().y();
    odom.pose.pose.position.z = T_w_i.translation().z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = v_imu.x();
    odom.twist.twist.linear.y = v_imu.y();
    odom.twist.twist.linear.z = v_imu.z();
    odom.twist.twist.angular.x = w_imu.x();
    odom.twist.twist.angular.y = w_imu.y();
    odom.twist.twist.angular.z = w_imu.z();
    odom_pub_->publish(odom);
  }
}

void BasaltVio::points_loop()
{
  while (running_) {
    basalt::VioVisualizationData::Ptr data;
    vis_queue_.pop(data);
    if (!data) {
      break;
    }
    publish_points(*data);
    if (depth_pub_->get_subscription_count() > 0 || depth_colored_pub_->get_subscription_count() > 0) {
      publish_depth(*data);
    }
  }
}

void BasaltVio::publish_points(const basalt::VioVisualizationData & data)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = to_stamp(data.t_ns);
  cloud.header.frame_id = world_frame_;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(data.points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const auto & p : data.points) {
    *x = static_cast<float>(p.x());
    *y = static_cast<float>(p.y());
    *z = static_cast<float>(p.z());
    ++x;
    ++y;
    ++z;
  }
  points_pub_->publish(cloud);
}

void BasaltVio::publish_depth(const basalt::VioVisualizationData & data)
{
  if (!data.opt_flow_res || !data.opt_flow_res->input_images || data.projections.empty() ||
    data.opt_flow_res->observations.empty())
  {
    return;
  }
  const auto & left = data.opt_flow_res->input_images->img_data[0].img;
  const int width = static_cast<int>(left->w);
  const int height = static_cast<int>(left->h);

  Image depth;
  depth.header.stamp = to_stamp(data.t_ns);
  depth.header.frame_id = camera_frame_;
  depth.width = width;
  depth.height = height;
  depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth.step = width * 2;
  depth.data.assign(depth.step * height, 0);

  Image colored;
  colored.header = depth.header;
  colored.width = width;
  colored.height = height;
  colored.encoding = sensor_msgs::image_encodings::RGB8;
  colored.step = width * 3;
  colored.data.resize(colored.step * height);
  for (int i = 0; i < width * height; ++i) {
    const uint8_t gray = static_cast<uint8_t>(left->ptr[i] >> 8);
    colored.data[3 * i] = gray;
    colored.data[3 * i + 1] = gray;
    colored.data[3 * i + 2] = gray;
  }

  // projections are [u, v, inverse distance, keypoint id] in the current left image
  std::unordered_map<size_t, double> inverse_distance;
  for (const auto & proj : data.projections[0]) {
    inverse_distance[static_cast<size_t>(proj[3])] = proj[2];
  }

  // like openvins: the tracked pixel position with the estimated depth, not a reprojection,
  // so the squares stay put while the pose estimate moves by fractions of a pixel
  const int half = 4;
  auto * depth_mm = reinterpret_cast<uint16_t *>(depth.data.data());
  for (const auto & [id, transform] : data.opt_flow_res->observations[0]) {
    const auto found = inverse_distance.find(id);
    if (found == inverse_distance.end() || found->second <= 0.0) {
      continue;
    }
    Eigen::Vector4d ray;
    if (!calib_.intrinsics[0].unproject(transform.translation().cast<double>(), ray)) {
      continue;
    }
    const double z = ray.z() / ray.head<3>().norm() / found->second;
    const int u = static_cast<int>(std::lround(transform.translation().x()));
    const int v = static_cast<int>(std::lround(transform.translation().y()));
    if (u < half || u >= width - half || v < half || v >= height - half || z <= 0.0) {
      continue;
    }
    depth_mm[v * width + u] = static_cast<uint16_t>(std::min(65535.0, 1000.0 * z));

    const double inverse_depth = 1.0 / z;
    const auto channel = [inverse_depth](double offset) {
      return static_cast<uint8_t>(255 - std::clamp(std::abs(offset - inverse_depth) * 255.0, 0.0, 255.0));
    };
    const uint8_t r = channel(2.0);
    const uint8_t g = channel(1.0);
    const uint8_t b = channel(0.0);
    for (int y = v - half; y <= v + half; ++y) {
      for (int x = u - half; x <= u + half; ++x) {
        uint8_t * px = &colored.data[(y * width + x) * 3];
        px[0] = r;
        px[1] = g;
        px[2] = b;
      }
    }
  }

  depth_pub_->publish(depth);
  depth_colored_pub_->publish(colored);
}

void BasaltVio::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  flow_->input_queue.push(nullptr);
  vio_->imu_data_queue.push(nullptr);
  vio_->maybe_join();
  state_queue_.push(nullptr);
  vis_queue_.push(nullptr);
  if (state_thread_.joinable()) {
    state_thread_.join();
  }
  if (points_thread_.joinable()) {
    points_thread_.join();
  }
}

}  // namespace basalt_ros
