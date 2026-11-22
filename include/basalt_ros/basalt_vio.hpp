#pragma once

#include <basalt/calibration/calibration.hpp>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/utils/vio_config.h>
#include <basalt/vi_estimator/vio_estimator.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace basalt_ros
{

class BasaltVio : public rclcpp::Node
{
public:
  explicit BasaltVio(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BasaltVio() override;

private:
  using Image = sensor_msgs::msg::Image;
  using StereoSync = message_filters::TimeSynchronizer<Image, Image>;

  void load_calibration(const std::string & path);
  void imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void stereo_cb(const Image::ConstSharedPtr & left, const Image::ConstSharedPtr & right);
  bool to_basalt(const Image & msg, basalt::ImageData & out) const;
  bool try_initialize(int64_t frame_ns);
  void state_loop();
  void points_loop();
  void publish_points(const basalt::VioVisualizationData & data);
  void publish_depth(const basalt::VioVisualizationData & data);
  void stop();

  std::string world_frame_;
  std::string imu_frame_;
  std::string camera_frame_;

  basalt::Calibration<double> calib_;
  basalt::VioConfig config_;
  basalt::OpticalFlowBase::Ptr flow_;
  basalt::VioEstimatorBase::Ptr vio_;
  tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> state_queue_;
  tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> vis_queue_;

  double init_window_;
  double init_max_accel_std_;
  double init_max_gyro_std_;
  bool initialized_ = false;
  std::deque<basalt::ImuData<double>::Ptr> imu_window_;

  std::mutex gyro_mutex_;
  Eigen::Vector3d last_gyro_ = Eigen::Vector3d::Zero();
  int64_t last_imu_ns_ = 0;
  size_t imu_dropped_ = 0;
  int64_t last_image_ns_ = 0;
  std::atomic<bool> running_{true};
  std::thread state_thread_;
  std::thread points_thread_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  message_filters::Subscriber<Image> left_sub_;
  message_filters::Subscriber<Image> right_sub_;
  std::shared_ptr<StereoSync> stereo_sync_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<Image>::SharedPtr depth_colored_pub_;
};

}  // namespace basalt_ros
