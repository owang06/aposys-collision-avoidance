#ifndef SENSOR_FUSION_NODE
#define SENSOR_FUSION_NODE
#include "param.hpp"
#include "player.hpp"
#include "state.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace sf {
class SensorFusionNode : public rclcpp::Node {
public:
  SensorFusionNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());

  ~SensorFusionNode() override;

private:
  static TimePoint tp_from(const rclcpp::Time &t) {
    return TimePoint(std::chrono::nanoseconds(t.nanoseconds()));
  }

  bool try_init(const Eigen::Quaterniond &ori_bi, const Eigen::Vector3d &pos_bi,
                size_t N_IMU, size_t N_GPS);

  void warmup_and_init(bool use_file);

  // ---------------- ROS Callbacks (push‑only) ------------------
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);

  void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  // ---------------- EKF spin (runs every 2 ms) ------------------
  void ekf_spin();

  void publish_pose();

  double est_init_shift(const std::vector<ImuSample> &imu,
                        const std::vector<GpsSample> &gps);

  void maybe_init();
  // Members -----------------------------------------------------
  KalmanFilter::Ptr filter;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
  rclcpp::TimerBase::SharedPtr filter_timer, init_timer;

  tbb::concurrent_bounded_queue<ImuSample> imu_queue;
  tbb::concurrent_bounded_queue<GpsSample> gps_queue;
  std::optional<ImuSample> imu_hold;
  std::optional<GpsSample> gps_hold;

  std::vector<ImuSample> init_imu;
  std::vector<GpsSample> init_gps;

  std::unique_ptr<FilePlayer> player;
  std::ofstream pose_log;
  size_t int_step_count{0};
  size_t N_IMU{100}, N_GPS{5};

  Eigen::Quaterniond ori_bi;
  Eigen::Vector3d pos_bi;
};
}; // namespace sf
#endif
