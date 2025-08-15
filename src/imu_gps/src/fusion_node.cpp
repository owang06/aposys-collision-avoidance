#include "imu_gps/fusion_node.hpp"
#include <chrono>
#include <thread>

namespace sf {

namespace detail {
static inline Eigen::Matrix3d array_to_eig(const std::array<double, 9> &src) {
  Eigen::Matrix3d M;
  for (int r = 0, k = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c, ++k)
      M(r, c) = src[k];
  return M;
}
} // namespace detail

SensorFusionNode::SensorFusionNode(const rclcpp::NodeOptions &opts)
    : Node("sensor_fusion_node", opts),
      filter(std::make_shared<KalmanFilter>()) {
  using std::placeholders::_1;
  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 50, std::bind(&SensorFusionNode::imu_cb, this, _1));
  gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix", 10, std::bind(&SensorFusionNode::gps_cb, this, _1));
  pose_pub =
      create_publisher<geometry_msgs::msg::PoseStamped>("/aposys/pose", 10);

  twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/aposys/twist", 10);

  imu_queue.set_capacity(1000);
  gps_queue.set_capacity(50);

  filter_timer =
      this->create_wall_timer(std::chrono::milliseconds(2),
                              std::bind(&SensorFusionNode::ekf_spin, this));

  // Optional offline playback
  std::string offline_dir =
      this->declare_parameter<std::string>("offline_dir", "");
  // std::string offline_dir = "/home/or/Desktop/CollisionAvoidance/";

  if (!offline_dir.empty()) {
    RCLCPP_INFO(get_logger(), "Offline mode: %s", offline_dir.c_str());
    player = std::make_unique<FilePlayer>(offline_dir, imu_queue, gps_queue);
    player->read_initial_samples(N_IMU, N_GPS);
  }

  pose_log.open("pose_output.txt");

  ori_bi = Eigen::Quaterniond::Identity();
  pos_bi = Eigen::Vector3d::Zero();

  init_timer =
      create_wall_timer(std::chrono::milliseconds(10), // check at 100 Hz
                        std::bind(&SensorFusionNode::maybe_init, this));
}

void SensorFusionNode::maybe_init() {
  constexpr size_t N_IMU = 100, N_GPS = 5;

  if (filter->is_ready()) {
    init_timer->cancel(); // safety: should run only once
    return;
  }

  if (try_init(ori_bi, pos_bi, N_IMU, N_GPS)) {
    RCLCPP_INFO(get_logger(), "EKF initialised (online)");
    init_timer->cancel(); // stop the timer after success
    if (player) {
      imu_queue.clear();
      gps_queue.clear();
      player->start(); // start playback after initialisation
    }
  }
}

void SensorFusionNode::warmup_and_init(bool use_file) {
  Eigen::Quaterniond ori_bi = Eigen::Quaterniond::Identity();
  Eigen::Vector3d pos_bi = Eigen::Vector3d::Zero();
  size_t N_IMU = 100, N_GPS = 5;

  RCLCPP_INFO(get_logger(), "Initializing State Parameters");
  if (use_file)
    player->read_initial_samples(N_IMU, N_GPS);

  while (!try_init(ori_bi, pos_bi, N_IMU, N_GPS))
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

SensorFusionNode::~SensorFusionNode() {
  if (player)
    player->join();

  if (pose_log.is_open())
    pose_log.close();
}

void SensorFusionNode::imu_cb(const sensor_msgs::msg::Imu::SharedPtr m) {
  ImuSample s;
  s.stamp = tp_from(rclcpp::Time(m->header.stamp));

  s.gyro << m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z;
  s.accel << m->linear_acceleration.x, m->linear_acceleration.y,
      m->linear_acceleration.z;

  // ✚ extract covariances
  s.cov_q.mat = detail::array_to_eig(m->orientation_covariance);
  s.cov_gyro.mat = detail::array_to_eig(m->angular_velocity_covariance);
  s.cov_accel.mat = detail::array_to_eig(m->linear_acceleration_covariance);

  imu_queue.try_push(std::move(s));
}

void SensorFusionNode::gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr m) {
  GpsSample g;
  g.stamp = tp_from(rclcpp::Time(m->header.stamp));
  g.lat = m->latitude;
  g.lon = m->longitude;
  g.alt = m->altitude;

  g.pos_cov.mat = detail::array_to_eig(m->position_covariance);
  gps_queue.try_push(std::move(g));
}

double SensorFusionNode::est_init_shift(const std::vector<ImuSample> &imu,
                                        const std::vector<GpsSample> &gps) {
  std::vector<double> diffs;
  diffs.reserve(gps.size());

  // Pointers for linear merge (imu & gps are time‑sorted)
  size_t i = 0;
  for (const auto &g : gps) {
    // advance imu pointer until imu[i].stamp is just >= g.stamp
    while (i + 1 < imu.size() && imu[i + 1].stamp <= g.stamp)
      ++i;

    // choose the closer of imu[i] and imu[i+1] (if exists)
    TimePoint imu_time = imu[i].stamp;
    if (i + 1 < imu.size()) {
      auto dt_prev =
          std::chrono::duration<double>(g.stamp - imu[i].stamp).count();
      auto dt_next =
          std::chrono::duration<double>(imu[i + 1].stamp - g.stamp).count();
      if (std::abs(dt_next) < std::abs(dt_prev))
        imu_time = imu[i + 1].stamp;
    }

    double diff = std::chrono::duration<double>(g.stamp - imu_time).count();
    diffs.push_back(diff); // sign convention: GPS − IMU
  }

  // median
  std::nth_element(diffs.begin(), diffs.begin() + diffs.size() / 2,
                   diffs.end());
  return diffs[diffs.size() / 2];
}

bool SensorFusionNode::try_init(const Eigen::Quaterniond &ori_bi,
                                const Eigen::Vector3d &pos_bi, size_t N_IMU,
                                size_t N_GPS) {
  ImuSample s;
  GpsSample g;

  if (init_imu.size() < N_IMU && imu_queue.try_pop(s))
    init_imu.push_back(s);

  if (init_gps.size() < N_GPS && gps_queue.try_pop(g))
    init_gps.push_back(g);

  if (init_imu.size() < N_IMU || init_gps.size() < N_GPS)
    return false;

  // means
  Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero(),
                  acc_mean = Eigen::Vector3d::Zero();
  for (auto &i : init_imu) {
    gyro_mean += i.gyro;
    acc_mean += i.accel;
  }

  gyro_mean /= N_IMU;
  acc_mean /= N_IMU;

  GpsSample origin;
  for (auto &o : init_gps) {
    origin.lat += o.lat;
    origin.lon += o.lon;
    origin.alt += o.alt;
  }

  origin.lat /= N_GPS;
  origin.lon /= N_GPS;
  origin.alt /= N_GPS;

  double sft0 = est_init_shift(init_imu, init_gps);
  filter->init(acc_mean, gyro_mean, sft0, init_imu.back().stamp,
               std::move(origin), ori_bi, pos_bi);

  RCLCPP_INFO(get_logger(), "EKF initialised with online extrinsics");
  init_imu.clear();
  init_gps.clear();

  return true;
}

void SensorFusionNode::ekf_spin() {
  if (!filter->is_ready())
    return;

  if (!imu_hold) {
    ImuSample s;
    if (imu_queue.try_pop(s))
      imu_hold = s;
  }

  if (!gps_hold) {
    GpsSample g;
    if (gps_queue.try_pop(g))
      gps_hold = g;
  }

  if (!imu_hold && !gps_hold)
    return;

  bool use_imu = true;
  if (gps_hold) {
    if (imu_hold) {
      // sft is estimated online
      use_imu = imu_hold->stamp <=
                gps_hold->stamp -
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(filter->sft()));
    } else
      use_imu = false;
  }

  if (use_imu) {
    filter->propagate(*imu_hold);
    imu_hold.reset();
  } else {
    filter->gps_update(*gps_hold);
    gps_hold.reset();
  }

  // publish most recent pose update
  publish_pose();
}

void SensorFusionNode::publish_pose() {
  const auto st = filter->get_state(); // Copy current filter state

  // Transform IMU pose into world frame
  const Eigen::Matrix3d R_WB = st.q_WB.toRotationMatrix();
  const Eigen::Vector3d p_WI = st.p_WB + R_WB * st.p_BI;
  const Eigen::Quaterniond q_WI = st.q_WB * st.q_BI;

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map"; // Convention: world-fixed frame

  msg.pose.position.x = p_WI.x();
  msg.pose.position.y = p_WI.y();
  msg.pose.position.z = p_WI.z();

  msg.pose.orientation.w = q_WI.w();
  msg.pose.orientation.x = q_WI.x();
  msg.pose.orientation.y = q_WI.y();
  msg.pose.orientation.z = q_WI.z();

  pose_pub->publish(msg);

  if (pose_log.is_open()) {
    pose_log << std::fixed << std::setprecision(6) << "["
             << this->now().seconds() << "] Position: (" << p_WI.x() << ", "
             << p_WI.y() << ", " << p_WI.z() << ") Orientation (quat): ("
             << q_WI.w() << ", " << q_WI.x() << ", " << q_WI.y() << ", "
             << q_WI.z() << ")\n";
  }

  // linear velocity of IMU frame in world
  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg.header;

  twist.twist.linear.x = st.v_WB.x();
  twist.twist.linear.y = st.v_WB.y();
  twist.twist.linear.z = st.v_WB.z();

  twist_pub->publish(twist);
}
} // namespace sf
