#ifndef STATE_KALMAN_HPP
#define STATE_KALMAN_HPP
#include "helper.hpp"
#include "param.hpp"
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <mutex>
#include <shared_mutex>

namespace sf {

// Structure of the EKF state variable
constexpr int POS_WB = 0; // body to world
constexpr int VEL_WB = 3;
constexpr int ORI_WB = 6;
constexpr int BGA = 9;
constexpr int BAA = 12;
constexpr int BAT = 15;
constexpr int SFT = 18;
constexpr int POS_BI = 19; // imu to body
constexpr int ORI_BI = 22;
static constexpr int ERR_SIZE = 25;

// Structure of process noise (prediction).
constexpr int Q_ACC = 0;
constexpr int Q_GYRO = 3;
constexpr int Q_BGA_DRIFT = 6;
constexpr int Q_BAA_DRIFT = 9;
constexpr int Q_SFT = 12;
constexpr int Q_POS_BI = 13;
constexpr int Q_ORI_BI = 16;
constexpr int Q_DIM = 19;

/* ---------------------------------------------------------
 *  World frame convention
 *  ENU :  +Z  = Up     (gravity = [ 0  0 -g])
 *  NED :  +Z  = Down   (gravity = [ 0  0 +g])
 * --------------------------------------------------------*/
enum class WorldFrame { ENU, NED };

struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // (Body to world orientation) in IMU frame
  Eigen::Vector3d p_WB{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_WB{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_WB{1, 0, 0, 0}; // body -> world

  Eigen::Vector3d b_g{Eigen::Vector3d::Zero()};
  Eigen::Vector3d b_a{Eigen::Vector3d::Zero()};
  Eigen::Vector3d b_at{Eigen::Vector3d::Zero()};

  // Lever-arm imu to body orientation
  Eigen::Quaterniond q_BI{1, 0, 0, 0}; // imu->body
  Eigen::Vector3d p_BI{Eigen::Vector3d::Zero()};
  double sft{0.0};

  void init(const Eigen::Quaterniond &q_WB0, const Eigen::Vector3d &acc_mean,
            const Eigen::Vector3d &gyro_mean, const Eigen::Vector3d &grav,
            double sft0,
            const Eigen::Quaterniond &ori_bi = Eigen::Quaterniond::Identity(),
            const Eigen::Vector3d &pos_bi = Eigen::Vector3d::Zero());

  void propagate(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
                 const Eigen::Vector3d &grav, double dt);

  void print_state(std::ostream &os = std::cout);

  void print_state_yaml(std::ostream &os = std::cout);

  Eigen::Vector3d grav_imu() const;

private:
  Eigen::Vector3d g_w;
};
/*
 * ====================
 * Stationarity detector *
 * ====================
 */
class StationaryDetector {
public:
  using Ptr = std::shared_ptr<StationaryDetector>;
  explicit StationaryDetector(double g_mag);

  void init(const Eigen::Vector3d &grav);

  bool update(const ImuSample &s, const Eigen::Vector3d &ori_grav);

private:
  size_t K_;
  size_t acc_ok_, gyro_ok_;
  double acc_thresh;
  double gyro_thresh = 0.01;
};

/*
 * ====================
 * Kalman Filter stuff *
 * ====================
 */
struct KalmanFilter {
  mutable std::shared_mutex state_mtx;
  mutable std::shared_mutex data_mtx;

public:
  using Ptr = std::shared_ptr<KalmanFilter>;
  using StateCov = Eigen::Matrix<double, ERR_SIZE, ERR_SIZE>;
  using ProcessCov = Eigen::Matrix<double, Q_DIM, Q_DIM>;
  using DydqMat = Eigen::Matrix<double, ERR_SIZE, Q_DIM>;

  explicit KalmanFilter(bool est_init_pose = true,
                        const WorldFrame &coord_sys = WorldFrame::ENU);

  void init_covs();

  void init(const Eigen::Vector3d &acc_m, const Eigen::Vector3d &gyro_m,
            double sft, TimePoint prev_ts, GpsSample &&gps_origin,
            const Eigen::Quaterniond &ori_bi = Eigen::Quaterniond::Identity(),
            const Eigen::Vector3d &pos_bi = Eigen::Vector3d::Zero(),
            double g_mag = 9.80665);

  void propagate(const ImuSample &s);

  void gps_update(const GpsSample &info);

  double sft() const { return curr.sft; } // estimates lag time

  void update_drift(double dt);

  void update_derivaties(const Eigen::Vector3d &accel_m,
                         const Eigen::Vector3d &gyro_m);

  void zupt_update();

  State get_state() const {
    std::shared_lock<std::shared_mutex> lock(state_mtx);
    return curr;
  }

  bool is_ready() const {
    std::shared_lock<std::shared_mutex> lock(data_mtx);
    return ready;
  }

private:
  bool est_init_pose;
  WorldFrame coord_sys;

  State curr;
  Eigen::Vector3d grav_w;
  GpsSample gps_init;
  TimePoint last_stamp;
  bool ready{false};

  StateCov P, dydx;
  ProcessCov Q;
  DydqMat dydq;

  StationaryDetector::Ptr sd;
  size_t interval_stuff{0};
};
}; // namespace sf
#endif
