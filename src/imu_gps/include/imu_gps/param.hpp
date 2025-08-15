#ifndef IMU_GPS_HPP
#define IMU_GPS_HPP
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

namespace writers {
inline void print_vector(std::ostream &os, const std::string &label,
                         const Eigen::Vector3d &v, int width = 12) {
  os << std::setw(10) << label << ": " << std::fixed << std::setprecision(6)
     << std::setw(width) << v.x() << ", " << std::setw(width) << v.y() << ", "
     << std::setw(width) << v.z() << "\n";
}

inline void print_quaternion(std::ostream &os, const std::string &label,
                             const Eigen::Quaterniond &q, int width = 12) {
  os << std::setw(10) << label << ": " << std::fixed << std::setprecision(6)
     << "w: " << std::setw(width) << q.w() << ", "
     << "x: " << std::setw(width) << q.x() << ", "
     << "y: " << std::setw(width) << q.y() << ", "
     << "z: " << std::setw(width) << q.z() << "\n";
}
}; // namespace writers

namespace sf { // Sensor Fusion
using TimePoint = std::chrono::time_point<std::chrono::steady_clock,
                                          std::chrono::nanoseconds>;

// -----------------------------------------------------------------------------
struct Cov3d {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d mat{Eigen::Matrix3d::Zero()};
};

struct ImuSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TimePoint stamp{};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
  Cov3d cov_q, cov_gyro, cov_accel;
};

struct GpsSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TimePoint stamp{};
  double lat{0.0}, lon{0.0}, alt{0.0};
  Cov3d pos_cov;
};

struct KFNoise {
  double init_pos_noise_std = 1e-3;
  double init_ori_noise_std = 0.0316227766;
  double init_vel_noise_std = 0.1;

  // accelerometer and gyro process noise - From manufacturer
  double acc_process_noise_std = 0.003;
  double gyro_process_noise_std = 0.0017;

  // Biases - From manufacturer?
  double init_baa_noise_std = 1e-3;
  double init_bga_noise_std = 1e-3;
  double init_bat_noise_std = 1e-3;

  double noise_init_sft = 1e-3;

  // Random process noise and random walk init
  double baa_process_noise_std = 1e-4;
  double bga_process_noise_std = 1e-4;

  // extrinsics
  double ext_init_pos_noise_std = init_pos_noise_std / 10;
  double ext_init_ori_noise_std = init_ori_noise_std / 10;

  // only for measurement, state, and process covariance with std

  // mean reverse parameters not affected by scale
  double baa_process_noise_rev = 0.1;
  double bga_process_noise_rev = 0.1;
  bool online_calibrate_ext = true;

  // for stationarity detector
  size_t stationarity_win = 50;
  double zupt_var_std = 0.02;

  // gps measurement error
  double gps_err = 1.5; // m^2
  // scale
  double std_scale = 1;
};

} // namespace sf

extern sf::KFNoise noise_params;
#endif
