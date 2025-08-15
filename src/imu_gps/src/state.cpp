#include "imu_gps/state.hpp"
#include "imu_gps/gps_conversions.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <shared_mutex>

namespace sf {

namespace ekf_util {

template <int N>
inline void van_loan(const Eigen::Matrix<double, N, N> &F,
                     const Eigen::Matrix<double, N, N> &S, double dt,
                     Eigen::Matrix<double, N, N> &Phi,
                     Eigen::Matrix<double, N, N> &Qd) {
  constexpr int M = 2 * N;
  Eigen::Matrix<double, M, M> A;
  A.setZero();

  A.template block<N, N>(0, 0) = -F;
  A.template block<N, N>(0, N) = S; //  G Qc Gᵀ  pre‑computed outside
  A.template block<N, N>(N, N) = F.transpose();

  Eigen::Matrix<double, M, M> expA = (A * dt).exp(); // Padé / scaling‑squaring

  Phi = expA.template block<N, N>(N, N).transpose();
  Qd = Phi * expA.template block<N, N>(0, N);

  // enforce symmetry – numerical hygiene
  Qd = 0.5 * (Qd + Qd.transpose());
}

} // namespace ekf_util

void State::init(const Eigen::Quaterniond &q_WB0,
                 const Eigen::Vector3d &acc_mean,
                 const Eigen::Vector3d &gyro_mean, const Eigen::Vector3d &grav,
                 double sft0, const Eigen::Quaterniond &ori_bi,
                 const Eigen::Vector3d &pos_bi) {
  q_WB = q_WB0;
  v_WB.setZero();
  p_WB.setZero();

  g_w = grav;

  p_BI = pos_bi;
  q_BI = ori_bi;
  sft = sft0;

  b_g = gyro_mean;
  b_a = acc_mean + grav_imu();

  b_at.setOnes();
}

Eigen::Vector3d State::grav_imu() const {
  Eigen::Matrix3d R_wb = q_WB.toRotationMatrix(); // body ->world
  Eigen::Matrix3d R_bi = q_BI.toRotationMatrix(); // imu -> body

  return (R_wb * R_bi).transpose() * g_w;
}

void State::propagate(const Eigen::Vector3d &acc_m,
                      const Eigen::Vector3d &gyro_m,
                      const Eigen::Vector3d &grav, double dt) {

  /* biases and std_scale (diagonal) imu frame -> body frame*/
  Eigen::Vector3d omega_b, accel_b;
  {
    Eigen::Matrix3d i_b_rot = q_BI.toRotationMatrix();

    omega_b = i_b_rot * (gyro_m - b_g);
    accel_b = i_b_rot * (b_at.asDiagonal() * acc_m - b_a);
  }

  Eigen::Matrix3d G0, G1, G2;
  {
    G0 = helper::gamma(omega_b, dt, helper::GType::Zero);
    G1 = helper::gamma(omega_b, dt, helper::GType::One);
    G2 = helper::gamma(omega_b, dt, helper::GType::Two);
  }

  const Eigen::Matrix3d R_prev = q_WB.toRotationMatrix();

  // calculating deltas
  Eigen::Quaterniond dq(G0);
  Eigen::Vector3d dv = (grav + R_prev * G1 * accel_b) * dt;
  Eigen::Vector3d dp =
      v_WB * dt + (0.5 * grav + R_prev * G2 * accel_b) * dt * dt;

  /* rotation update */
  q_WB = (q_WB * dq).normalized();

  /* velocity update*/
  v_WB.noalias() += dv;

  /* position */
  p_WB.noalias() += dp;

  // updating the bias information
  if (noise_params.bga_process_noise_std > 0)
    b_g *= std::exp(-noise_params.bga_process_noise_rev * dt);

  if (noise_params.baa_process_noise_std > 0)
    b_a *= std::exp(-noise_params.baa_process_noise_rev * dt);
}

void State::print_state(std::ostream &os) {
  os << "\n========= EKF State =========\n";
  writers::print_vector(os, "p_WB", p_WB);
  writers::print_vector(os, "v_WB", v_WB);
  writers::print_quaternion(os, "q_WB", q_WB);

  writers::print_vector(os, "b_g", b_g);
  writers::print_vector(os, "b_a", b_a);
  writers::print_vector(os, "b_at", b_at);

  writers::print_vector(os, "p_BI", p_BI);
  writers::print_quaternion(os, "q_BI", q_BI);

  os << std::setw(10) << "sft"
     << ": " << std::fixed << std::setprecision(6) << sft << "\n";
  os << "=============================\n";
}

void State::print_state_yaml(std::ostream &os) {
  os << std::fixed << std::setprecision(6);
  os << "state:\n";
  os << "  p_WB: [" << p_WB.x() << ", " << p_WB.y() << ", " << p_WB.z()
     << "]\n";
  os << "  v_WB: [" << v_WB.x() << ", " << v_WB.y() << ", " << v_WB.z()
     << "]\n";
  os << "  q_WB:\n";
  os << "    w: " << q_WB.w() << "\n";
  os << "    x: " << q_WB.x() << "\n";
  os << "    y: " << q_WB.y() << "\n";
  os << "    z: " << q_WB.z() << "\n";
  os << "  b_g: [" << b_g.x() << ", " << b_g.y() << ", " << b_g.z() << "]\n";
  os << "  b_a: [" << b_a.x() << ", " << b_a.y() << ", " << b_a.z() << "]\n";
  os << "  b_at: [" << b_at.x() << ", " << b_at.y() << ", " << b_at.z()
     << "]\n";
  os << "  p_BI: [" << p_BI.x() << ", " << p_BI.y() << ", " << p_BI.z()
     << "]\n";
  os << "  q_BI:\n";
  os << "    w: " << q_BI.w() << "\n";
  os << "    x: " << q_BI.x() << "\n";
  os << "    y: " << q_BI.y() << "\n";
  os << "    z: " << q_BI.z() << "\n";
  os << "  sft: " << sft << "\n";
}
/*
 * ====================
 * Stationarity detector *
 * ====================
 */
StationaryDetector::StationaryDetector(double g_mag)
    : K_(noise_params.stationarity_win), acc_ok_(0), gyro_ok_(0),
      acc_thresh(0.05 * g_mag) {}

bool StationaryDetector::update(const ImuSample &s,
                                const Eigen::Vector3d &ori_grav) {

  // -------- accelerometer test ----------
  double acc_err = (s.accel + ori_grav).norm();
  if (acc_err < acc_thresh)
    acc_ok_ = std::min(acc_ok_ + 1, K_);
  else
    acc_ok_ = 0;

  // -------- gyro test -------------------
  if (s.gyro.norm() < gyro_thresh)
    gyro_ok_ = std::min(gyro_ok_ + 1, K_);
  else
    gyro_ok_ = 0;

  // -------- decision --------------------
  if (acc_ok_ == K_ && gyro_ok_ == K_) {
    acc_ok_ = gyro_ok_ = 0; // one‑shot trigger
    return true;            // declare "stationary"
  }

  return false;
}
/*
 * ====================
 * Kalman Filter stuff *
 * ====================
 */
KalmanFilter::KalmanFilter(bool est_init_pose, const WorldFrame &coord_sys)
    : est_init_pose(est_init_pose), coord_sys(coord_sys) {
  std::cout << "[Kalman Filter] Pre Covariances Initialized" << std::endl;
  init_covs();
  std::cout << "[Kalman Filter] Covariances Initialized" << std::endl;
}

void KalmanFilter::init_covs() {

  P.setZero();
  Q.setZero();

  P.block<3, 3>(POS_WB, POS_WB).setIdentity() *=
      std::pow(noise_params.init_pos_noise_std, 2);
  P.block<3, 3>(ORI_WB, ORI_WB).setIdentity() *=
      std::pow(noise_params.init_ori_noise_std, 2);
  P.block<3, 3>(VEL_WB, VEL_WB).setIdentity() *=
      std::pow(noise_params.init_vel_noise_std, 2);
  P.block<3, 3>(BAA, BAA).setIdentity() *=
      std::pow(noise_params.init_baa_noise_std, 2);
  P.block<3, 3>(BGA, BGA).setIdentity() *=
      std::pow(noise_params.init_bga_noise_std, 2);
  P.block<3, 3>(BAT, BAT).setIdentity() *=
      std::pow(noise_params.init_bat_noise_std, 2);
  P.block<1, 1>(SFT, SFT).setIdentity() *=
      std::pow(noise_params.noise_init_sft, 2);

  if (noise_params.online_calibrate_ext) {
    P.block(POS_BI, POS_BI, 3, 3).setIdentity() *=
        std::pow(noise_params.ext_init_pos_noise_std, 2);
    P.block(ORI_BI, ORI_BI, 3, 3).setIdentity() *=
        std::pow(noise_params.ext_init_ori_noise_std, 2);
  }

  P *= std::pow(noise_params.std_scale, 2);

  Q.block<3, 3>(Q_ACC, Q_ACC).setIdentity() *=
      std::pow(noise_params.baa_process_noise_std, 2);
  Q.block<3, 3>(Q_GYRO, Q_GYRO).setIdentity() *=
      std::pow(noise_params.bga_process_noise_std, 2);
  Q *= std::pow(noise_params.std_scale, 2);
}

void KalmanFilter::init(const Eigen::Vector3d &acc_avg,
                        const Eigen::Vector3d &gyro_avg, double sft,
                        TimePoint prev_ts, GpsSample &&gps_origin,
                        const Eigen::Quaterniond &ori_bi,
                        const Eigen::Vector3d &pos_bi, double g_mag) {
  grav_w = (coord_sys == WorldFrame::ENU)
               ? Eigen::Vector3d(0, 0, -g_mag)
               : Eigen::Vector3d(0, 0, +g_mag); // NED

  Eigen::Vector3d a_b = acc_avg.normalized();
  Eigen::Quaternion q_WB0 = Eigen::Quaterniond::Identity();
  if (est_init_pose)
    q_WB0 = Eigen::Quaterniond::FromTwoVectors(a_b, -grav_w).normalized();

  {
    std::unique_lock<std::shared_mutex> lock(state_mtx);
    curr.init(q_WB0, acc_avg, gyro_avg, grav_w, sft, ori_bi, pos_bi);
  }

  gps_init = std::move(gps_origin);
  last_stamp = prev_ts;

  curr.print_state_yaml();

  sd = std::make_shared<StationaryDetector>(std::abs(g_mag));
  std::unique_lock<std::shared_mutex> lock(data_mtx);
  ready = true;
}

void KalmanFilter::update_drift(double dt) {
  double noise_scale_sq = std::pow(noise_params.std_scale, 2);

  if (noise_params.baa_process_noise_std > 0.0) {
    const double qc = std::pow(noise_params.baa_process_noise_std, 2);
    // const double theta = noise_params.baa_process_noise_rev;
    Q.block(Q_BAA_DRIFT, Q_BAA_DRIFT, 3, 3).setIdentity(3, 3) *=
        noise_scale_sq * qc;
  }

  if (noise_params.bga_process_noise_std > 0.0) {
    const double qc = std::pow(noise_params.bga_process_noise_std, 2);
    // const double theta = noise_params.bga_process_noise_rev;
    Q.block(Q_BGA_DRIFT, Q_BGA_DRIFT, 3, 3).setIdentity(3, 3) *=
        noise_scale_sq * qc;
  }
}

void KalmanFilter::update_derivaties(const Eigen::Vector3d &accel_m,
                                     const Eigen::Vector3d &gyro_m) {

  // derivatives stuff
  dydx.setZero();
  dydq.setZero();

  Eigen::Matrix3d Rx_hat = curr.q_BI.toRotationMatrix();
  Eigen::Vector3d omega_c = gyro_m - curr.b_g;

  // Orientation derivatives
  dydx.block<3, 3>(ORI_WB, ORI_WB) = -helper::skew(Rx_hat * omega_c);
  dydx.block<3, 3>(ORI_WB, BGA) = -Rx_hat;
  dydx.block<3, 3>(ORI_WB, ORI_BI) = -Rx_hat * helper::skew(omega_c);
  dydq.block<3, 3>(ORI_WB, Q_GYRO) = -Rx_hat;

  // Velocity derivatives
  Eigen::Matrix3d Ry_hat = curr.q_WB.toRotationMatrix();
  Eigen::Matrix3d Ryx_hat = Ry_hat * Rx_hat;
  Eigen::Vector3d ai_hat = curr.b_at.asDiagonal() * accel_m - curr.b_a;
  dydx.block<3, 3>(VEL_WB, ORI_WB) = -Ry_hat * helper::skew(Rx_hat * ai_hat);
  dydx.block<3, 3>(VEL_WB, ORI_BI) = -Ryx_hat * helper::skew(ai_hat);
  dydx.block<3, 3>(VEL_WB, BAT) = Ryx_hat * accel_m.asDiagonal();
  dydx.block<3, 3>(VEL_WB, BAA) = -Ryx_hat;
  dydq.block<3, 3>(VEL_WB, Q_ACC) = -Ryx_hat;

  // Position Derivatives
  dydx.block<3, 3>(POS_WB, VEL_WB).setIdentity();

  // Bias derivatives
  dydx.block<3, 3>(BGA, BGA) =
      -noise_params.bga_process_noise_rev * Eigen::Matrix3d::Identity();
  dydq.block<3, 3>(BGA, Q_GYRO).setIdentity();

  dydx.block<3, 3>(BAA, BAA) =
      -noise_params.baa_process_noise_rev * Eigen::Matrix3d::Identity();
  dydq.block<3, 3>(BAA, Q_ACC).setIdentity();

  // shift time -- random walk parameters
  dydq.block<1, 1>(SFT, Q_SFT)(0, 0) = 1.0;

  if (noise_params.online_calibrate_ext) {
    dydq.block<3, 3>(ORI_BI, Q_ORI_BI) = Eigen::Matrix3d::Identity();
    dydq.block<3, 3>(POS_BI, Q_POS_BI) = Eigen::Matrix3d::Identity();
  } else {
    dydq.block<3, 3>(ORI_BI, Q_ORI_BI).setZero();
    dydq.block<3, 3>(POS_BI, Q_POS_BI).setZero();
  }
}

void KalmanFilter::propagate(const ImuSample &s) {
  if (!ready)
    return;

  double dt = std::chrono::duration<double>(s.stamp - last_stamp).count();
  if (dt > 0) // use strict > 0
  {
    update_drift(dt);

    { // --- 1. state prediction ---
      std::unique_lock<std::shared_mutex> lk(state_mtx);
      curr.propagate(s.accel, s.gyro, grav_w, dt);
    }

    update_derivaties(s.accel, s.gyro); // fills dydx, dydq

    // --- 2. covariance prediction ---
    StateCov Phi, Qd;
    ekf_util::van_loan<ERR_SIZE>(dydx, dydq * Q * dydq.transpose(), dt, Phi,
                                 Qd);
    P = Phi * P * Phi.transpose() + Qd;
    P = 0.5 * (P + P.transpose());

    // --- 3. ZUPT measurement (if stationary) ---
    if (sd && sd->update(s, curr.grav_imu()))
      zupt_update(); // uses the *predicted* P
  }

  last_stamp = s.stamp;

  // print information
}

void KalmanFilter::zupt_update() {
  Eigen::Matrix<double, 3, ERR_SIZE> H =
      Eigen::Matrix<double, 3, ERR_SIZE>::Zero();
  H.block<3, 3>(0, VEL_WB) = Eigen::Matrix3d::Identity();

  State x_nom = get_state(); // thread‑safe copy
  Eigen::Vector3d z = Eigen::Vector3d::Zero();
  Eigen::Vector3d zhat = x_nom.v_WB;
  Eigen::Vector3d r = z - zhat;

  Eigen::Matrix3d R =
      Eigen::Matrix3d::Identity() * std::pow(noise_params.zupt_var_std, 2);

  Eigen::Matrix<double, ERR_SIZE, 3> PHt = P * H.transpose();
  Eigen::Matrix3d S = H * PHt + R;
  Eigen::Matrix<double, ERR_SIZE, 3> K = PHt * S.inverse();
  Eigen::Matrix<double, ERR_SIZE, 1> dx = K * r;
  std::cout << dx << std::endl;
  // 7.  Inject corrections into nominal state
  // (a) orientation updates
  Eigen::Vector3d dtheta_WB = dx.segment<3>(ORI_WB);
  Eigen::Vector3d dtheta_BI = dx.segment<3>(ORI_BI);
  x_nom.q_WB = (x_nom.q_WB * helper::small_quat(dtheta_WB)).normalized();
  x_nom.q_BI = (x_nom.q_BI * helper::small_quat(dtheta_BI)).normalized();

  // (b) position, velocity, biases, lever‑arm, sft
  x_nom.v_WB += dx.segment<3>(VEL_WB);
  x_nom.p_WB += dx.segment<3>(POS_WB);
  x_nom.b_g += dx.segment<3>(BGA);
  x_nom.b_a += dx.segment<3>(BAA);
  x_nom.b_at += dx.segment<3>(BAT);
  x_nom.p_BI += dx.segment<3>(POS_BI);
  x_nom.sft += dx(SFT);

  curr = x_nom; // write‑back

  StateCov I_KH = StateCov::Identity() - K * H;
  P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
}

void KalmanFilter::gps_update(const GpsSample &info) {
  // convert to ENU
  Eigen::Vector3d z = geodesy::lla_to_enu(gps_init, info);

  // 2.  Nominal predicted position of IMU in world
  State x_nom = get_state(); // thread‑safe copy
  const Eigen::Matrix3d R_WB = x_nom.q_WB.toRotationMatrix();
  const Eigen::Matrix3d R_BI = x_nom.q_BI.toRotationMatrix();
  Eigen::Vector3d z_hat = x_nom.p_WB + R_WB * R_BI * x_nom.p_BI;

  // 3.  Residual
  Eigen::Vector3d r = z - z_hat;

  Eigen::Matrix<double, 3, ERR_SIZE> H =
      Eigen::Matrix<double, 3, ERR_SIZE>::Zero();
  Eigen::Matrix3d RWB_RBI = R_WB * R_BI;
  Eigen::Vector3d lever = R_BI * x_nom.p_BI;

  H.block<3, 3>(0, ORI_WB) = -R_WB * helper::skew(lever);
  H.block<3, 3>(0, POS_WB) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(0, ORI_BI) = -RWB_RBI * helper::skew(x_nom.p_BI);
  H.block<3, 3>(0, POS_BI) = RWB_RBI;

  Eigen::Matrix3d R = info.pos_cov.mat;
  // 6.  Kalman gain & covariance update (Joseph form)
  std::unique_lock<std::shared_mutex> lock(state_mtx); // P & state
  Eigen::Matrix<double, ERR_SIZE, 3> PHt = P * H.transpose();
  Eigen::Matrix3d S = H * PHt + R;
  Eigen::Matrix<double, ERR_SIZE, 3> K = PHt * S.inverse();
  Eigen::Matrix<double, ERR_SIZE, 1> dx = K * r;

  // 7.  Inject corrections into nominal state
  // (a) orientation updates
  Eigen::Vector3d dtheta_WB = dx.segment<3>(ORI_WB);
  Eigen::Vector3d dtheta_BI = dx.segment<3>(ORI_BI);
  x_nom.q_WB = (x_nom.q_WB * helper::small_quat(dtheta_WB)).normalized();
  x_nom.q_BI = (x_nom.q_BI * helper::small_quat(dtheta_BI)).normalized();

  // (b) position, velocity, biases, lever‑arm, sft
  x_nom.v_WB += dx.segment<3>(VEL_WB);
  x_nom.p_WB += dx.segment<3>(POS_WB);
  x_nom.b_g += dx.segment<3>(BGA);
  x_nom.b_a += dx.segment<3>(BAA);
  x_nom.b_at += dx.segment<3>(BAT);
  x_nom.p_BI += dx.segment<3>(POS_BI);
  x_nom.sft += dx(SFT);

  curr = x_nom; // write‑back

  // 8.  Covariance update
  StateCov I_KH = StateCov::Identity() - K * H;
  P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
}
} // namespace sf
