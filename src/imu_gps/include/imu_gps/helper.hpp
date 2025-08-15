#ifndef IMU_GPS_HELPER
#define IMU_GPS_HELPER
#include "param.hpp"
#include <cmath>

namespace helper {

enum class GType { Zero, One, Two };

static inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

static inline Eigen::Matrix3d gamma(const Eigen::Vector3d &omega,
                                    const double &dt, GType typ) {
  const Eigen::Vector3d phi = omega * dt;
  const double ang = phi.norm();
  const Eigen::Matrix3d Phi = skew(phi);
  const Eigen::Matrix3d Phi2 = Phi * Phi;

  double A, B, C, D; // set only what we need per typ

  if (ang < 1e-8) {
    // sin(phi) ≈ (phi) − (phi)³/6 ,   cos(phi) ≈ 1 − (phi)²/2
    A = 1.0 - ang * ang / 6.0;
    B = 0.5 - ang * ang / 24.0;
    C = 1.0 / 6.0 - ang * ang / 120.0;
    D = 1.0 / 24.0 - ang * ang / 720.0;
  } else {

    const double s = std::sin(ang);
    const double c = std::cos(ang);
    const double ang2 = ang * ang;

    A = s / ang;
    B = (1.0 - c) / ang2;
    C = (ang - s) / (ang2 * ang);
    D = (ang2 + 2.0 * c - 2.0) / (2.0 * ang2 * ang2);
  }

  // building the requsted integral
  switch (typ) {
  case GType::Zero: // Γ0
    return Eigen::Matrix3d::Identity() + A * Phi + B * Phi2;

  case GType::One: // Γ1
    return Eigen::Matrix3d::Identity() + B * Phi + C * Phi2;

  case GType::Two: // Γ2
    return 0.5 * Eigen::Matrix3d::Identity() + C * Phi + D * Phi2;
  }

  return Eigen::Matrix3d::Identity();
}

static inline Eigen::Quaterniond small_quat(const Eigen::Vector3d &dtheta) {
  double angle = dtheta.norm();
  if (angle < 1e-12)
    return Eigen::Quaterniond::Identity();
  Eigen::Vector3d axis = dtheta / angle;
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}
}; // namespace helper
#endif
