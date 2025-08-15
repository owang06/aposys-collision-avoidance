#ifndef GPS_CONV_HPP
#define GPS_CONV_HPP
#include "param.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace geodesy {
static constexpr double a = 6378137.0;           // semi‑major (m)
static constexpr double f = 1.0 / 298.257223563; // flattening
static constexpr double b = a * (1.0 - f);
static constexpr double e2 = 1.0 - (b * b) / (a * a);

inline Eigen::Vector3d lla_to_ecef(double lat_deg, double lon_deg, double h) {
  double lat = lat_deg * M_PI / 180.0;
  double lon = lon_deg * M_PI / 180.0;
  double sin_lat = std::sin(lat), cos_lat = std::cos(lat);
  double sin_lon = std::sin(lon), cos_lon = std::cos(lon);
  double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

  double x = (N + h) * cos_lat * cos_lon;
  double y = (N + h) * cos_lat * sin_lon;
  double z = (N * (1 - e2) + h) * sin_lat;
  return {x, y, z};
}

inline Eigen::Matrix3d ecef_to_enu_rotation(double lat_deg, double lon_deg) {
  double lat = lat_deg * M_PI / 180.0;
  double lon = lon_deg * M_PI / 180.0;
  double sin_lat = std::sin(lat), cos_lat = std::cos(lat);
  double sin_lon = std::sin(lon), cos_lon = std::cos(lon);

  Eigen::Matrix3d R;
  R << -sin_lon, cos_lon, 0, -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
      cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
  return R;
}

inline Eigen::Vector3d lla_to_enu(const sf::GpsSample &ref,
                                  const sf::GpsSample &pt) {
  Eigen::Vector3d ecef_ref = lla_to_ecef(ref.lat, ref.lon, ref.alt);
  Eigen::Vector3d ecef_pt = lla_to_ecef(pt.lat, pt.lon, pt.alt);
  Eigen::Vector3d delta = ecef_pt - ecef_ref;
  Eigen::Matrix3d R = ecef_to_enu_rotation(ref.lat, ref.lon);

  return R * delta; // E,N,U  (world frame)
}
} // namespace geodesy
#endif
