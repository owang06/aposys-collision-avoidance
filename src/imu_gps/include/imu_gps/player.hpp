#ifndef DATA_PLAYER_HPP
#define DATA_PLAYER_HPP
#include "param.hpp"
#include <cstddef>
#include <fstream>
#include <string>
#include <tbb/concurrent_queue.h>
#include <yaml-cpp/yaml.h>

// -----------------------------------------------------------------------------
//  FilePlayer – reads YAML docs (--- delimiter) and pushes into queues at
//  real‑time pace
// -----------------------------------------------------------------------------
namespace sf {

static bool next_yaml_doc(std::istream &is, std::string &doc_out) {
  doc_out.clear();
  std::string line;

  while (std::getline(is, line)) {
    // New document delimiter
    if (line.rfind("---", 0) == 0 && !doc_out.empty())
      return true;

    doc_out += line + '\n';
  }
  // EOF: return whatever we collected (if any)
  return !doc_out.empty();
}

class FilePlayer {
public:
  FilePlayer(const std::filesystem::path &dir,
             tbb::concurrent_bounded_queue<ImuSample> &imu_q,
             tbb::concurrent_bounded_queue<GpsSample> &gps_q,
             double speed = 1.0)
      : dir_(dir), imu_q_(imu_q), gps_q_(gps_q), speed_(speed) {}

  void start() {
    worker_ = std::thread([this] { this->run(); });
  }

  void join() {
    if (worker_.joinable())
      worker_.join();
  }

  bool read_initial_samples(size_t N_imu, size_t N_gps) {
    std::ifstream imu_in(dir_ / "imu_data.txt");
    std::ifstream gps_in(dir_ / "gps_data.txt");

    if (!imu_in || !gps_in) {
      std::cerr << "FilePlayer: imu.yaml or gps.yaml missing in " << dir_
                << std::endl;
      return false;
    }

    std::string doc;

    // first N_imu
    for (size_t i = 0; i < N_imu && next_yaml_doc(imu_in, doc); ++i)
      imu_q_.push(parse_imu(YAML::Load(doc)));

    // first N_gps
    for (size_t i = 0; i < N_gps && next_yaml_doc(gps_in, doc); ++i)
      gps_q_.push(parse_gps(YAML::Load(doc)));

    return size_t(imu_q_.size()) == N_imu && size_t(gps_q_.size()) == N_gps;
  }

private:
  void run() {
    std::ifstream imu_in(dir_ / "imu_data.txt");
    std::ifstream gps_in(dir_ / "gps_data.txt");

    if (!imu_in || !gps_in) {
      std::cerr << "FilePlayer: log files missing in " << dir_ << "\n";
      return;
    }

    std::string imu_buf, gps_buf;
    bool imu_ok = next_yaml_doc(imu_in, imu_buf);
    bool gps_ok = next_yaml_doc(gps_in, gps_buf);

    TimePoint base_stamp =
        imu_ok ? to_timepoint(YAML::Load(imu_buf)["header"]["stamp"])
               : TimePoint::clock::now();
    auto ref_time = std::chrono::steady_clock::now();

    while (imu_ok || gps_ok) {
      TimePoint imu_t =
          imu_ok ? to_timepoint(YAML::Load(imu_buf)["header"]["stamp"])
                 : TimePoint::max();
      TimePoint gps_t =
          gps_ok ? to_timepoint(YAML::Load(gps_buf)["header"]["stamp"])
                 : TimePoint::max();

      if (imu_t <= gps_t && imu_ok) {
        ImuSample s = parse_imu(YAML::Load(imu_buf));
        wait_for(s.stamp, base_stamp, ref_time);
        imu_q_.push(std::move(s));

        imu_ok = next_yaml_doc(imu_in, imu_buf); // advance stream
      } else if (gps_ok) {
        GpsSample s = parse_gps(YAML::Load(gps_buf));
        wait_for(s.stamp, base_stamp, ref_time);
        gps_q_.push(std::move(s));

        gps_ok = next_yaml_doc(gps_in, gps_buf); // advance stream
      }
    }
  }

  void wait_for(TimePoint msg_stamp, TimePoint base, TimePoint ref) {
    auto elapsed = (msg_stamp - base) / speed_;
    auto target = ref + elapsed;
    std::this_thread::sleep_until(target);
  }

  static TimePoint to_timepoint(const YAML::Node &stamp) {
    auto sec = stamp["sec"].as<int64_t>();
    auto nsec = stamp["nanosec"].as<int32_t>();
    return TimePoint(std::chrono::nanoseconds{sec * 1000000000LL + nsec});
  }

  static ImuSample parse_imu(const YAML::Node &n) {
    ImuSample s;
    s.stamp = to_timepoint(n["header"]["stamp"]);

    const auto &ang = n["angular_velocity"];
    s.gyro << ang["x"].as<double>(), ang["y"].as<double>(),
        ang["z"].as<double>();
    const auto &acc = n["linear_acceleration"];
    s.accel << acc["x"].as<double>(), acc["y"].as<double>(),
        acc["z"].as<double>();
    // covariances omitted for brevity – parse similarly
    return s;
  }

  static GpsSample parse_gps(const YAML::Node &n) {
    GpsSample s;
    s.stamp = to_timepoint(n["header"]["stamp"]);
    s.lat = n["latitude"].as<double>();
    s.lon = n["longitude"].as<double>();
    s.alt = n["altitude"].as<double>();
    // cov parsing alike
    return s;
  }

  std::filesystem::path dir_;
  tbb::concurrent_bounded_queue<ImuSample> &imu_q_;
  tbb::concurrent_bounded_queue<GpsSample> &gps_q_;
  double speed_;
  std::thread worker_;
};
} // namespace sf
#endif
