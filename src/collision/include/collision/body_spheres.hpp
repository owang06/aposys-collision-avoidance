#ifndef BODY_SPHERES_HPP
#define BODY_SPHERES_HPP
#include <cstddef>
#include <eigen3/Eigen/Core>
#include <vector>

/*------------------*
 *  Data structures *
 *------------------*/
struct Box {
  Eigen::Vector3f centre{0, 0, 0};
  float width{1.f};
  float height{1.f};
  float depth{1.f};
};

struct Sphere {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Vec = std::vector<Sphere, Eigen::aligned_allocator<Sphere>>;

  Eigen::Vector3f offset; ///< centre in vehicle frame
  float radius;           ///< metres
};

struct SphereParams {
  float epsilon = 0.02f;      ///< safety clearance (m)
  int k_max = 16;             ///< total sphere budget (≥ number of comps)
  bool tighten = true;        ///< shrink radii to nearest sample
  bool keep_voids = false;    ///< keep spheres with no interior samples
  float sym_thresh = 0.05f;   ///< isotropy threshold for PCA fallback
  bool add_edge_mids = false; ///< also seed 12 edge mid‑points
  int res_u = 20;             ///< surface sampling resolution U
  int res_v = 20;             ///< surface sampling resolution V
  float jitter = 0.0f;        ///< outward jitter along face normal [m]
};

struct Adhoc {
  using EigVec =
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;

  static Sphere::Vec make_body_spheres(const Box &box, const SphereParams &cfg,
                                       EigVec &points);
};
#endif
