#ifndef COLLISION_ALGO_HPP
#define COLLISION_ALGO_HPP

#include <cfloat>
#include <cmath>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif

#include "body_spheres.hpp"

namespace CollisionAlgo {
using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;
using Idx = uint32_t;
using EigVec = Adhoc::EigVec;

// --------------------------------------- //
//  Near‑field index : uniform voxel hash  //
// --------------------------------------- //
class VoxelHash {
public:
  void build(const EigVec &P, float cell, float R2_cut_);

  inline std::pair<bool, int> any_within(const Vec3f &c, float r2) const;

private:
  float cell_{};
  std::unordered_map<uint64_t, std::vector<Idx>> tbl_;
  EigVec c_storage;
  const EigVec *cloud_{};
  inline uint64_t key(const Vec3f &p) const;
};

// -------------------------------------------------- //
//  Far‑field index : flattened kd‑tree (mini‑CAPT)
//  Ref: https://arxiv.org/pdf/2406.02807
// -------------------------------------------------- //
class CaptTree {
  struct Node {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3f mn, mx;
    uint32_t left;
    size_t num_pack;
  };

public:
  using Ptr = std::shared_ptr<CaptTree>;
  void build(const EigVec &P);

  // returns a flag and the point that causes the collision
  std::pair<bool, int> any_within(const Vec3f &c, float r) const;

private:
  bool is_leaf(const Node &n) const;

  uint32_t build_rec(size_t lo, size_t hi, int depth);

private:
  std::vector<Node> nodes_;
  std::vector<Idx> idx_;
  std::vector<float> leaf_;
  const EigVec *cloud_{};
  float cutoff_sq_{};
};

// ------------------------------- //
//  Hybrid index combining both
// ----------------------------- //
class HybridMap {
  const size_t num_pts_for_tree = 200;

public:
  void build(const EigVec &pts, float cell = 0.05f, float R_cut = 3.f);

  std::pair<bool, int> any_within(const Vec3f &c, float r) const;

private:
  VoxelHash near_;
  CaptTree::Ptr far_ = nullptr;
  float R2_cut_{};
  const EigVec *cloud_{};
  bool near_only = false;
};

// --------------------- //
//  Collision helpers
// ------------------- //
struct CollideInfo {
  int body = -1, sphere = -1, point = -1;
  Vec3d hit{0, 0, 0};
};

// Function for the collision avoidance algorithm
inline bool collision_free(const Sphere *body_begin, const Sphere *body_end,
                           const HybridMap &H, const EigVec &cloud,
                           CollideInfo *info = nullptr, int body_id = 0) {
  int sidx = 0;
  for (auto it = body_begin; it != body_end; ++it, ++sidx) {
    auto [hit, pid] = H.any_within(it->offset, it->radius);
    if (hit) {
      if (info) {
        info->body = body_id;
        info->sphere = sidx;
        info->point = pid;
        info->hit = cloud[pid].cast<double>();
      }
      return false;
    }
  }
  return true;
}

inline std::vector<CollideInfo>
check_collision(const std::vector<Sphere::Vec> &bodies, const HybridMap &H,
                const EigVec &cloud, CollideInfo *info = nullptr) {
  std::vector<CollideInfo> all;
  all.reserve(bodies.size());

#ifdef _OPENMP
#pragma omp parallel
#endif
  {
    std::vector<CollideInfo> hits;
    hits.reserve(bodies[0].size());

#ifdef _OPENMP
#pragma omp for nowait
#endif
    for (int b = 0; b < int(bodies.size()); ++b) {
      CollideInfo info;
      if (!collision_free(bodies[b].data(), bodies[b].data() + bodies[b].size(),
                          H, cloud, &info, b)) {
        hits.push_back(info);
      }
    }

    // Merge back under a lock
#ifdef _OPENMP
#pragma omp critical
#endif
    all.insert(all.end(), hits.begin(), hits.end());
  }

  return all;
}
}; // namespace CollisionAlgo
#endif
