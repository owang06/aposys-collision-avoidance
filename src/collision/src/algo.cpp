#include "collision/algo.hpp"
#include <algorithm>
#include <cfloat>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numeric>

namespace CollisionAlgo {
namespace detail {
inline uint32_t part1(uint32_t x) {
  x = (x | (x << 16)) & 0x030000FFu;
  x = (x | (x << 8)) & 0x0300F00Fu;
  x = (x | (x << 4)) & 0x030C30C3u;
  x = (x | (x << 2)) & 0x09249249u;
  return x;
}

inline uint64_t morton3D(int x, int y, int z) {
  return (uint64_t)part1(x) | ((uint64_t)part1(y) << 1) |
         ((uint64_t)part1(z) << 2);
}

inline float dist2_AABB(const Vec3f &p, const Vec3f &mn, const Vec3f &mx) {
  float d = 0.f;
  for (int i = 0; i < 3; ++i) {
    float v = (p[i] < mn[i] ? mn[i] - p[i] : (p[i] > mx[i] ? p[i] - mx[i] : 0));
    d += v * v;
  }

  return d;
}
} // namespace detail

// --------------------------------------- //
//  Near‑field index : uniform voxel hash  //
// --------------------------------------- //
uint64_t VoxelHash::key(const Vec3f &p) const {
  return detail::morton3D(int(std::floor(p.x() / cell_)),
                          int(std::floor(p.y() / cell_)),
                          int(std::floor(p.z() / cell_)));
}

void VoxelHash::build(const EigVec &P, float cell, float R2_cut_) {
  cell_ = cell;
  tbl_.clear();
  tbl_.reserve(size_t(P.size() * 1.1));

  c_storage = P;
  cloud_ = &c_storage;
  float cut_sq = R2_cut_ * R2_cut_;
  for (Idx i = 0; i < P.size(); ++i) {
    if (P[i].squaredNorm() < cut_sq) {
      auto k = key(P[i]);
      tbl_[k].push_back(i);
    }
  }
}

std::pair<bool, int> VoxelHash::any_within(const Vec3f &c, float r2) const {
  int ix = int(std::floor(c.x() / cell_));
  int iy = int(std::floor(c.y() / cell_));
  int iz = int(std::floor(c.z() / cell_));

  // std::cout << "In any within here" << std::endl;
  // if (cloud_)
  //   std::cout << "Cloud exists" << std::endl;
  // check all neigbours for potential match
  for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
      for (int dz = -1; dz <= 1; ++dz) {
        auto it = tbl_.find(detail::morton3D(ix + dx, iy + dy, iz + dz));
        if (it == tbl_.end())
          continue;

        for (auto id : it->second) {
          if (((*cloud_)[id] - c).squaredNorm() < r2)
            return {true, int(id)};
        }
      }

  return {false, -1};
}

// -------------------------------------------------- //
//  Far‑field index : flattened kd‑tree (mini‑CAPT)
// -------------------------------------------------- //
bool CaptTree::is_leaf(const Node &n) const { return n.left & 0x80000000u; }

uint32_t CaptTree::build_rec(size_t lo, size_t hi, int depth) {
  uint32_t cur = nodes_.size();
  nodes_.push_back({});

  Vec3f mn = (*cloud_)[idx_[lo]], mx = mn;
  for (size_t i = lo + 1; i < hi; ++i) {
    mn = mn.cwiseMin((*cloud_)[idx_[i]]);
    mx = mx.cwiseMax((*cloud_)[idx_[i]]);
  }

  if (hi - lo <= 8) {
    Node &n = nodes_[cur];
    n.mn = mn;
    n.mx = mx;
    // tells us the start idx for [x,y,z] of leaf [0-7]
    n.left = 0x80000000u | uint32_t(leaf_.size() / 24);
    n.num_pack = hi - lo;

    for (size_t i = lo; i < hi; ++i) {
      Vec3f p = (*cloud_)[idx_[i]];
      leaf_.push_back(p.x());
    }

    for (size_t i = lo; i < hi; ++i) {
      Vec3f p = (*cloud_)[idx_[i]];
      leaf_.push_back(p.y());
    }

    for (size_t i = lo; i < hi; ++i) {
      Vec3f p = (*cloud_)[idx_[i]];
      leaf_.push_back(p.z());
    }

    return cur;
  }

  Vec3f ext = mx - mn;
  int axis;
  ext.maxCoeff(&axis); // splitting across range
  size_t mid = (lo + hi) >> 1;
  std::nth_element(
      idx_.begin() + lo, idx_.begin() + mid, idx_.begin() + hi,
      [&](Idx a, Idx b) { return (*cloud_)[a][axis] < (*cloud_)[b][axis]; });

  Node &n = nodes_[cur];
  n.mn = mn;
  n.mx = mx;
  n.left = build_rec(lo, mid, depth + 1);
  build_rec(mid, hi, depth + 1);
  return cur;
}

void CaptTree::build(const EigVec &P) {
  cloud_ = &P;
  idx_.resize(P.size());
  std::iota(idx_.begin(), idx_.end(), 0);
  nodes_.clear();

  build_rec(0, P.size(), 0);
}

std::pair<bool, int> CaptTree::any_within(const Vec3f &c, float r) const {
  float r2 = r * r;
  uint32_t stack[64];
  int sp = 0;
  stack[sp++] = 0;

  while (sp) {
    const Node &n = nodes_[stack[--sp]];
    if (detail::dist2_AABB(c, n.mn, n.mx) > r2)
      continue;
    if (is_leaf(n)) {
      const size_t base = n.left & 0x7FFFFFFFu; // leaf flag bit31
      const size_t cnt = n.num_pack;
#ifdef __AVX2__
      __m256 cx = _mm256_set1_ps(c.x()), cy = _mm256_set1_ps(c.y()),
             cz = _mm256_set1_ps(c.z()), rr = _mm256_set1_ps(r2);

      // we only loop once mostly but written generically
      for (size_t i = 0; i < cnt; i += 8) {
        __m256 px, py, pz;
        memcpy(&px, &leaf_[3 * (base + i) + 0], sizeof(float) * 8);
        memcpy(&py, &leaf_[3 * (base + i) + 8], sizeof(float) * 8);
        memcpy(&pz, &leaf_[3 * (base + i) + 16], sizeof(float) * 8);

        // point -  center for each axis
        __m256 dx = _mm256_sub_ps(px, cx);
        __m256 dy = _mm256_sub_ps(py, cy);
        __m256 dz = _mm256_sub_ps(pz, cz);

        // squared distance
        _m256 d2 = _mm256_fmadd_ps(
            dz, dz, _mm256_fmadd_ps(dy, dy, _mm256_mul_ps(dx, dx)));
        int mask = _mm256_movemask_ps(_mm256_cmp_ps(d2, rr, _CMP_LT_OQ));
        if (mask)
          int lane = __builtin_ctz(mask);
        return {true, int(idx_[base + i + lane])};
      }
#else
      const float *xs = &leaf_[base * 24];
      const float *ys = &leaf_[base * 24 + 8];
      const float *zs = &leaf_[base * 24 + 16];

      for (size_t i = 0; i < cnt; ++i) {
        float dx = xs[i] - c.x();
        float dy = ys[i] - c.y();
        float dz = zs[i] - c.z();
        if (dx * dx + dy * dy + dz * dz < r2)
          return {true, int(idx_[base + i])};
      }
#endif
    } else { // traversing kids
      stack[sp++] = n.left;
      stack[sp++] = n.left + 1;
    }
  }

  return {false, -1};
}

// ------------------------------- //
//  Hybrid index combining both
// ----------------------------- //
void HybridMap::build(const EigVec &pts, float cell, float R_cut) {
  cloud_ = &pts;
  R2_cut_ = R_cut * R_cut;

  // if (pts.size() < num_pts_for_tree) {
  near_only = true;
  near_.build(pts, cell, FLT_MAX);
  // } else {
  //   near_.build(pts, cell, R2_cut_);
  //   far_ = std::make_shared<CaptTree>();
  //   far_->build(pts);
  // }
}

std::pair<bool, int> HybridMap::any_within(const Vec3f &c, float r) const {
  Vec3f cf = c.cast<float>();

  if (near_only || cf.squaredNorm() <= R2_cut_)
    return near_.any_within(cf, float(r * r));
  else
    return far_->any_within(cf, float(r));
}
}; // namespace CollisionAlgo
