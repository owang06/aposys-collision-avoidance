#include "collision/body_spheres.hpp"
#include <cfloat>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <queue>
#include <random>
#include <unordered_map>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace detail {
using Vec3 = Eigen::Vector3f;
using EigVec = Adhoc::EigVec;
using Matrix3 = Eigen::Matrix3f;

static inline std::mt19937_64 &global_rng() {
  static std::mt19937_64 rng{std::random_device{}()};
  return rng;
}

inline void sample_box_faces(EigVec &pts, const Box &box, int res_u, int res_v,
                             float jitter) {
  Vec3 half{box.width * 0.5f, box.height * 0.5f, box.depth * 0.5f};
  pts.reserve(pts.size() + 6 * res_u * res_v);

  std::uniform_real_distribution<float> u01{0.f, 1.f};

  const Vec3 normals[6] = {{+1, 0, 0}, {-1, 0, 0}, {0, +1, 0},
                           {0, -1, 0}, {0, 0, +1}, {0, 0, -1}};
  const Vec3 u_dirs[6] = {{0, +1, 0}, {0, +1, 0}, {+1, 0, 0},
                          {+1, 0, 0}, {+1, 0, 0}, {+1, 0, 0}};
  const Vec3 v_dirs[6] = {{0, 0, +1}, {0, 0, +1}, {0, 0, +1},
                          {0, 0, +1}, {0, +1, 0}, {0, +1, 0}};
  const float u_ext[6] = {box.height, box.height, box.width,
                          box.width,  box.width,  box.width};
  const float v_ext[6] = {box.depth, box.depth,  box.depth,
                          box.depth, box.height, box.height};

  for (int f = 0; f < 6; ++f) {
    const Vec3 &n = normals[f];
    const Vec3 &ud = u_dirs[f];
    const Vec3 &vd = v_dirs[f];
    float ue = u_ext[f];
    float ve = v_ext[f];

    float d = n.dot(half);
    Vec3 face_center = box.centre + n * d;

    for (int iu = 0; iu < res_u; ++iu) {
      float u = static_cast<float>(iu) / (res_u - 1) - 0.5f;
      for (int iv = 0; iv < res_v; ++iv) {
        float v = static_cast<float>(iv) / (res_v - 1) - 0.5f;
        Vec3 p = face_center + ud * (u * ue) + vd * (v * ve);

        if (jitter > 0.f) {
          float mag = std::cbrt(u01(global_rng())) * jitter; // bias outward
          p += n * mag;
        }
        pts.push_back(p);
      }
    }
  }
}

// -------------------- inject corners / edge mids ---------------------
inline void inject_key_points(EigVec &pts, const Box &box, bool add_edge_mids) {
  Vec3 half{box.width * 0.5f, box.height * 0.5f, box.depth * 0.5f};

  // 8 corners
  for (int sx : {-1, 1})
    for (int sy : {-1, 1})
      for (int sz : {-1, 1})
        pts.emplace_back(box.centre +
                         Vec3(sx * half.x(), sy * half.y(), sz * half.z()));

  if (!add_edge_mids)
    return;
  // 12 edge centres
  const int s[2] = {-1, 1};
  for (int sx : s)
    for (int sy : s)
      pts.emplace_back(box.centre + Vec3(sx * half.x(), sy * half.y(), 0));
  for (int sx : s)
    for (int sz : s)
      pts.emplace_back(box.centre + Vec3(sx * half.x(), 0, sz * half.z()));
  for (int sy : s)
    for (int sz : s)
      pts.emplace_back(box.centre + Vec3(0, sy * half.y(), sz * half.z()));
}
/** PCA frame (falls back to I on near‑isotropic shapes). */
static std::pair<Vec3, Matrix3> robust_frame(const EigVec &P,
                                             float sym_thresh) {
  assert(!P.empty());
  Vec3 mu = Vec3::Zero();
  for (const auto &p : P)
    mu += p;
  mu /= static_cast<float>(P.size());

  Matrix3 C = Matrix3::Zero();
  for (const auto &p : P) {
    Vec3 d = p - mu;
    C += d * d.transpose();
  }
  C /= static_cast<float>(P.size());

  Eigen::SelfAdjointEigenSolver<Matrix3> es(C);
  auto ev = es.eigenvalues();
  float anis = (ev.maxCoeff() - ev.minCoeff()) / std::max(ev.maxCoeff(), 1e-6f);

  Matrix3 R = (anis < sym_thresh) ? Matrix3::Identity() : es.eigenvectors();
  return {mu, R};
}

/* Voxel key & hash so we can flood‑fill components on an eps‑grid. */
struct VoxelKey {
  int x, y, z;
};
struct KeyHash {
  std::size_t operator()(const VoxelKey &k) const noexcept {
    return 73856093u * static_cast<uint32_t>(k.x) ^
           19349663u * static_cast<uint32_t>(k.y) ^
           83492791u * static_cast<uint32_t>(k.z);
  }
};
inline bool operator==(const VoxelKey &a, const VoxelKey &b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

// ------------------------- find_components ---------------------------
struct Component {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<int> idx; ///< indices into Q
  Vec3 qmin{FLT_MAX, FLT_MAX, FLT_MAX}, qmax{-FLT_MAX, -FLT_MAX, -FLT_MAX};
};

using ComponentVec = std::vector<Component>;
inline ComponentVec find_components(const EigVec &Q, float eps) {
  using Map = std::unordered_map<VoxelKey, std::vector<int>, KeyHash>;
  Map vox;
  vox.reserve(static_cast<std::size_t>(Q.size() * 1.1f));

  // PCA‑space bounding box → axis‑scaled eps per dimension
  Vec3 qmin = Q.front(), qmax = Q.front();
  for (const auto &p : Q) {
    qmin = qmin.cwiseMin(p);
    qmax = qmax.cwiseMax(p);
  }
  Vec3 ext = qmax - qmin;
  float maxExt = std::max(ext.maxCoeff(), 1e-6f);
  Vec3 axis_eps = (ext / maxExt) * eps;
  for (int i = 0; i < 3; ++i)
    if (axis_eps[i] < 1e-6f)
      axis_eps[i] = eps; // clamp

  auto vkey = [&](const Vec3 &q) {
    return VoxelKey{int(std::floor(q.x() / axis_eps.x())),
                    int(std::floor(q.y() / axis_eps.y())),
                    int(std::floor(q.z() / axis_eps.z()))};
  };

  for (int i = 0; i < (int)Q.size(); ++i)
    vox[vkey(Q[i])].push_back(i);

  std::vector<int> label(Q.size(), -1);
  ComponentVec comps;
  comps.reserve(8); // typical boxes small number of comps

  for (const auto &kv : vox) {
    for (int root : kv.second) {
      if (label[root] >= 0)
        continue;

      comps.emplace_back();
      Component &C = comps.back();
      std::vector<int> bfs;
      bfs.reserve(32);
      bfs.push_back(root);
      label[root] = int(comps.size()) - 1;

      for (size_t head = 0; head < bfs.size(); ++head) {
        int idx = bfs[head];
        C.idx.push_back(idx);
        C.qmin = C.qmin.cwiseMin(Q[idx]);
        C.qmax = C.qmax.cwiseMax(Q[idx]);
        VoxelKey vk = vkey(Q[idx]);
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz) {
              auto it = vox.find({vk.x + dx, vk.y + dy, vk.z + dz});
              if (it == vox.end())
                continue;
              for (int j : it->second) {
                if (label[j] < 0) {
                  label[j] = label[root];
                  bfs.push_back(j);
                }
              }
            }
      }
    }
  }
  return comps;
}

/** C – split k_max across components by volume */
inline std::vector<int> allocate_budget(const ComponentVec &comps, int k_max) {
  size_t m = comps.size();
  assert(k_max >= static_cast<int>(m) && "k_max must be >= #components");
  std::vector<float> vols(m), exact(m);
  float total_vol = 0.f;
  for (size_t i = 0; i < m; ++i) {
    Vec3 ext = comps[i].qmax - comps[i].qmin;
    vols[i] = ext.x() * ext.y() * ext.z();
    total_vol += vols[i];
  }
  std::vector<int> k(m);
  for (size_t i = 0; i < m; ++i) {
    exact[i] = total_vol > 0 ? (k_max * (vols[i] / total_vol)) : 1.f;
    k[i] = std::max(1, int(std::floor(exact[i])));
  }
  int sum = std::accumulate(k.begin(), k.end(), 0);
  int leftover = k_max - sum;
  std::vector<size_t> order(m);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
    return (exact[a] - k[a]) > (exact[b] - k[b]);
  });
  for (int j = 0; j < leftover; ++j)
    k[order[j]]++;
  return k;
}

inline Sphere::Vec grid_spheres_one(const EigVec &P, const Component &C,
                                    const Matrix3 &R, const Vec3 &mu, int k_c,
                                    float epsilon, bool keep_voids,
                                    bool tighten) {
  Vec3 ext = C.qmax - C.qmin;
  float GM = std::cbrt(ext.prod());
  Vec3 ideal = Vec3::Ones();
  if (k_c > 1)
    ideal = ext / GM * std::pow(float(k_c), 1.f / 3.f);

  Eigen::Vector3i n(std::max(1, int(std::round(ideal.x()))),
                    std::max(1, int(std::round(ideal.y()))),
                    std::max(1, int(std::round(ideal.z()))));
  while (n.prod() > k_c) {
    int idx;
    n.maxCoeff(&idx);
    n[idx]--;
  }
  Vec3 step = ext.cwiseQuotient(n.cast<float>());
  float r_base = 0.5f * step.norm() + epsilon;

  Sphere::Vec local;
  local.reserve(static_cast<size_t>(n.prod()));
  for (int ix = 0; ix < n.x(); ++ix)
    for (int iy = 0; iy < n.y(); ++iy)
      for (int iz = 0; iz < n.z(); ++iz) {
        Vec3 q_local =
            C.qmin + step.cwiseProduct(Vec3(ix + 0.5f, iy + 0.5f, iz + 0.5f));
        local.push_back({(R * q_local + mu).cast<float>(), r_base});
      }

  // redundancy pruning
  float inner = r_base - (keep_voids ? 0.f : epsilon);
  float inner2 = inner * inner;
  Sphere::Vec kept;
  kept.reserve(local.size());
  for (const auto &s : local) {
    bool inside = false;
    if (!keep_voids) {
      for (int id : C.idx) {
        if ((P[id] - s.offset).squaredNorm() < inner2) {
          inside = true;
          break;
        }
      }
    }
    if (inside || keep_voids)
      kept.push_back(s);
  }
  local.swap(kept);

  // tighten radii
  if (tighten) {
    for (auto &s : local) {
      float r2 = 0.f;
      for (int id : C.idx) {
        float d2 = (P[id] - s.offset).squaredNorm();
        if (d2 > r2)
          r2 = d2;
      }
      s.radius = std::max(std::sqrt(r2) + epsilon, epsilon);
    }
  }
  return local;
}

inline float dist_to_nearest_sphere(const Vec3 &p, const Sphere::Vec &S) {
  float d_min = FLT_MAX;
  for (const auto &s : S) {
    d_min = std::min(d_min, (p - s.offset).norm() - s.radius);
  }
  return d_min;
}
} // namespace detail

Sphere::Vec Adhoc::make_body_spheres(const Box &box, const SphereParams &cfg,
                                     EigVec &pts) {
  using namespace detail;

  // A. sample surface
  sample_box_faces(pts, box, cfg.res_u, cfg.res_v, cfg.jitter);
  inject_key_points(pts, box, cfg.add_edge_mids);

  // B. robust frame
  auto [mu, R] = robust_frame(pts, cfg.sym_thresh);

  // C. PCA coords
  EigVec Q;
  Q.reserve(pts.size());
  for (const auto &p : pts)
    Q.emplace_back(R.transpose() * (p - mu));

  // D. components
  auto comps = find_components(Q, cfg.epsilon);

  // E. budget
  auto kAlloc = allocate_budget(comps, cfg.k_max);

  // F. per‑component grid spheres (parallel)
  Sphere::Vec spheres;
  spheres.reserve(cfg.k_max + 20);
  std::vector<Sphere::Vec> locals(comps.size());
#pragma omp parallel for schedule(static)
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(comps.size()); ++i)
    locals[i] = grid_spheres_one(pts, comps[i], R, mu, kAlloc[i], cfg.epsilon,
                                 cfg.keep_voids, cfg.tighten);

  for (auto &v : locals)
    spheres.insert(spheres.end(), v.begin(), v.end());

  // G. corner fail‑safe
  Vec3 half{box.width * 0.5f, box.height * 0.5f, box.depth * 0.5f};
  for (int sx : {-1, 1})
    for (int sy : {-1, 1})
      for (int sz : {-1, 1}) {
        Vec3 c = box.centre + Vec3(sx * half.x(), sy * half.y(), sz * half.z());
        if (dist_to_nearest_sphere(c, spheres) > cfg.epsilon) {
          spheres.push_back({c, cfg.epsilon});
        }
      }

  return spheres;
}
