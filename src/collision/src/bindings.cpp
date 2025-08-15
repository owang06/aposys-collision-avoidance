#include "collision/algo.hpp"
#include "collision/body_spheres.hpp"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace py = pybind11;

namespace detail {
static Adhoc::EigVec
to_cloud(py::array_t<float, py::array::c_style | py::array::forcecast> arr) {
  if (arr.ndim() != 2 || arr.shape(1) != 3)
    throw std::runtime_error("cloud must be shape (N,3)");

  int N = int(arr.shape(0));
  Adhoc::EigVec out(N);
  auto buf = arr.unchecked<2>();

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < N; ++i)
    out[i] = CollisionAlgo::Vec3f(buf(i, 0), buf(i, 1), buf(i, 2));

  return out;
}
} // namespace detail

PYBIND11_MODULE(collision_geom, m) {
  /* ------------ Details related to Body Sphere approximation -------- */
  py::class_<Box>(m, "Box")
      .def(py::init<>())                     // default ctor
      .def_readwrite("centre", &Box::centre, // Eigen::Vector3f
                     "Centre of the axis‑aligned box")
      .def_readwrite("width", &Box::width)
      .def_readwrite("height", &Box::height)
      .def_readwrite("depth", &Box::depth)
      .def("__repr__", [](const Box &b) {
        std::ostringstream oss;
        oss << "<Box centre=[" << b.centre.x() << "," << b.centre.y() << ","
            << b.centre.z() << "], w=" << b.width << ", h=" << b.height
            << ", d=" << b.depth << ">";
        return oss.str();
      });

  py::class_<Sphere>(m, "Sphere")
      .def(py::init<const Eigen::Vector3f &, double>())
      .def_readonly("offset", &Sphere::offset)
      .def_readonly("radius", &Sphere::radius)
      .def("__repr__", [](const Sphere &s) {
        return "<Sphere center=[" + std::to_string(s.offset.x()) + "," +
               std::to_string(s.offset.y()) + "," +
               std::to_string(s.offset.z()) +
               "],  r=" + std::to_string(s.radius) + ">";
      });

  py::class_<SphereParams>(m, "SphereParams")
      .def(py::init<>()) // default
      .def_readwrite("epsilon", &SphereParams::epsilon)
      .def_readwrite("k_max", &SphereParams::k_max)
      .def_readwrite("tighten", &SphereParams::tighten)
      .def_readwrite("keep_voids", &SphereParams::keep_voids)
      .def_readwrite("sym_thresh", &SphereParams::sym_thresh)
      .def_readwrite("add_edge_mids", &SphereParams::add_edge_mids)
      .def_readwrite("res_u", &SphereParams::res_u)
      .def_readwrite("res_v", &SphereParams::res_v)
      .def_readwrite("jitter", &SphereParams::jitter);

  py::class_<CollisionAlgo::CollideInfo>(m, "CollideInfo")
      .def_readonly("body", &CollisionAlgo::CollideInfo::body)
      .def_readonly("sphere", &CollisionAlgo::CollideInfo::sphere)
      .def_readonly("point", &CollisionAlgo::CollideInfo::point)
      .def_readonly("hit", &CollisionAlgo::CollideInfo::hit)
      .def("__repr__", [](const CollisionAlgo::CollideInfo &s) {
        std::ostringstream oss;
        oss << "<CollideInfo body_id=" << s.body << ", sphere_id=" << s.sphere
            << ", point_id=" << s.point << ", hit=[" << s.hit.x() << ","
            << s.hit.y() << "," << s.hit.z() << "]>";
        return oss.str();
      });

  /* ------------ Main user Functions ------------------ */
  m.def(
      "make_body_spheres",
      // ------------------------ overload #1 --------------------
      //   (box   , params)  ->  List[Sphere]
      [](const Box &box, const SphereParams &params) {
        Adhoc::EigVec scratch; // we don't need the samples
        py::gil_scoped_release unlock;
        return Adhoc::make_body_spheres(box, params, scratch);
      },
      py::arg("box"), py::arg("params") = SphereParams{},
      R"pbdoc(
        Cover the given axis‑aligned Box with spheres.

        Returns
        -------
        List[Sphere]
    )pbdoc");

  //-------------------------------------------------------------
  m.def(
      "make_body_spheres",
      // ------------------------ overload #1 --------------------
      //   (box   , params)  ->  List[Sphere]
      [](const Box &box, const SphereParams &params) {
        Adhoc::EigVec scratch; // we don't need the samples
        py::gil_scoped_release unlock;
        return Adhoc::make_body_spheres(box, params, scratch);
      },
      py::arg("box"), py::arg("params") = SphereParams{},
      R"pbdoc(
        Cover the given axis‑aligned Box with spheres.

        Returns
        -------
        List[Sphere]
    )pbdoc");

  m.def(
      "make_body_spheres",
      // ------------------------ overload #2 --------------------
      //   (box, params, return_samples=True) ->
      //        (List[Sphere], ndarray(M,3))
      [](const Box &box, const SphereParams &params,
         bool return_samples) -> py::object {
        Adhoc::EigVec samples;
        py::gil_scoped_release unlock;
        auto spheres = Adhoc::make_body_spheres(box, params, samples);

        if (!return_samples)
          return py::cast(spheres); // just spheres

        // Convert samples -> NumPy
        ssize_t M = static_cast<ssize_t>(samples.size());
        std::vector<py::ssize_t> shape = {static_cast<py::ssize_t>(M), 3};
        py::array_t<float> np_samples(shape);

        auto buf = np_samples.mutable_unchecked<2>();
        for (ssize_t i = 0; i < M; ++i) {
          buf(i, 0) = samples[i].x();
          buf(i, 1) = samples[i].y();
          buf(i, 2) = samples[i].z();
        }
        return py::make_tuple(spheres, np_samples);
      },
      py::arg("box"), py::arg("params") = SphereParams{},
      py::arg("return_samples") = false,
      R"pbdoc(
        Same as the overload above but, when `return_samples=True`,
        also returns the internal surface‑sampling points that drove
        the PCA & grid algorithm.

        Returns
        -------
        List[Sphere]                        # default
        or
        (List[Sphere], ndarray(M,3))        # if return_samples
    )pbdoc");

  // ------------------------------------------------------------------
  //  Python‑friendly helper that takes raw dimensions.
  //    * length  → box.depth   (Z)
  //    * height  → box.height  (Y)
  //    * width   → box.width   (X)
  //  Returns either List[Sphere]  OR  (List[Sphere], ndarray(M,3))
  // ------------------------------------------------------------------
  m.def(
      "make_body_spheres_from_dims",
      [](float length, float height, float width,
         const SphereParams &params = SphereParams{},
         bool return_samples = false) -> py::object {
        // 1.  Build the Box (centre at origin)
        Box box;
        box.centre = Eigen::Vector3f::Zero();
        box.width = width;   // X
        box.height = height; // Y
        box.depth = length;  // Z  (name keeps automotive sense)

        // 2.  Call the C++ routine
        Adhoc::EigVec samples;
        py::gil_scoped_release unlock;
        auto spheres = Adhoc::make_body_spheres(box, params, samples);

        // 3.  Package return value
        if (!return_samples)
          return py::cast(spheres);

        // convert samples → NumPy
        ssize_t M = static_cast<ssize_t>(samples.size());
        std::vector<py::ssize_t> shape = {static_cast<py::ssize_t>(M), 3};
        py::array_t<float> np_samples(shape);
        auto buf = np_samples.mutable_unchecked<2>();
        for (ssize_t i = 0; i < M; ++i) {
          buf(i, 0) = samples[i].x();
          buf(i, 1) = samples[i].y();
          buf(i, 2) = samples[i].z();
        }
        return py::make_tuple(spheres, np_samples);
      },
      py::arg("length"), // Z
      py::arg("height"), // Y
      py::arg("width"),  // X
      py::arg("params") = SphereParams{}, py::arg("return_samples") = false,
      R"pbdoc(
          Cover a box given by (length, height, width) with spheres.

          Parameters
          ----------
          length : float
              Size of the box along Z.
          height : float
              Size of the box along Y.
          width : float
              Size of the box along X.
          params : SphereParams, optional
              Tuning knobs (k_max, epsilon, …).
          return_samples : bool, optional
              When True, also returns the algorithm's internal
              surface-sampling points.

          Returns
          -------
          List[Sphere]
              if return_samples is False

          (List[Sphere], ndarray(M,3))
              if return_samples is True
      )pbdoc");

  /* ------------ Details related to Collision Check -------- */
  using HMap = CollisionAlgo::HybridMap;
  py::class_<HMap>(m, "HybridMap")
      .def(py::init<>())
      .def(
          "build",
          [](HMap &h, // build from NumPy
             py::array_t<float, py::array::c_style | py::array::forcecast> pts,
             float voxel_cell, float r_split) {
            auto cloud = detail::to_cloud(pts);

            h.build(cloud, voxel_cell, r_split);
          },
          py::arg("points"), py::arg("voxel_cell") = 0.05f,
          py::arg("r_split") = 3.0f, "Build hybrid map from point cloud.")
      .def(
          "any_within",
          [](HMap &h, py::sequence center, float radius) {
            CollisionAlgo::Vec3f c{py::float_(center[0]), py::float_(center[1]),
                                   py::float_(center[2])};

            auto [hit, idx] = h.any_within(c, radius);
            return py::make_tuple(hit, idx);
          },
          py::arg("center"), py::arg("radius"),
          R"pbdoc(Check if any point in the cloud lies within `radius` of `center`.
                  Returns
                  -------
                  (hit: bool, point_idx: int)
                  )pbdoc");

  /* ------------ Main user Functions ------------------ */
  m.def(
      "collision_free",
      [](const Sphere::Vec &body, HMap &H,
         py::array_t<float, py::array::c_style | py::array::forcecast> pts,
         bool debug = false) -> py::object {
        // convert cloud
        auto cloud = detail::to_cloud(pts);

        // call into your header
        CollisionAlgo::CollideInfo info;
        bool ok = CollisionAlgo::collision_free(body.data(),
                                                body.data() + body.size(), H,
                                                cloud, debug ? &info : nullptr);
        if (!debug)
          return py::bool_(ok);
        else
          return py::make_tuple(ok, info);
      },
      py::arg("body"), py::arg("index"), py::arg("cloud"),
      py::arg("debug") = false,
      R"pbdoc(Check a single body (list of Sphere) against the point cloud.
        Parameters
        ----------
        body : List[Sphere]
        index : HybridMap
        cloud : (N,3) float32 array
        debug : bool
            If true, also returns CollideInfo.

        Returns
        -------
        bool or (bool, CollideInfo)
        )pbdoc");

  // ---- check_collision wrapper -----------------------------------------
  m.def(
      "check_collision",
      [](const std::vector<Sphere::Vec> &bodies, HMap &H,
         py::array_t<float, py::array::c_style | py::array::forcecast> pts) {
        // convert cloud
        auto cloud = detail::to_cloud(pts);

        // call into your header
        return CollisionAlgo::check_collision(bodies, H, cloud);
      },
      py::arg("bodies"), py::arg("index"), py::arg("cloud"),
      R"pbdoc(Collect collisions for multiple bodies.
            Parameters
            ----------
            bodies : List[List[Sphere]]
            index : HybridMap
            cloud : (N,3) float32 array

            Returns
            -------
            List[CollideInfo]
            )pbdoc");
}
