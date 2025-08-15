import numpy as np
import collision_geom as cg
import open3d as o3d

def visualize(points: np.ndarray, spheres: list, out_path="snapshot.png"):
    # --- build point cloud ---
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd.paint_uniform_color([0.7,0.7,0.7])  # light gray

    # --- compute and draw bounding box around the cloud ---
    aabb = pcd.get_axis_aligned_bounding_box()
    aabb.color = (0.0, 1.0, 0.0)            # green
    bbox_lines = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(aabb)

    # --- prepare semi‑transparent red material for spheres ---
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "defaultLitTransparency"
    mat.base_color = [1, 0, 0, 0.5]

    # --- offscreen renderer setup ---
    w, h = 1024, 768
    renderer = o3d.visualization.rendering.OffscreenRenderer(w, h)
    scene = renderer.scene
    scene.set_background([1,1,1,1])

    # --- add geometries ---
    scene.add_geometry("pcd", pcd, o3d.visualization.rendering.MaterialRecord())
    scene.add_geometry("bbox", bbox_lines, o3d.visualization.rendering.MaterialRecord())
    for i, s in enumerate(spheres):
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius=s.radius, resolution=20)
        mesh.compute_vertex_normals()
        mesh.translate(s.offset.astype(np.float64))
        scene.add_geometry(f"sphere_{i}", mesh, mat)

    # --- camera setup ---
    bb = scene.bounding_box
    center = np.asarray(bb.get_center(), dtype=np.float32).reshape(3,1)
    extent = bb.get_extent()
    eye    = np.asarray(center.flatten() + [0, 0, max(extent)*1.5], dtype=np.float32).reshape(3,1)
    up     = np.array([0,1,0], dtype=np.float32).reshape(3,1)
    renderer.setup_camera(60.0, center, eye, up, near_clip=0.1, far_clip=1000.0)

    # --- render & save ---
    img = renderer.render_to_image()
    o3d.io.write_image(out_path, img)
    print(f"Wrote {out_path}")

def build_body(width, height, depth, params, res_u:int = 40, res_v:int = 20,
          jitter:float = 0.02, display=False):
    # 1) sample just the six faces of the box
    pts = cg.sample_regions(width, height, depth,
                            res_u, res_v, jitter)
    # 2) approximate as spheres
    spheres = cg.make_body_spheres(pts, params)
    if (display):
        visualize(pts, spheres, out_path="snapshot.png")
    return spheres

def make_index(cloud, cell=0.5, r_split=3.0):
    idx = cg.HybridMap()
    idx.build(cloud, voxel_cell=cell, r_split=r_split)
    return idx

def random_cloud(count, center, scale):
    # uniform around `center` ±scale in each axis
    return (np.random.rand(count,3) - 0.5)*2.0*scale + center
