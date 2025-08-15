import numpy as np
import open3d as o3d
import collision_geom as cg
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))
from helper import *

def test_scenario(width:float, height:float, length:float, spheres:cg.Sphere, params:cg.SphereParams):
    # 1) NO COLLISION: put cloud well outside box + margin
    no_coll_center = np.array([width/2 + 2.0 + params.epsilon,
                               0.0, 0.0], dtype=np.float32)
    cloud_no = random_cloud(5000, no_coll_center, scale=1.0)
    idx_no = make_index(cloud_no)
    ok_no = cg.collision_free(spheres, idx_no, cloud_no)
    print("No-collision test passed?", ok_no)

    # 2) FULL COLLISION: put cloud dense inside the box
    full_coll_center = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    cloud_full = random_cloud(5000, full_coll_center,
                              scale=np.array([width/2, height/2, length/2], np.float32))
    idx_full = make_index(cloud_full)
    ok_full, info_full = cg.collision_free(spheres, idx_full, cloud_full, debug=True)
    print("Full-collision test passed (should be False)?", ok_full)
    print("  First hit:", info_full)

    # 3) MIXED: half points inside, half points outside
    cloud_mix_in  = random_cloud(2500, full_coll_center,
                                scale=np.array([width/4, height/4, length/4], np.float32))
    cloud_mix_out = random_cloud(2500, no_coll_center,
                                scale=1.0)
    cloud_mix = np.vstack([cloud_mix_in, cloud_mix_out])
    idx_mix = make_index(cloud_mix)
    collisions = cg.check_collision([spheres], idx_mix, cloud_mix)
    print("Mixed test found collisions:", len(collisions), "hits")
    if collisions:
        print("  Sample hit info:", collisions[0])

def run_algo(width:float, height: float, length:float, params:cg.SphereParams, res_u:int = 40, res_v:int = 20,
          jitter:float = 0.002):
    pass


def visualize_box_spheres(box: cg.Box,
                          spheres: list,            # List[cg.Sphere]
                          out_path: str = "snapshot.png",
                          bg_color: tuple = (1, 1, 1, 1)):
    """
    Render an axis‑aligned Box and its sphere cover to an off‑screen PNG.

    Parameters
    ----------
    box : cg.Box
        Axis‑aligned box (centre, width, height, depth).
    spheres : List[cg.Sphere]
        Covering spheres returned by `cg.make_body_spheres`.
    out_path : str, optional
        Destination PNG file.  Default = 'snapshot.png'.
    bg_color : 4‑tuple, optional
        RGBA background colour. Default = white.
    """
    # --- create a line set for the box --------------------------------
    half = np.array([box.width, box.height, box.depth]) * 0.5
    c    = np.asarray(box.centre)

    # 8 corners (Open3D order doesn’t matter, we define our own lines)
    corners = np.array([
        c + [+half[0], +half[1], +half[2]],
        c + [-half[0], +half[1], +half[2]],
        c + [-half[0], -half[1], +half[2]],
        c + [+half[0], -half[1], +half[2]],
        c + [+half[0], +half[1], -half[2]],
        c + [-half[0], +half[1], -half[2]],
        c + [-half[0], -half[1], -half[2]],
        c + [+half[0], -half[1], -half[2]],
    ], dtype=np.float64)

    # 12 edges as pairs of indices into `corners`
    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],   # top
        [4, 5], [5, 6], [6, 7], [7, 4],   # bottom
        [0, 4], [1, 5], [2, 6], [3, 7]    # pillars
    ]

    bbox_lines = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(corners),
        lines=o3d.utility.Vector2iVector(lines))
    bbox_lines.paint_uniform_color([0, 1, 0])       # green

    # --- sphere material (semi‑transparent red) -----------------------
    sphere_mat = o3d.visualization.rendering.MaterialRecord()
    sphere_mat.shader     = "defaultLitTransparency"
    sphere_mat.base_color = [1, 0, 0, 0.35]         # RGBA

    # --- off‑screen renderer -----------------------------------------
    w, h = 1280, 960
    renderer = o3d.visualization.rendering.OffscreenRenderer(w, h)
    scene = renderer.scene
    scene.set_background(list(bg_color))

    scene.add_geometry("bbox", bbox_lines,
                       o3d.visualization.rendering.MaterialRecord())

    # add each sphere
    for i, s in enumerate(spheres):
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius=s.radius,
                                                       resolution=24)
        mesh.compute_vertex_normals()
        mesh.translate(s.offset.astype(np.float64))
        scene.add_geometry(f"sphere_{i}", mesh, sphere_mat)

    # --- camera: front view along +Z ----------------------------------
    bb = scene.bounding_box
    center  = bb.get_center()
    extent  = max(bb.get_extent())
    eye     = center + np.array([0, 0, 1.5 * extent])
    up      = np.array([0, 1, 0])
    scene.camera.look_at(center, eye, up)
    scene.camera.set_projection(60.0, w / h, 0.1, 1000.0)

    # --- render & save ------------------------------------------------
    img = renderer.render_to_image()
    o3d.io.write_image(out_path, img)
    print(f"[visualize_box_spheres] wrote {out_path}")

def main():
    # Step 1: create a dummy vehicle point cloud
    length  = 13.6
    height  = 2.036
    width   = 0.0          # <- set whatever you need

    # params = cg.SphereParams()
    # params.k_max        = 24
    # params.add_edge_mids = True

    # # spheres only
    # spheres = cg.make_body_spheres_from_dims(length, height, width, params)

    # # spheres + sampled surface points
    # spheres, samples = cg.make_body_spheres_from_dims(length, height, width,
    #                                                 params,
    #                                                 True)
    box = cg.Box()
    box.centre = np.array([0.0, 0.0, 0.0])
    box.width  = width
    box.height = height
    box.depth  = length

    # 2. sphere params
    params = cg.SphereParams()
    params.k_max        = 24
    params.add_edge_mids = True

    # 3. generate spheres
    spheres = cg.make_body_spheres(box, params)

    # 4. visualise
    visualize_box_spheres(box, spheres, out_path="box_cover.png")


if __name__ == "__main__":
    main()
