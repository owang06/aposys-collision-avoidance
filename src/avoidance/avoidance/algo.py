import rclpy, numpy as np
from rclpy.node import Node
from rclpy.time import Time as RclTime
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from radar_types.msg import Burst
from message_filters import Subscriber, Cache
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
import collision_geom as cg
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))
from avoidance.helper import *
import pygame

class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor')

        # -------- Parameters --------
        self.declare_parameter('safety_horizon', 2.0)      # s
        self.declare_parameter('rollout_dt', 0.1)         # s
        self.declare_parameter('imu_cache', 400)           # msgs
        self.declare_parameter('pose_cache', 200)          # msgs
        self.declare_parameter('twist_cache', 200)         # msgs
        self.declare_parameter('sync_slop', 0.01)          # s
        self.declare_parameter('vehicle_width', 2.036)
        self.declare_parameter('vehicle_length', 13.6)
        self.declare_parameter('vehicle_height', 0.0)
        self.declare_parameter('threshold_high', 1.60)   # m  - overlap or exactly touching
        self.declare_parameter('threshold_med',  4.00)   # m  - within 50_cm
        self.declare_parameter('threshold_low',  7.00)   # m  - within 1_m
        self.declare_parameter('safety_distance', 1.00)
        self.declare_parameter('imu_only', True)
        self.declare_parameter('stop_speed', 0.25)

        # ---- Sound system ---
        pygame.mixer.init()
        self._beep_slow  = pygame.mixer.Sound("src/avoidance/sounds/slowbeep.mp3")
        self._beep_med   = pygame.mixer.Sound("src/avoidance/sounds/mediumbeep.mp3")
        self._beep_fast  = pygame.mixer.Sound("src/avoidance/sounds/fastbeep.mp3")
        self._current_snd = None

        # -------- Subscriptions --------
        self.imu_only = self.get_parameter('imu_only').value

        # Custom QoS for the radar topic
        radar_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 10
        )
        self.radar_sub = Subscriber(self, Burst,       'mr76/burst', qos_profile = radar_qos)
        self.imu_sub   = Subscriber(self, Imu,         '/imu/data')

        if not self.imu_only:
            self.pose_sub  = Subscriber(self, PoseStamped, '/aposys/pose')
            self.twist_sub  = Subscriber(self, TwistStamped, '/aposys/twist')

        # Approximate time caches
        self.imu_cache   = Cache(self.imu_sub,   cache_size=400)
        self.pose_cache  = Cache(self.pose_sub,  cache_size=200)  if not self.imu_only else None
        self.twist_cache = Cache(self.twist_sub, cache_size=200)  if not self.imu_only else None

        self.alert_pub = self.create_publisher(String, 'collision_warning', 10)

        # -------- Prototype list of spheres in the body frame --------
        self._offsets = None
        self._radii   = None
        self._initialize_spheres(width=self.get_parameter('vehicle_width').value,
                                 height=self.get_parameter('vehicle_height').value,
                                 length=self.get_parameter('vehicle_length').value,
                                 safety=self.get_parameter('safety_distance').value)

        # Radar callback
        self.radar_sub.registerCallback(self.on_radar)


    def _play_beep(self, risk: str | None) -> None:
        """Start or switch looping sound according to `risk`."""
        snd = {'LOW':  self._beep_slow,
               'MED':  self._beep_med,
               'HIGH': self._beep_fast}.get(risk)

        if snd is self._current_snd:              # already playing correct track
            return
        self._stop_beep()                         # stop whatever was playing
        if snd is not None:
            snd.play(loops=-1)                    # infinite loop
            self._current_snd = snd

    def _stop_beep(self) -> None:
        """Silence any playing warning."""
        if self._current_snd is not None:
            self._current_snd.stop()
            self._current_snd = None

    def _initialize_spheres(self,
                        width:  float,
                        height: float,
                        length: float,
                        safety: float = 0.20):
        """
        Create safety spheres at:
        - 8 vehicle corners
        - 6 face centers (middle of each face)
        - 1 center of the vehicle
        """

        hx, hy, hz = 0.5 * np.array([length, width, height], dtype=np.float32)

        # 8 corner points
        signs = (-1, +1)
        corners = np.array([[sx * hx, sy * hy, sz * hz]
                            for sx in signs for sy in signs for sz in signs],
                        dtype=np.float32)

        # 6 face centers
        face_centers = np.array([
            [ 0.0,  0.0, +hz],  # top
            [ 0.0,  0.0, -hz],  # bottom
            [ 0.0, +hy,  0.0],  # right
            [ 0.0, -hy,  0.0],  # left
            [+hx,  0.0,  0.0],  # front
            [-hx,  0.0,  0.0],  # back
        ], dtype=np.float32)

        # 1 center
        center = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)

        # Combine all offsets
        offsets = np.vstack([corners, face_centers, center])  # shape (15,3)

        # Create spheres
        self.base_spheres = [cg.Sphere(offset, safety) for offset in offsets]

        # Cache for rollout
        self._offsets = offsets.copy()
        self._radii   = np.full(len(self.base_spheres), safety, dtype=np.float64)

    def stamp_to_sec(self, stamp: Time) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def caches_ready(self):
        """True when we have at least one message from every **active** topic."""
        if self.imu_cache.getLast() is None:
            return False
        if not self.imu_only:
            return (self.pose_cache.getLast()  is not None and
                    self.twist_cache.getLast() is not None)
        return True

    def _start_caches(self):
        """Instantiate caches the first time we see radar."""
        self.get_logger().info('First radar burst received - starting caches')
        self.imu_cache = Cache(
            self.imu_sub,
            cache_size=self.get_parameter('imu_cache').value)
        self.pose_cache = Cache(
            self.pose_sub,
            cache_size=self.get_parameter('pose_cache').value)
        self.twist_cache = Cache(
            self.twist_sub,
            cache_size=self.get_parameter('twist_cache').value)

    @staticmethod
    def _bracket(cache, stamp_msg):
        """
        Return (msg_before, msg_after) that straddle <stamp_msg> in time,
        or (None, None) if either neighbour is missing.
        """
        stamp = RclTime.from_msg(stamp_msg)          # <-- key line

        before = cache.getElemBeforeTime(stamp)
        after  = cache.getElemAfterTime(stamp)
        return before, after

    @staticmethod
    def _interp_ratio(t_query: float, t0: float, t1: float, eps: float = 1e-9) -> float:
        """
        Return interpolation factor r ∈ [0,1] such that
            r = 0  -> use sample at t0
            r = 1  -> use sample at t1
        Guarded for the degenerate case t0 == t1.
        """
        denom = t1 - t0
        if abs(denom) < eps:
            return 0.0
        return (t_query - t0) / denom

    def sample_pose(self, t: Time):
        """Return pose exactly at stamp `t` using linear interpolation."""
        if self.imu_only:
            # Body frame = world frame → zero translation, identity quaternion
            return np.zeros(3), np.array([0, 0, 0, 1], dtype=float)

        before, after = self._bracket(self.pose_cache, t)
        if before is None and after is None:
            self.get_logger().info("Both empty")
            return None                         # cache still empty

        if after is None or before is None:
            # One‑sided -> just use the sample we have
            m = before if after is None else after
            pos = np.array([m.pose.position.x,
                            m.pose.position.y,
                            m.pose.position.z])
            quat = [m.pose.orientation.x, m.pose.orientation.y,
                    m.pose.orientation.z, m.pose.orientation.w]
            return pos, quat

        # ratio in [0,1]
        t_query = self.stamp_to_sec(t)
        t0 = self.stamp_to_sec(before.header.stamp)
        t1 = self.stamp_to_sec(after. header.stamp)
        r  = self._interp_ratio(t_query, t0, t1)

        # position — simple lerp
        p0 = np.array([before.pose.position.x,
                       before.pose.position.y,
                       before.pose.position.z])
        p1 = np.array([after.pose.position.x,
                       after.pose.position.y,
                       after.pose.position.z])
        pos = (1-r)*p0 + r*p1

        # orientation — slerp using scipy or tf_transform
        q0 = [before.pose.orientation.x, before.pose.orientation.y,
              before.pose.orientation.z, before.pose.orientation.w]
        q1 = [after.pose.orientation.x, after.pose.orientation.y,
              after.pose.orientation.z, after.pose.orientation.w]
        quat = quaternion_slerp(q0, q1, r)

        return pos, quat

    def sample_imu(self, t: Time):
        """Return angular velocity & linear acceleration vectors exactly at t."""
        before, after = self._bracket(self.imu_cache, t)
        if before is None and after is None:
            return None

        if after is None or before is None:
            m = before if after is None else after
            w = np.array([m.angular_velocity.x,
                        m.angular_velocity.y,
                        m.angular_velocity.z])
            a = np.array([m.linear_acceleration.x,
                        m.linear_acceleration.y,
                        m.linear_acceleration.z])
            return w, a

        t_query = self.stamp_to_sec(t)
        t0 = self.stamp_to_sec(before.header.stamp)
        t1 = self.stamp_to_sec(after. header.stamp)
        r  = self._interp_ratio(t_query, t0, t1)

        # slerp (imu is small dt so linear is fine)
        w0 = np.array([before.angular_velocity.x,
                       before.angular_velocity.y,
                       before.angular_velocity.z])
        w1 = np.array([after.angular_velocity.x,
                       after.angular_velocity.y,
                       after.angular_velocity.z])
        a0 = np.array([before.linear_acceleration.x,
                       before.linear_acceleration.y,
                       before.linear_acceleration.z])
        a1 = np.array([after.linear_acceleration.x,
                       after.linear_acceleration.y,
                       after.linear_acceleration.z])
        return (1-r)*w0 + r*w1, (1-r)*a0 + r*a1

    def sample_twist(self, t: Time):
        if self.imu_only:
            return np.zeros(3)

        before, after = self._bracket(self.twist_cache, t)
        if before is None and after is None:
            return None

        if after is None or before is None:
            m = before if after is None else after
            return np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])

        r = (self.stamp_to_sec(t) -
            self.stamp_to_sec(before.header.stamp)) / \
            (self.stamp_to_sec(after.header.stamp) -
            self.stamp_to_sec(before.header.stamp))
        v0 = (1 - r) * np.array([before.twist.linear.x,
                                before.twist.linear.y,
                                before.twist.linear.z]) + \
            r * np.array([after.twist.linear.x,
                        after.twist.linear.y,
                        after.twist.linear.z])
        return v0

    def _min_clearance(self, points_world: np.ndarray, spheres) -> float:
        """Smallest (distance – radius) between any point and any sphere layer."""
        d_min = np.inf
        for s in spheres:
            centre = np.asarray(s.offset)
            r      = float(s.radius)
            d      = np.linalg.norm(points_world - centre, axis=1).min() - r
            if d < d_min:
                d_min = d
        return d_min

    def _rollout_spheres(self, pose, imu, v0):
        """Wrapper around `rollout_spheres` from helper.py for convenience."""
        horizon = self.get_parameter('safety_horizon').value
        dt_step = self.get_parameter('rollout_dt').value
        g       = np.array([0., 0., -9.81])
        return rollout_spheres(self._offsets, self._radii,
                            pose, imu, v0,
                            horizon, dt_step, gravity=g)

    def on_radar(self, radar_msg: Burst) -> None:
        print("before")
        if not self.caches_ready():
            return
        
        print("after")

        t_query = radar_msg.header
        pose    = self.sample_pose(t_query)
        imu     = self.sample_imu(t_query)
        vel     = self.sample_twist(t_query)

        if imu is None:
            self.get_logger().warn('No IMU data buffered yet')
            return

        p0, q0 = pose
        R0     = quat_to_R(q0)

        cloud_body = np.array([[o.x_m, o.y_m, 0.0] for o in radar_msg.objects],
                              dtype=np.float32)
        if cloud_body.size == 0:
            self._play_beep(None)
            return

        if self.imu_only:
            cloud_frame = cloud_body
        else:
            cloud_frame = (R0 @ cloud_body.T).T + p0   # world frame

        #  Roll‑out & risk evaluation
        if self.imu_only:
            # No reliable world motion → instant clearance only
            rollout_layers = [self.base_spheres]          # single layer
            dt_step        = 0.0 
        else:
            rollout_layers = self._rollout_spheres(pose, imu, vel)
            dt_step        = self.get_parameter('rollout_dt').value

        t_first_impact = None
        min_clear      = np.inf

        for k, layer in enumerate(rollout_layers, start=1):
            d = self._min_clearance(cloud_frame, layer)
            if d < min_clear:
                min_clear = d
            if d <= 0.0 and t_first_impact is None:
                t_first_impact = k * dt_step
        
        print(f"MIN CLEARANCE: {min_clear}")

        # ----------------------------------------------------------
        #  Map distance → qualitative risk
        # ----------------------------------------------------------
        thr_high = self.get_parameter('threshold_high').value
        thr_med  = self.get_parameter('threshold_med').value
        thr_low  = self.get_parameter('threshold_low').value

        print(f"THR HIGH: {thr_high} THR MED: {thr_med} THR LOW: {thr_low}")

        risk = None
        if min_clear <= thr_high:
            risk = 'HIGH'
        elif min_clear <= thr_med:
            risk = 'MED'
        elif min_clear <= thr_low:
            risk = 'LOW'

        # If the vehicle is effectively stopped, suppress the beeper
        speed = np.linalg.norm(vel) if vel is not None else 0.0
        # if speed <= self.get_parameter('stop_speed').value:
        #     risk = None

        # ----------------------------------------------------------
        #  Notify & sound
        # ----------------------------------------------------------
        self._play_beep(risk)

        if risk is not None:                               # build log / topic
            msg = String()
            if t_first_impact is not None and dt_step > 0:
                msg.data = (f'{risk} - predicted contact in '
                            f'{t_first_impact:.2f}_s '
                            f'(clearance {min_clear:.2f}_m, v={speed:.2f}_m/s)')
            else:
                msg.data = (f'{risk} - minimum clearance {min_clear:.2f}_m '
                            f'(v={speed:.2f}_m/s)')
            self.alert_pub.publish(msg)
            self.get_logger().warn(msg.data)
        else:
            self._stop_beep()

    def destroy_node(self):
        """Ensure pygame shuts down cleanly when the node is destroyed."""
        self._stop_beep()
        pygame.mixer.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init()
    node = CollisionMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
