#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from radar_types.msg import Object, Burst
from radar_pub.assembler import CycleAssembler, parse_frame, BurstBundle
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from radar_pub.radar_can import USBCAN, VCI_USBCAN1
from queue import Queue, Empty
from threading import Thread
from builtin_interfaces.msg import Time as RosTime


class RadarPublisher(Node):
    def __init__(self):
        super().__init__('radar_pub')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # Maybe change to reliable later, best effort is speed focused
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

         # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('poll_frequency',   0.005),
                ('radar_cycle_time', 0.06),
                ('bitrate',          '500K'),
                ('device_type',      VCI_USBCAN1),
                ('visualise',        True),
                ('use_pointcloud',   False),
            ]
        )

        poll_freq      = self.get_parameter('poll_frequency').value
        cycle_time     = self.get_parameter('radar_cycle_time').value
        device_type    = self.get_parameter('device_type').value
        self.bitrate   = self.get_parameter('bitrate').value
        self.visualise = self.get_parameter('visualise').value
        self.use_cloud = self.get_parameter('use_pointcloud').value

        # publishers and helpers
        self.pub = self.create_publisher(Burst, 'mr76/burst', qos_profile)
        self.assembler = CycleAssembler(init_period=cycle_time)

        # Can adapter
        self.can = USBCAN(device_type=device_type)
        self.can.open()
        self._init_radar()

        # burst queue for thread hand-off
        self._queue: Queue[BurstBundle] = Queue()

        # Pooling and publishing threads
        self.timer = self.create_timer(poll_freq, self._poll_can)
        self._pub_thread = Thread(target=self._run_pub, daemon=True)
        self._pub_thread.start()
        self.get_logger().info(
            f"Polling every {poll_freq}s, radar_period≈{cycle_time}s, bitrate={self.bitrate}"
        )

    def _init_radar(self):
        self.can.reset_can()
        self.can.init_can(bitrate=self.bitrate)
        self.can.start_can()

    @staticmethod
    def _time_msg_to_float(t: RosTime) -> float:
        return t.sec + t.nanosec * 1e-9

    @staticmethod
    def _ros_time(epoch: float) -> RosTime:
        """Convert float-seconds since epoch to builtin_interfaces/Time."""
        sec  = int(epoch)
        nsec = int((epoch - sec) * 1e9)
        return RosTime(sec=sec, nanosec=nsec)

    def _poll_can(self):
        acq_ts = self.get_clock().now().to_msg()
        for fid, pl in self.can.receive(timeout_ms=0):
            msg = parse_frame(fid, pl)
            if not msg:
                continue
            self.assembler.push(msg, wall_ts=self._time_msg_to_float(acq_ts))

        # Get the data that has been stacked from the data frame
        for bundle in self.assembler.take_ready():
            self._queue.put(bundle)

    def _run_pub(self):
        while rclpy.ok():
            try:
                burst = self._queue.get(timeout=0.1)
                self._publish_burst(burst)

            except Empty:
                continue



    def _publish_burst(self, bundle: BurstBundle):
        if not bundle.objects:
            return
        # self.get_logger().info("\nNew information label")
        bmsg = Burst()
        bmsg.header.stamp = self._ros_time(bundle.start_ts)
        bmsg.cycle        = bundle.cycle
        bmsg.sensor       = bundle.sensor

        for o in bundle.objects:
            m = Object()
            m.id                = o.obj_id
            m.x_m               = o.x_m
            m.y_m               = o.y_m
            m.v_long_mps        = o.v_long_mps
            m.v_lat_mps         = o.v_lat_mps
            m.rcs_dbm2          = o.rcs_dbm2
            m.dyn_prop          = o.dyn_prop
            m.obj_class         = o.obj_class
            m.timestamp         = o.wall_ts
            m.warn_region_bits  = o.warn_region_bits or 0
            # self.get_logger().info(f"---x[m] {o.x_m}, y[m] {o.y_m}")
            bmsg.objects.append(m)

        self.pub.publish(bmsg)

def main():
    rclpy.init()
    node = RadarPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
