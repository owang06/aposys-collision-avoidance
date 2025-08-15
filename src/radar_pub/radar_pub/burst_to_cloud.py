#!/usr/bin/env python3
import rclpy, struct
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from radar_types.msg import Burst

FIELDS = [
    PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]

class BurstCloud(Node):
    def __init__(self):
        super().__init__("burst_cloud")
        self.pub = self.create_publisher(PointCloud2, "mr76/cloud", 10)
        self.sub = self.create_subscription(Burst, "mr76/burst", self.cb, 10)

    def cb(self, msg: Burst):
        buf = bytearray()
        for o in msg.objects:
            buf += struct.pack("<ffff", o.x_m, o.y_m, 0.0, o.rcs_dbm2)
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.header.frame_id = "map"
        cloud.height = 1
        cloud.width  = len(msg.objects)
        cloud.is_dense = False
        cloud.point_step = 16
        cloud.row_step   = 16 * cloud.width
        cloud.fields = FIELDS
        cloud.data = bytes(buf)
        self.pub.publish(cloud)

def main():
    rclpy.init()
    rclpy.spin(BurstCloud())
    rclpy.shutdown()
