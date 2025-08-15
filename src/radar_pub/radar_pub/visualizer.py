#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from radar_types.msg import Burst

class BurstViz(Node):
    def __init__(self):
        super().__init__("burst_viz")
        self.pub = self.create_publisher(MarkerArray, "mr76/markers", 10)
        self.sub = self.create_subscription(Burst, "mr76/burst", self.cb, 10)

    def cb(self, msg: Burst):
        marr = MarkerArray()
        now  = self.get_clock().now().to_msg()
        for i, o in enumerate(msg.objects):
            m = Marker()
            m.header.stamp = msg.header.stamp
            m.header.frame_id = "map"     # or your radar frame
            m.ns = f"radar{msg.sensor}"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = o.x_m
            m.pose.position.y = o.y_m
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            # red if collision
            if o.warn_region_bits:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            marr.markers.append(m)
        self.pub.publish(marr)

def main():
    rclpy.init()
    rclpy.spin(BurstViz())
    rclpy.shutdown()
