#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray  
from std_msgs.msg import Header
from radar_types.msg import Burst

class BurstViz(Node):
    def __init__(self):
        super().__init__("burst_viz")
        self.pub = self.create_publisher(MarkerArray, "mr76/markers", 10)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(Burst, "mr76/burst", self.cb, qos_profile)

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
            m.scale.x = m.scale.y = m.scale.z = 0.5
            # red if collision
            if o.warn_region_bits:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            marr.markers.append(m)

        # ------- CAR ------- #
        # Force delete previous car marker to avoid RViz caching issues
        delete_marker = Marker()
        delete_marker.header = Header()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = msg.header.stamp
        delete_marker.ns = "car"
        delete_marker.id = 10000
        delete_marker.action = Marker.DELETE
        marr.markers.append(delete_marker)

        car_marker = Marker()
        car_marker.header = Header()
        car_marker.header.frame_id = "map"
        car_marker.header.stamp = msg.header.stamp
        car_marker.ns = "car"
        car_marker.id = 10000
        car_marker.type = Marker.CUBE
        car_marker.action = Marker.ADD
        car_marker.pose.position.x = 0.0
        car_marker.pose.position.y = 0.0
        car_marker.pose.position.z = 0.1  # to raise it slightly off ground
        car_marker.pose.orientation.w = 1.0
        car_marker.scale.x = 2.0  # Length
        car_marker.scale.y = 1.0   # Width
        car_marker.scale.z = 0.2   # Height
        car_marker.color.a = 0.8
        car_marker.color.r = 0.0
        car_marker.color.g = 0.0
        car_marker.color.b = 1.0
        car_marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        marr.markers.append(car_marker)


        # ------- DOT ------- #

        front_dot = Marker()
        front_dot.header = Header()
        front_dot.header.frame_id = "map"
        front_dot.header.stamp = msg.header.stamp
        front_dot.ns = "car"
        front_dot.id = 20002
        front_dot.type = Marker.SPHERE
        front_dot.action = Marker.ADD
        front_dot.pose.position.x = 1.0
        front_dot.pose.position.y = 0.0
        front_dot.pose.position.z = 0.0
        front_dot.pose.orientation.w = 1.0
        front_dot.scale.x = front_dot.scale.y = front_dot.scale.z = 0.2
        front_dot.color.a = 1.0
        front_dot.color.r = 1.0
        front_dot.color.g = 0.4
        front_dot.color.b = 0.7
        marr.markers.append(front_dot)


        self.pub.publish(marr)

def main():
    rclpy.init()
    node = BurstViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()