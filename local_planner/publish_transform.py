#!/usr/bin/env python3
import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Path
from turtlesim.msg import Pose


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class VehicleTransformPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.publish_tf)

        self.global_path = self.create_subscription(
            Path,                     # Message type
            'global_path',               # Topic name
            self.path_cb,     # Callback function
            10                          # Queue size
        )
        self.idx = 0
        self.pose = None

        self.x, self.y = 0, 0

        self.vel = 1 * timer_period #1m/s
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.dt = 0.0

    def path_cb(self, msg):
        self.pose = msg.poses[self.idx].pose
        self.idx += 1
        

    def publish_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'vehicle'

        #t.transform.translation.x = self.x
        #t.transform.translation.y = self.y
        if self.pose:
            t.transform.translation.x = self.pose.position.x
            t.transform.translation.y = self.pose.position.y
        else:
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0

        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        self.x += math.cos(self.theta) * self.vel
        self.y += math.sin(self.theta) * self.vel
        self.theta += self.dt

def main(args=None):
    rclpy.init(args=args)
    vehicle_transform_pub = VehicleTransformPublisher()
    rclpy.spin(vehicle_transform_pub)

    vehicle_transform_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

