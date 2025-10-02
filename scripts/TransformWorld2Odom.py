#!/usr/bin/env python3
import math
import sys

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static odom frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):
        super().__init__('world2odom_tf2_broadcaster')

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        #timer_period = 1/self.frequency
        #self.timer = self.create_timer(timer_period, self.make_transforms)
        self.make_transforms()

    def make_transforms(self):
        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = 'world'
        t_base.child_frame_id = 'odom'
        t_base.transform.translation.x = 0.0
        t_base.transform.translation.y = 0.0
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.x = 0.0
        t_base.transform.rotation.y = 0.0
        t_base.transform.rotation.z = 0.0
        t_base.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_base)
        print('publishing transformation')


def main():
    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()