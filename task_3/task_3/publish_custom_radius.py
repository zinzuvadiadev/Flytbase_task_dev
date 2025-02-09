#!/usr/bin/env python3
"""
This node is publishing a custom circle radius to the topic `/custom_circle_radius`
Use this for custom radius, need this goal 5 and goal 6

"""

import rclpy
import numpy

from rclpy.node import Node
from std_msgs.msg import Float32

class CustomRadiusPublisher(Node):
    def __init__(self):
        super().__init__('custom_radius_publisher')
        self.publisher_ = self.create_publisher(Float32, '/custom_circle_radius', 10)
        self.get_logger().info("Custom Radius Publisher started. Enter new radius values.")

    def publish_radius(self, radius):
        msg = Float32()
        msg.data = radius
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published new radius: {radius}")

def main(args=None):
    rclpy.init(args=args)
    node = CustomRadiusPublisher()
    try:
        while rclpy.ok():
            user_input = input("Enter new circle radius")
            if user_input.lower() == 'exit':
                break
            try:
                radius = float(user_input)
                node.publish_radius(radius)
            except ValueError:
                node.get_logger().error("Invalid input.")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
