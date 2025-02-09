#!/usr/bin/env python3
"""
This node drives the turtlesim turtle in a circle.
The node publishes velocity commands on `/turtle1/cmd_vel`, publishes the real pose on `/rt_real_pose`
and a noisy version on `/rt_noisy_pose` every 5 seconds, 
and subscribes to `/custom_circle_radius` To update the circle radius.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from math import atan2, pi
import random


class CircleTurtleController(Node):
    def __init__(self):
        super().__init__('circle_turtle_controller')
        
        # Declare and get parameters for circle motion.
        self.declare_parameter('circle_radius', 2.0)
        self.declare_parameter('linear_speed', 1.0)
        self.circle_radius = self.get_parameter('circle_radius').value
        self.linear_speed = self.get_parameter('linear_speed').value
        
        # Compute angular speed for circular motion.
        self.angular_speed = self.linear_speed / self.circle_radius
        
        # Noise standard deviation (10 units) for noisy pose.
        self.noise_std = 10.0

        # Publishers:
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.real_pose_pub = self.create_publisher(Pose, '/rt_real_pose', 10)
        self.noisy_pose_pub = self.create_publisher(Pose, '/rt_noisy_pose', 10)

        # Subscribers:
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.radius_sub = self.create_subscription(Float32, '/custom_circle_radius', self.radius_callback, 10)
        self.current_pose = None

        self.cmd_timer = self.create_timer(0.05, self.cmd_vel_callback)    
        self.pose_timer = self.create_timer(5.0, self.publish_pose_callback) 
        
        self.get_logger().info(
            f"Circle Turtle started with: radius = {self.circle_radius}, linear_speed = {self.linear_speed}, "
            f"angular_speed = {self.angular_speed:.2f}"
        )

    def cmd_vel_callback(self):
        """Continuously publish Twist commands for circular motion."""
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist)

    def pose_callback(self, msg):
        """Update the current turtle pose."""
        self.current_pose = msg

    def radius_callback(self, msg):
        """Update the circle radius."""
        new_radius = msg.data
        if new_radius <= 0:
            self.get_logger().warn("Received -ve radius. Change it to +ve.")
            return
        self.circle_radius = new_radius
        self.angular_speed = self.linear_speed / self.circle_radius
        self.get_logger().info(f"New radius: {self.circle_radius:.2f},New angular speed: {self.angular_speed:.2f}")

    def publish_pose_callback(self):
        """
        To publish the real turtle pose on `/rt_real_pose` and a noisy version
        on `/rt_noisy_pose` (with Gaussian noise added to x, y, and theta).
        """
        if self.current_pose is None:
            return

        # Real pose.
        real_pose = self.current_pose
        self.real_pose_pub.publish(real_pose)

        # Gausian Noise.
        noisy_pose = Pose()
        noisy_pose.x = real_pose.x + random.gauss(0, self.noise_std)
        noisy_pose.y = real_pose.y + random.gauss(0, self.noise_std)
        noisy_pose.theta = real_pose.theta + random.gauss(0, self.noise_std)
        noisy_pose.linear_velocity = real_pose.linear_velocity
        noisy_pose.angular_velocity = real_pose.angular_velocity

        self.noisy_pose_pub.publish(noisy_pose)

        self.get_logger().info(
            f"Real Pose: ({real_pose.x:.2f}, {real_pose.y:.2f}, {real_pose.theta:.2f}) | "
            f"Noisy Pose: ({noisy_pose.x:.2f}, {noisy_pose.y:.2f}, {noisy_pose.theta:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CircleTurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
