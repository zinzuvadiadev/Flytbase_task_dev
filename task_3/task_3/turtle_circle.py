#!/usr/bin/env python3
"""
This node drives the turtlesim turtle in a circle.
It subscribes to `/turtle1/cmd_vel` to update velocity dynamically.
It publishes real pose on `/rt_real_pose` and a noisy pose on `/rt_noisy_pose` every 5 seconds.
It also subscribes to `/custom_circle_radius` to update the circle radius.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import Pose
from std_msgs.msg import Float32
import random


class CircleTurtleController(Node):
    def __init__(self):
        super().__init__('circle_turtle_controller')
        
        # Declare parameters
        self.declare_parameter('circle_radius', 5.0)
        self.declare_parameter('linear_speed', 1.0)
        self.circle_radius = self.get_parameter('circle_radius').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.linear_speed / self.circle_radius

        # Noise standard deviation for noisy pose
        self.noise_std = 10.0

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.real_pose_pub = self.create_publisher(Pose, '/rt_real_pose', 10)
        self.noisy_pose_pub = self.create_publisher(Pose, '/rt_noisy_pose', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.radius_sub = self.create_subscription(Float32, '/custom_circle_radius', self.radius_callback, 10)
        self.vel_sub = self.create_subscription(Twist, '/turtle1/cmd_vel', self.vel_callback, 10)
        
        self.current_pose = None
        self.last_received_twist = None  # Stores the last received velocity command

        # Timers
        self.cmd_timer = self.create_timer(0.05, self.cmd_vel_callback)    
        self.pose_timer = self.create_timer(5.0, self.publish_pose_callback) 
        
        self.get_logger().info(
            f"Circle Turtle started with: radius = {self.circle_radius}, linear_speed = {self.linear_speed}, "
            f"angular_speed = {self.angular_speed:.2f}"
        )

    def cmd_vel_callback(self):
        """Publish velocity commands dynamically for circular motion."""
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist)


    def vel_callback(self, msg):
        # Store initial radius only once
        if self.circle_radius is None:
            self.circle_radius = self.linear_speed / self.angular_speed  # Initial radius

        # Update linear speed
        self.linear_speed = msg.linear.x

        self.get_logger().info(f"Updated velocity: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}")


    def pose_callback(self, msg):
        """Updates the current pose of the turtle."""
        self.current_pose = msg

    def radius_callback(self, msg):
        """Updates the circle radius and adjusts angular speed accordingly."""
        new_radius = msg.data
        if new_radius <= 0:
            self.get_logger().warn("Received negative radius. Ignoring update.")
            return
        self.circle_radius = new_radius
        self.angular_speed = self.linear_speed / self.circle_radius
        self.get_logger().info(f"New radius: {self.circle_radius:.2f}, New angular speed: {self.angular_speed:.2f}")

    def publish_pose_callback(self):
        """Publishes the real and noisy pose of the turtle."""
        if self.current_pose is None:
            return

        # Publish real pose
        self.real_pose_pub.publish(self.current_pose)

        # Create and publish noisy pose
        noisy_pose = Pose()
        noisy_pose.x = self.current_pose.x + random.gauss(0, self.noise_std)
        noisy_pose.y = self.current_pose.y + random.gauss(0, self.noise_std)
        noisy_pose.theta = self.current_pose.theta + random.gauss(0, self.noise_std)
        noisy_pose.linear_velocity = self.current_pose.linear_velocity
        noisy_pose.angular_velocity = self.current_pose.angular_velocity

        self.noisy_pose_pub.publish(noisy_pose)

        self.get_logger().info(
            f"Real Pose: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}, {self.current_pose.theta:.2f}) | "
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
