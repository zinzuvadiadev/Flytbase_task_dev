#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose
from turtlesim.srv import Spawn
import time

class ChaseTurtle(Node):
    def __init__(self):
        super().__init__('chase_turtle_noisy')

        # Subscribers
        self.rt_noisy_pose_sub = self.create_subscription(
            TurtlePose, '/rt_noisy_pose', self.rt_noisy_pose_callback, 10)

        self.pt_pose_sub = self.create_subscription(
            TurtlePose, '/turtle2/pose', self.pt_pose_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Timer for control loop (every 5 sec)
        self.timer = self.create_timer(5.0, self.control_loop)

        # Variables
        self.rt_noisy_pose = None
        self.pt_pose = None
        self.noisy_positions = []  # Store last few RT positions
        self.window_size = 3  # Number of noisy points to average

        # Spawn chasing turtle (PT)
        self.spawn_turtle(5.0, 5.0, 0.0)

    def spawn_turtle(self, x, y, theta):
        """Spawns a new turtle at (x, y)."""
        client = self.create_client(Spawn, '/spawn')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for spawn service...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = "turtle2"

        future = client.call_async(request)
        
    def rt_noisy_pose_callback(self, msg):
        """Callback for RT's noisy pose(every 5s)."""
        self.rt_noisy_pose = np.array([msg.x,msg.y,msg.theta])
        self.noisy_positions.append(self.rt_noisy_pose)

        # Keep only the last few noisy points
        if len(self.noisy_positions) > self.window_size:
            self.noisy_positions.pop(0)

    def pt_pose_callback(self, msg):
        """Callback for PT's current pose."""
        self.pt_pose = np.array([msg.x, msg.y, msg.theta])

    def estimate_future_position(self):
        """Estimate future position using a simple moving average."""
        if len(self.noisy_positions) < 2:
            return None  # Not enough data to estimate

        avg_position = np.mean(self.noisy_positions, axis=0)
        return avg_position

    def control_loop(self):
        """Control loop to move PT towards estimated RT position."""
        if self.pt_pose is None or self.rt_noisy_pose is None:
            self.get_logger().warn("Waiting for pose data...")
            return

        estimated_target = self.estimate_future_position()
        if estimated_target is None:
            self.get_logger().warn("Not enough data to estimate target!")
            return

        target_x, target_y,_ = estimated_target
        pt_x, pt_y, pt_theta = self.pt_pose

        dx = target_x - pt_x
        dy = target_y - pt_y
        distance = np.sqrt(dx**2 + dy**2)

        twist = Twist()
        angle_to_target = np.arctan2(dy, dx)
        angle_error = angle_to_target - pt_theta
        twist.angular.z = 2.0 * angle_error  # Rotation speed

        if abs(angle_error) < 0.1:  # Small angle threshold
            twist.linear.x = max(0.5, min(2.0, distance))  

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Moving to: {target_x:.2f}, {target_y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    chase_turtle = ChaseTurtle()
    rclpy.spin(chase_turtle)
    chase_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()