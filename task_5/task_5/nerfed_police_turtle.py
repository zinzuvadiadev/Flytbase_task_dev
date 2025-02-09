#!/usr/bin/env python3
"""
Police Turtle Slow Node (PT) for ROS 2

- The Robber Turtle (RT) moves in a circle and publishes its real pose on `/rt_real_pose`.
- PT spawns 10 seconds later at a random location.
- PT moves at half the speed of RT.
- The chase is complete when PT is ≤ 3.0 units from RT.

"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import random
import math
import time

class PoliceTurtleSlow(Node):
    def __init__(self):
        super().__init__('police_turtle_slow')
        self.get_logger().info("Police Turtle Slow node started.")
        
        # Wait 10 seconds before spawning PT
        self.get_logger().info("Waiting 10 seconds before spawning Police Turtle...")
        time.sleep(10)
        
        # Spawn PT at a random location
        self.spawn_police_turtle()
        
        # Publisher for PT's velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        
        # Subscribers for PT’s pose and RT’s pose
        self.create_subscription(Pose, '/turtle2/pose', self.pt_pose_callback, 10)
        self.create_subscription(Pose, '/rt_real_pose', self.rt_pose_callback, 10)
        
        self.pt_pose = None
        self.rt_pose = None
        
        # PID Control parameters
        self.Kp_linear = 1.0
        self.Kp_angular = 4.0
        self.max_angular_speed = 2.0
        
        # Smooth acceleration/deceleration limits
        self.max_accel_linear = 1.0
        self.max_decel_linear = 1.5
        self.max_accel_angular = 1.0
        self.max_decel_angular = 1.0
        
        self.current_vel = Twist()
        self.last_time = time.time()
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
    def spawn_police_turtle(self):
        """Spawns Police Turtle (PT) at a random location using the turtlesim spawn service."""
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for spawn service for Police Turtle...")
        
        req = Spawn.Request()
        req.x = random.uniform(1.0, 10.0)
        req.y = random.uniform(1.0, 10.0)
        req.theta = random.uniform(0, 2 * math.pi)
        req.name = "turtle2"
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"Police Turtle spawned at ({req.x:.2f}, {req.y:.2f})")
        else:
            self.get_logger().error("Failed to spawn Police Turtle")
    
    def pt_pose_callback(self, msg):
        """Updates PT’s current pose."""
        self.pt_pose = msg
        
    def rt_pose_callback(self, msg):
        """Updates RT’s (Robber Turtle’s) current pose from `/rt_real_pose`."""
        self.rt_pose = msg
        
    def update_velocity(self, current, desired, dt, max_accel, max_decel):
        """
        Gradually adjusts velocity while limiting acceleration and deceleration.
        """
        diff = desired - current
        max_step = max_accel * dt if diff > 0 else max_decel * dt
        return current + max(min(diff, max_step), -max_step)
    
    def step_vel(self, desired_vel):
        """Smoothly adjusts PT’s velocity toward the desired velocity."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        new_vel = Twist()
        new_vel.linear.x = self.update_velocity(
            self.current_vel.linear.x, desired_vel.linear.x, dt,
            self.max_accel_linear, self.max_decel_linear
        )
        new_vel.angular.z = self.update_velocity(
            self.current_vel.angular.z, desired_vel.angular.z, dt,
            self.max_accel_angular, self.max_decel_angular
        )
        
        self.current_vel = new_vel
        self.cmd_pub.publish(new_vel)
    
    def control_loop(self):
        """Computes the chase command using intercept prediction."""
        if self.pt_pose is None or self.rt_pose is None:
            return
        
        # PT's position
        x_p, y_p = self.pt_pose.x, self.pt_pose.y
        
        # RT's position and motion
        x_r, y_r = self.rt_pose.x, self.rt_pose.y
        v_r = self.rt_pose.linear_velocity
        theta_r = self.rt_pose.theta
        
        # PT moves at half the speed of RT
        pt_max_speed = 0.5 * v_r
        
        # Compute intercept time T
        A = v_r**2 - pt_max_speed**2
        B = 2 * ((x_r - x_p) * v_r * math.cos(theta_r) + (y_r - y_p) * v_r * math.sin(theta_r))
        C = (x_r - x_p)**2 + (y_r - y_p)**2
        
        discriminant = B**2 - 4 * A * C
        T = 1.0  # Default fallback time
        if A != 0 and discriminant >= 0:
            T1 = (-B + math.sqrt(discriminant)) / (2 * A)
            T2 = (-B - math.sqrt(discriminant)) / (2 * A)
            candidates = [t for t in (T1, T2) if t > 0]
            T = min(candidates) if candidates else 1.0
        
        # Predict intercept point
        x_int = x_r + v_r * math.cos(theta_r) * T
        y_int = y_r + v_r * math.sin(theta_r) * T
        
        # Check if PT has caught RT
        d_rt = math.sqrt((x_r - x_p)**2 + (y_r - y_p)**2)
        if d_rt <= 3.0:
            self.get_logger().info("Chase complete! Police Turtle caught the Robber Turtle.")
            self.cmd_pub.publish(Twist())  # Stop movement
            self.control_timer.cancel()
            return
        
        # Compute steering toward intercept point
        dx, dy = x_int - x_p, y_int - y_p
        desired_angle = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(desired_angle - self.pt_pose.theta), math.cos(desired_angle - self.pt_pose.theta))
        
        # Compute velocity using proportional control
        distance_to_intercept = math.sqrt(dx**2 + dy**2)
        desired_linear_vel = min(self.Kp_linear * distance_to_intercept, pt_max_speed)
        desired_angular_vel = max(min(self.Kp_angular * angle_error, self.max_angular_speed), -self.max_angular_speed)
        
        # Publish velocity command
        desired_twist = Twist()
        desired_twist.linear.x = desired_linear_vel
        desired_twist.angular.z = desired_angular_vel
        
        self.step_vel(desired_twist)
        self.get_logger().info(
            f"Intercept T: {T:.2f}, predicted intercept: ({x_int:.2f}, {y_int:.2f}), "
            f"PT->RT distance: {d_rt:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PoliceTurtleSlow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
