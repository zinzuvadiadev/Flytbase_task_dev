#!/usr/bin/env python3
"""
Police Turtle Node (PT) for ROS 2

This spawns a Police Turtle (PT) that tries to catch Robber Turtle (RT).
RT should already be running (using the node from Goal 3) and moving in a circle,
publishing real pose on the `/rt_real_pose` topic on every 5 sec.

PT spawns 10 seconds after RT is launched, from a random location, and then
chases RT. PT moves at a higher speed than RT, and when the distance between PT
and RT is ≤ 3 units, the chase is considered complete.
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import random
import math
import time


class PoliceTurtle(Node):
    def __init__(self):
        super().__init__('police_turtle')
        self.get_logger().info("Police Turtle node started.")

        # Wait 10 seconds before spawning PT (to let RT launch first)
        self.get_logger().info("Waiting 10 seconds before spawning Police Turtle...")
        time.sleep(10)

        # Spawn Police Turtle (PT) as "turtle2" at a random location
        self.spawn_police_turtle()

        # Publisher for PT's velocity (PT will be controlled via /turtle2/cmd_vel)
        self.cmd_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        # Subscribe to PT's own pose
        self.create_subscription(Pose, '/turtle2/pose', self.pt_pose_callback, 10)
        # Subscribe to RT's real pose published on /rt_real_pose
        self.create_subscription(Pose, '/rt_real_pose', self.rt_pose_callback, 10)

        # Storage for the current poses
        self.pt_pose = None
        self.rt_pose = None

        # Control parameters (PT moves faster than RT)
        self.Kp_linear = 2.0    # Proportional gain for linear velocity
        self.Kp_angular = 4.0   # Proportional gain for angular velocity
        self.max_linear_speed = 3.0   # Maximum linear speed for PT
        self.max_angular_speed = 2.0  # Maximum angular speed for PT

        # Acceleration/deceleration limits for smoother motion
        self.max_accel_linear = 1.5
        self.max_decel_linear = 2.0
        self.max_accel_angular = 1.5
        self.max_decel_angular = 1.5

        # For gradual velocity updates
        self.current_vel = Twist()
        self.last_time = time.time()

        # Timer for the control loop (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

    def spawn_police_turtle(self):
        """Spawn the Police Turtle (PT) using the turtlesim spawn service."""
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for spawn service for Police Turtle...")
        req = Spawn.Request()
        # Random spawn coordinates (within the typical turtlesim window)
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
        """Callback to update PT's own pose."""
        self.pt_pose = msg

    def rt_pose_callback(self, msg):
        """Callback to update RT's real pose (from /rt_real_pose)."""
        self.rt_pose = msg

    def update_velocity(self, current, desired, dt, max_accel, max_decel):
        """
        Gradually adjust a velocity component toward the desired value while limiting acceleration.
        """
        diff = desired - current
        if diff > 0:
            max_step = max_accel * dt
            if diff > max_step:
                diff = max_step
        else:
            max_step = max_decel * dt
            if diff < -max_step:
                diff = -max_step
        return current + diff

    def step_vel(self, desired_vel):
        """
        Update the current velocity gradually toward the desired velocity command.
        """
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
        """
        Control loop that computes the chase command:
          - Uses the latest RT pose (from /rt_real_pose) and PT's pose.
          - If the distance between PT and RT is ≤ 3 units, stops the chase.
          - Otherwise, computes desired linear and angular velocities and publishes them.
        """
        if self.pt_pose is None or self.rt_pose is None:
            return

        # Compute the distance between PT and RT
        dx = self.rt_pose.x - self.pt_pose.x
        dy = self.rt_pose.y - self.pt_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        self.get_logger().info(f"Distance to RT: {distance:.2f}")

        if distance <= 3.0:
            self.get_logger().info("Chase complete! Police Turtle has caught the Robber Turtle.")
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.control_timer.cancel()  # Stop the control loop
            return

        # Compute the desired heading from PT to RT
        desired_angle = math.atan2(dy, dx)
        # Compute the angular error (normalize to [-pi, pi])
        angle_error = desired_angle - self.pt_pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Compute desired velocities using proportional control
        desired_linear_vel = self.Kp_linear * distance
        desired_angular_vel = self.Kp_angular * angle_error

        # Cap the desired speeds
        desired_linear_vel = min(desired_linear_vel, self.max_linear_speed)
        desired_angular_vel = max(min(desired_angular_vel, self.max_angular_speed), -self.max_angular_speed)

        desired_twist = Twist()
        desired_twist.linear.x = desired_linear_vel
        desired_twist.angular.z = desired_angular_vel

        self.step_vel(desired_twist)

def main(args=None):
    rclpy.init(args=args)
    police_node = PoliceTurtle()
    rclpy.spin(police_node)
    police_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
