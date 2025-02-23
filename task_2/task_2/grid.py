#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import Pose
from math import sqrt, atan2, pi
import time

class TurtleGridController(Node):
    def __init__(self):
        super().__init__('turtle_grid_controller')
        self.get_logger().info("Initializing TurtleGridController Node...")

        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = None

        # Gains for straight-line driving and rotation
        self.Kp_linear = 1.0    # Lower gain for smoother acceleration
        self.Kp_angular = 4.0   # Angular gain for rotation

        # Maximum speed limits (to avoid hitting walls)
        self.max_linear_speed = 1.0    # Reduced maximum linear speed
        self.max_angular_speed = 1.0   # Reduced maximum angular speed

        # Acceleration limits (m/s² for linear, rad/s² for angular)
        self.max_accel_linear = 1.8        # Acceleration remains the same
        self.max_decel_linear = 2.0      # Increase deceleration for faster stopping
        self.max_accel_angular = 1.2
        self.max_decel_angular = 1.2

        # For gradual velocity updates
        self.current_vel = Twist()
        self.last_time = time.time()

        # Grid corners:(x, y, desired_heading_in_degrees)
        self.grid_corners = [
            (1, 1, 0),
            (10, 1, 90),
            (10,2,180),
            (1,2,90),
            (1, 3, 0),
            (10, 3, 180),
            (10,4, 90),
            (1,4,180),
            (1, 5, 90),
            (10, 5, 0),
            (10, 6, 90),
            (1, 6, 180),
            (1, 7, 90),
            (10, 7, 0),
            (10, 8, 90),
            (1, 8, 180),
            (1, 9, 90),
            (10, 9, 0)
        ]

    def pose_callback(self, msg):
        self.pose = msg
        self.get_logger().info(f"Received Pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")

    def normalize_angle(self, angle):
        """Normalize angle to the range [-pi, pi]."""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def update_velocity(self, current, desired, dt, max_accel, max_decel):
        """
        Gradually adjust a velocity component toward the desired value while limiting acceleration and deceleration.
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
        Update the current velocity toward the desired velocity using a fixed time step for consistency.
        """
        dt = 0.05  # fixed time step (20 Hz)
        self.last_time = time.time()
        new_vel = Twist()
        new_vel.linear.x = self.update_velocity(
            self.current_vel.linear.x, desired_vel.linear.x, dt,
            self.max_accel_linear, self.max_decel_linear)
        new_vel.angular.z = self.update_velocity(
            self.current_vel.angular.z, desired_vel.angular.z, dt,
            self.max_accel_angular, self.max_decel_angular)
        self.current_vel = new_vel
        self.vel_pub.publish(new_vel)
        self.get_logger().info(f"Publishing Velocity: Linear={new_vel.linear.x:.2f}, Angular={new_vel.angular.z:.2f}")


    def rotate_to_angle(self, target_deg):
        """
        Rotate the turtle using proportional control.
        """
        target_rad = target_deg * pi / 180.0

        # Wait for a valid pose
        while rclpy.ok() and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        while rclpy.ok():
            angular_error = self.normalize_angle(target_rad - self.pose.theta)
            if abs(angular_error) < 0.1:
                break
            twist = Twist()
            cmd_ang = self.Kp_angular * angular_error
            if abs(cmd_ang) > self.max_angular_speed:
                cmd_ang = self.max_angular_speed if cmd_ang > 0 else -self.max_angular_speed
            twist.angular.z = cmd_ang
            self.vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        # Stop rotation
        stop = Twist()
        self.vel_pub.publish(stop)
        time.sleep(0.5)

    def drive_straight(self, goal_x, goal_y):
        while rclpy.ok() and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        desired_angle = atan2(goal_y - self.pose.y, goal_x - self.pose.x)
        self.rotate_to_angle(desired_angle * 180.0 / pi)

        start_time = time.time()
        while rclpy.ok():
            distance_error = sqrt((goal_x - self.pose.x)**2 + (goal_y - self.pose.y)**2)

            # DEBUG LOG
            self.get_logger().info(f"Moving... Distance Error: {distance_error:.2f}")

            if distance_error < 0.1:  
                self.get_logger().info(f"Reached ({goal_x}, {goal_y})")
                break  # Exit loop properly

            twist = Twist()
            desired_speed = min(self.Kp_linear * distance_error, self.max_linear_speed)
            if distance_error < 0.2:  
                desired_speed *= 0.5  

            twist.linear.x = desired_speed
            self.step_vel(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        stop = Twist()
        self.vel_pub.publish(stop)
        time.sleep(0.5)

    def grid(self):
        for i, (goal_x, goal_y, angle_deg) in enumerate(self.grid_corners):
            self.get_logger().info(f"Moving to {goal_x}, {goal_y} (Index: {i})")

            self.drive_straight(goal_x, goal_y)
            
            self.get_logger().info(f"Rotating to {angle_deg}°")
            self.rotate_to_angle(angle_deg)

        self.get_logger().info("Grid completed")


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleGridController()

    # Wait for the first pose message before starting
    while rclpy.ok() and controller.pose is None:
        rclpy.spin_once(controller, timeout_sec=0.1)
    time.sleep(1.0)
    controller.grid()
    controller.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
