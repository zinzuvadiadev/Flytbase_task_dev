#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, atan2, pi
import time

class TurtleGridController(Node):
    def __init__(self):
        super().__init__('turtle_grid_controller')
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
        self.max_accel_linear = 1.2      # Acceleration remains the same
        self.max_decel_linear = 2.0      # Increase deceleration for faster stopping
        self.max_accel_angular = 1.2
        self.max_decel_angular = 1.2

        # For gradual velocity updates
        self.current_vel = Twist()
        self.last_time = time.time()

        # Grid corners: each tuple is (x, y, desired_heading_in_degrees)
        self.grid_corners = [
            (1, 1, 0),
            (10, 1, 90),
            (10,2,180),
            (1,2,90),
            (1, 3, 0),
            (10, 3, 180),
            (10,4,0),
            (1,4,90),
            (1, 5, 0),
            (10, 5, 90),
            (10, 6, 180),
            (1, 6, 90),
            (1, 7, 0),
            (10, 7, 180),
            (10, 8, 0),
            (1, 8, 90),
            (1, 9, 0),
            (10, 9, 0)
        ]

    def pose_callback(self, msg):
        self.pose = msg

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
            if abs(angular_error) < 0.02:
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
        """
        Drives the turtle in a straight line toward the goal.
        First, turtle rotates to face target point.
        After this turtle navigates to the point.
        """
        while rclpy.ok() and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Calculate the desired heading
        desired_angle = atan2(goal_y - self.pose.y, goal_x - self.pose.x)
        self.get_logger().info(f"Aligning to {desired_angle * 180 / pi:.2f}° for goal ({goal_x}, {goal_y})")
        self.rotate_to_angle(desired_angle * 180.0 / pi)

        start_time = time.time()
        while rclpy.ok():
            distance_error = sqrt((goal_x - self.pose.x)**2 + (goal_y - self.pose.y)**2)
            if distance_error < 0.05:
                break

            twist = Twist()
            # Proportional control for linear speed
            desired_speed = self.Kp_linear * distance_error
            if desired_speed > self.max_linear_speed:
                desired_speed = self.max_linear_speed
            twist.linear.x = desired_speed
            twist.angular.z = 0.0

            self.step_vel(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        # Stop the turtle once the goal is reached
        stop = Twist()
        self.vel_pub.publish(stop)
        time.sleep(0.5)
        total_time = time.time() - start_time
        self.get_logger().info(f"Reached goal ({goal_x}, {goal_y}) in {total_time:.2f} seconds")

    def grid(self):
        """
        Follow the grid corners and then rotating to the desired point.
        """
        for corner in self.grid_corners:
            goal_x, goal_y, angle_deg = corner
            self.drive_straight(goal_x, goal_y)
            self.get_logger().info(f"Rotating to {angle_deg}°")
            self.rotate_to_angle(angle_deg)
        self.get_logger().info("Grid created")
        rclpy.shutdown()

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
