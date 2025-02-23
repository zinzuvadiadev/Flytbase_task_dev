#!/usr/bin/env python3
"""
This script controls a Police Turtle (PT) to catch a Robber Turtle (RT) in Turtlesim.
The PT must deduce RT's motion, predict its future position, and intercept it while
only receiving RT's position updates once every 5 seconds.
"""

import rclpy
from rclpy.node import Node
from turtlesim_msgs.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_msgs.srv import Spawn
import random
import math
import time

class PoliceTurtle(Node):
    def __init__(self):
        super().__init__('police_turtle')
        self.get_logger().info("PT node started.")

        time.sleep(10)  # Wait before spawning PT
        self.spawn_police_turtle()

        self.cmd_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pt_pose_callback, 10)
        self.create_subscription(Pose, '/rt_real_pose', self.rt_pose_callback, 10)

        self.pt_pose = None
        self.rt_pose_history = []  # Store RT's past positions
        self.circle_params = None  # (xc, yc, radius, angular velocity)
        self.time_step = 5.0
        self.last_rt_update_time = time.time()

        self.Kp_linear = 2.0
        self.Kp_angular = 4.0
        self.max_linear_speed = 1.0
        self.max_angular_speed = 2.0

        self.control_timer = self.create_timer(0.05, self.control_loop)

    def spawn_police_turtle(self):
        client = self.create_client(Spawn, 'spawn')
        req = Spawn.Request()
        req.x = random.uniform(1.0, 10.0)
        req.y = random.uniform(1.0, 10.0)
        req.theta = random.uniform(0, 2 * math.pi)
        req.name = "turtle2"
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            self.get_logger().info(f"PT spawned at ({req.x:.2f}, {req.y:.2f})")
        else:
            self.get_logger().error("Failed to spawn PT")

    def pt_pose_callback(self, msg):
        self.pt_pose = msg

    def rt_pose_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_rt_update_time >= self.time_step:
            self.rt_pose_history.append((msg.x, msg.y, current_time))
            self.get_logger().info(f"RT update received: ({msg.x:.2f}, {msg.y:.2f})")
            if len(self.rt_pose_history) > 3:
                self.rt_pose_history.pop(0)
            if len(self.rt_pose_history) == 3:
                self.estimate_circle_params()
            self.last_rt_update_time = current_time

    def estimate_circle_params(self):
        """Estimate RT's circular motion parameters and validate if RT is actually moving in a circle."""
        if len(self.rt_pose_history) < 3:
            return  # Not enough data

        (x1, y1, _), (x2, y2, _), (x3, y3, _) = self.rt_pose_history
        
        # Check if RT is stationary
        if x1 == x2 == x3 and y1 == y2 == y3:
            self.get_logger().info("RT is stationary.")
            self.circle_params = None
            return
        
        # Compute circle center and radius using perpendicular bisectors
        A = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)
        B = (x1**2 + y1**2) * (y3 - y2) + (x2**2 + y2**2) * (y1 - y3) + (x3**2 + y3**2) * (y2 - y1)
        C = (x1**2 + y1**2) * (x2 - x3) + (x2**2 + y2**2) * (x3 - x1) + (x3**2 + y3**2) * (x1 - x2)
        D = (x1**2 + y1**2) * (x3 * y2 - x2 * y3) + (x2**2 + y2**2) * (x1 * y3 - x3 * y1) + (x3**2 + y3**2) * (x2 * y1 - x1 * y2)
        
        if A == 0:
            self.get_logger().warn("RT is not moving in a circle.")
            self.circle_params = None  # Do not assume circular motion
            return  
        
        xc = -B / (2 * A)
        yc = -C / (2 * A)
        radius = math.sqrt((x1 - xc)**2 + (y1 - yc)**2)

        # Validate the circle parameters
        if radius < 1.0 or radius > 10.0:
            self.get_logger().warn("Invalid circle.")
            self.circle_params = None  # Reset circular motion assumption
            return
        
        # Estimate angular velocity
        dt1 = self.rt_pose_history[1][2] - self.rt_pose_history[0][2]
        dt2 = self.rt_pose_history[2][2] - self.rt_pose_history[1][2]
        
        if dt1 == 0 or dt2 == 0:
            return  # Avoid division by zero
        
        angular_velocity = 2 * math.pi / (radius * 10)  # Basic estimation

        self.circle_params = (xc, yc, radius, angular_velocity)

    def predict_rt_position(self, future_time):
        """Predict RT's future position based on circular or linear motion."""
        if not self.rt_pose_history:
            return None  # No data available

        # Use Circular Motion Model if parameters are valid
        if self.circle_params:
            xc, yc, radius, angular_velocity = self.circle_params
            x_last, y_last, t_last = self.rt_pose_history[-1]
            angle_last = math.atan2(y_last - yc, x_last - xc)
            time_diff = future_time - t_last

            predicted_angle = angle_last + angular_velocity * time_diff
            predicted_x = xc + radius * math.cos(predicted_angle)
            predicted_y = yc + radius * math.sin(predicted_angle)
            return predicted_x, predicted_y
        
        # Otherwise, use Linear Motion Model
        if len(self.rt_pose_history) < 2:
            return None  # Not enough data for linear prediction

        (x1, y1, t1), (x2, y2, t2) = self.rt_pose_history[-2:]

        if t2 - t1 == 0:
            return x2, y2  # Avoid division by zero

        velocity_x = (x2 - x1) / (t2 - t1)
        velocity_y = (y2 - y1) / (t2 - t1)

        time_diff = future_time - t2
        predicted_x = x2 + velocity_x * time_diff
        predicted_y = y2 + velocity_y * time_diff

        return predicted_x, predicted_y


    def control_loop(self):
        if self.pt_pose is None or self.circle_params is None:
            return
        
        future_rt_position = self.predict_rt_position(time.time() + self.time_step)
        if not future_rt_position:
            return
        
        dx = future_rt_position[0] - self.pt_pose.x
        dy = future_rt_position[1] - self.pt_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance <= 1.0:
            self.get_logger().info("Chase successful. PT has caught RT.")
            self.cmd_pub.publish(Twist())
            self.control_timer.cancel()
            return
        
        desired_angle = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(desired_angle - self.pt_pose.theta),
                                 math.cos(desired_angle - self.pt_pose.theta))
        
        twist = Twist()
        twist.linear.x = self.max_linear_speed
        twist.angular.z = max(min(self.Kp_angular * angle_error, self.max_angular_speed), -self.max_angular_speed)
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    police_node = PoliceTurtle()
    rclpy.spin(police_node)
    police_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    