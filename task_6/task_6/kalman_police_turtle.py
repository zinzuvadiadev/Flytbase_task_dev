import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import Pose
from std_srvs.srv import Empty
from math import atan2, sqrt, sin, cos, radians, degrees
import numpy as np
from rclpy.qos import QoSProfile
from turtlesim_msgs.srv import Spawn, Kill
import random
import math

class TurtleChase(Node):
    def __init__(self):
        super().__init__('turtle_chase')
        
        self.declare_parameters(namespace='', parameters=[('turtle_name', 'turtle2')])
        self.turtle_name = self.get_parameter('turtle_name').get_parameter_value().string_value
        
        self.pose = Pose()
        self.target_pose = Pose()
        self.vel_pub = self.create_publisher(Twist, f'/{self.turtle_name}/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, f'/{self.turtle_name}/pose', self.pose_callback, 10)
        self.target_sub = self.create_subscription(Pose, '/rt_noisy_pose', self.target_callback, 10)
        
        self.spawn_police_turtle()
        
        self.timer = self.create_timer(0.1, self.chase_target)
        self.a = []  # List to store target positions
        
        # Kalman filter initialization
        self.kalman_x = self.init_kalman()
        self.kalman_y = self.init_kalman()
    
    def init_kalman(self):
        return {
            "x": 0,
            "P": 1,
            "Q": 0.001,
            "R": 0.1
        }
    
    def kalman_filter(self, state, measurement):
        state["x"] = state["x"]
        state["P"] = state["P"] + state["Q"]
        K = state["P"] / (state["P"] + state["R"])
        state["x"] = state["x"] + K * (measurement - state["x"])
        state["P"] = (1 - K) * state["P"]
        return state["x"]
    
    def spawn_police_turtle(self):
        kill_client = self.create_client(Kill, 'kill')
        if kill_client.wait_for_service(timeout_sec=2.0):
            kill_req = Kill.Request()
            kill_req.name = "turtle2"
            future = kill_client.call_async(kill_req)
            rclpy.spin_until_future_complete(self, future)
        
        client = self.create_client(Spawn, 'spawn')
        if client.wait_for_service(timeout_sec=2.0):
            req = Spawn.Request()
            req.x = random.uniform(1.0, 10.0)
            req.y = random.uniform(1.0, 10.0)
            req.theta = random.uniform(0, 2 * math.pi)
            req.name = "turtle2"
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result():
                self.get_logger().info("Successfully spawned turtle2 at ({}, {}, {}).".format(req.x, req.y, req.theta))
            else:
                self.get_logger().error("Failed to spawn turtle2.")
    
    def pose_callback(self, msg):
        self.pose = msg
    
    def target_callback(self, msg):
        filtered_x = self.kalman_filter(self.kalman_x, msg.x)
        filtered_y = self.kalman_filter(self.kalman_y, msg.y)
        
        self.target_pose.x = filtered_x
        self.target_pose.y = filtered_y
        
        if len(self.a) < 3:
            self.a.append([filtered_x, filtered_y])
    
    def chase_target(self):
        if len(self.a) < 3:
            return
        
        distance_to_target = sqrt((self.pose.x - self.target_pose.x) ** 2 + (self.pose.y - self.target_pose.y) ** 2)
        
        if distance_to_target <= 3.0:
            self.get_logger().info("Chase complete! Police Turtle caught the Robber Turtle.")
            self.vel_pub.publish(Twist())  # Stop movement
            return
        
        center, radius = self.define_circle(self.a[0], self.a[1], self.a[2])
        
        if center is None or radius == np.inf:
            self.get_logger().error("Circle fitting failed! Points might be collinear.")
            return
        
        predicted_x, predicted_y = self.next_point(self.a[1], self.a[2], center, radius)
        
        alpha = 0.7  # 70% predicted path, 30% direct chase
        goal_x = alpha * predicted_x + (1 - alpha) * self.target_pose.x
        goal_y = alpha * predicted_y + (1 - alpha) * self.target_pose.y
        
        self.get_logger().info(f"Target: ({self.target_pose.x:.2f}, {self.target_pose.y:.2f}) | "
                            f"Predicted: ({predicted_x:.2f}, {predicted_y:.2f}) | "
                            f"Final Goal: ({goal_x:.2f}, {goal_y:.2f}) | Radius: {radius:.2f}")
        
        self.a.append([goal_x, goal_y])
        self.move_to_goal([goal_x, goal_y])
    
    def next_point(self, p2, p3, center, radius):
        theta2 = atan2(p2[1] - center[1], p2[0] - center[0])
        theta3 = atan2(p3[1] - center[1], p3[0] - center[0])

        dtheta = theta3 - theta2  # Corrected angle progression 
        theta4 = theta3 + dtheta  # Predict next angle

        return [center[0] + radius * cos(theta4), center[1] + radius * sin(theta4)]

    
    def move_to_goal(self, goal):
        vel = Twist()
        
        distance = sqrt((self.pose.x - goal[0])**2 + (self.pose.y - goal[1])**2)
        angle_to_goal = atan2(goal[1] - self.pose.y, goal[0] - self.pose.x)
        angle_error = angle_to_goal - self.pose.theta
        angle_error = atan2(sin(angle_error), cos(angle_error))
        
        if distance < 0.3:
            self.vel_pub.publish(Twist())
            return
        
        vel.linear.x = 0.9
        vel.angular.z = max(-1.5, min(1.5, 4 * angle_error))
        
        self.vel_pub.publish(vel)
    
    def define_circle(self, p1, p2, p3):
        temp = p2[0]**2 + p2[1]**2
        bc = (p1[0]**2 + p1[1]**2 - temp) / 2
        cd = (temp - p3[0]**2 - p3[1]**2) / 2
        det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])
        if abs(det) < 1.0e-6:
            return (None, np.inf)
        cx = (bc * (p2[1] - p3[1]) - cd * (p1[1] - p2[1])) / det
        cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det
        return ((cx, cy), sqrt((cx - p1[0])**2 + (cy - p1[1])**2))

def main():
    rclpy.init()
    node = TurtleChase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
