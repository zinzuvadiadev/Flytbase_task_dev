import rclpy
from rclpy.node import Node
from turtlesim_msgs.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_msgs.srv import Spawn, Kill
import random
import math
import time
import numpy as np

class PoliceTurtleSlow(Node):
    def __init__(self):
        super().__init__('police_turtle_slow')
        self.get_logger().info("Police Turtle Slow node started.")
        
        time.sleep(10)  # Wait before spawning PT
        self.spawn_police_turtle()
        
        self.cmd_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.cmd_rt_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pt_pose_callback, 10)
        self.create_subscription(Pose, '/rt_real_pose', self.rt_pose_callback, 10)
        
        self.pt_pose = None
        self.rt_positions = []  # Store past RT positions
        self.rt_speed = 0.0
        self.circle_center = None
        self.circle_radius = None
        
        self.Kp_linear = 1.0
        self.Kp_angular = 4.0
        self.max_angular_speed = 2.0

        self.pt_speed = 0.1
        
        self.control_timer = self.create_timer(0.05, self.control_loop)
    
    def spawn_police_turtle(self):
        kill_client = self.create_client(Kill, 'kill')
        if kill_client.wait_for_service(timeout_sec=2.0):
            kill_req = Kill.Request()
            kill_req.name = "turtle2"
            future = kill_client.call_async(kill_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result():
                self.get_logger().info("Existing turtle2 killed successfully.")
            else:
                self.get_logger().info("No existing turtle2 found or failed to kill.")
        
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
        if len(self.rt_positions) >= 4:
            self.rt_positions.pop(0)
        self.rt_positions.append((msg.x, msg.y))
        
        if len(self.rt_positions) >= 4:
            self.estimate_circle_params()
    
    def estimate_circle_params(self):
        if len(self.rt_positions) < 4:
            return
        
        points = np.array(self.rt_positions)
        A = np.c_[2 * points, np.ones(points.shape[0])]
        b = np.sum(points ** 2, axis=1)        
        
        try:
            center = np.linalg.lstsq(A, b, rcond=None)[0][:2]
            self.circle_center = tuple(center)
            self.circle_radius = np.mean([math.dist(center, p) for p in self.rt_positions])
            self.get_logger().info(f"Estimated Circle: Center=({self.circle_center[0]:.2f}, {self.circle_center[1]:.2f}), Radius={self.circle_radius:.2f}")
        except:
            self.get_logger().error("Failed to estimate circle parameters")
    
    def predict_rt_position(self, future_time):
        if not self.circle_center or not self.circle_radius or len(self.rt_positions) < 2:
            return None
    
        x_r, y_r = self.rt_positions[-1]
        x_c, y_c = self.circle_center
        angle = math.atan2(y_r - y_c, x_r - x_c)
        
        self.rt_speed = math.dist(self.rt_positions[-1], self.rt_positions[-2]) / 5.0
        angle += self.rt_speed / self.circle_radius * future_time
        
        # lead_time = min(10, max(5, self.circle_radius / self.pt_speed))  # Adjust lead time dynamically
        # lead_angle = angle + (self.rt_speed / self.circle_radius) * lead_time  # Predict ahead
        return x_c + (self.circle_radius * 0.7) * math.cos(angle), y_c + (self.circle_radius * 0.7) * math.sin(angle) 
    
    def control_loop(self):
        if not self.pt_pose or len(self.rt_positions) < 2:
            return
        
        x_p, y_p = self.pt_pose.x, self.pt_pose.y
        future_rt_pos = self.predict_rt_position(5.0)
        if not future_rt_pos:
            return
        
        x_t, y_t = future_rt_pos
        dx, dy = x_t - x_p, y_t - y_p
        distance = ((x_t - x_p) ** 2 + (y_t - y_p) ** 2) ** 0.5
        
        if distance <= 2.0:
            self.get_logger().info("Chase successful! PT caught RT.")
            self.cmd_pub.publish(Twist())
            self.control_timer.cancel()
            return
        
        desired_angle = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(desired_angle - self.pt_pose.theta), math.cos(desired_angle - self.pt_pose.theta))
        
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = max(min(self.Kp_angular * angle_error * 1.5, self.max_angular_speed), -self.max_angular_speed)

        self.cmd_pub.publish(twist)
    

def main(args=None):

    rclpy.init(args=args)
    node = PoliceTurtleSlow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
