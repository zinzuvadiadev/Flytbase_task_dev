import rclpy
import time
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

class TurtleController(Node):
    def __init__(self, waypoints):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = None
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.lin_pid = PIDController(1.5, 0.01, 0.2)  # Tune these values
        self.ang_pid = PIDController(4.0, 0.02, 0.3)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.start_time = time.time()
        self.kill_and_spawn_turtle()

    def pose_callback(self, msg):
        self.pose = msg
    
    def kill_and_spawn_turtle(self):
        kill_client = self.create_client(Kill, 'kill')
        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill service...')
        
        kill_request = Kill.Request()
        kill_request.name = 'turtle1'
        kill_client.call_async(kill_request)
        self.get_logger().info('Killed existing turtle.')
        
        spawn_client = self.create_client(Spawn, 'spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        spawn_request = Spawn.Request()
        spawn_request.x = random.uniform(1.0, 10.0)
        spawn_request.y = random.uniform(1.0, 10.0)
        spawn_request.theta = random.uniform(0, 2*np.pi)
        spawn_request.name = 'turtle1'
        spawn_client.call_async(spawn_request)
        self.get_logger().info(f'Spawned new turtle at ({spawn_request.x}, {spawn_request.y})')
    
    def control_loop(self):
        if self.pose is None or self.current_waypoint_index >= len(self.waypoints):
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint_index]
        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y
        distance = np.hypot(dx, dy)
        angle_to_goal = np.arctan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta
        
        dt = time.time() - self.start_time
        self.start_time = time.time()
        
        ang_vel = self.ang_pid.compute(angle_error, dt)
        lin_vel = self.lin_pid.compute(distance, dt) if abs(angle_error) < 0.1 else 0.0
        
        twist = Twist()
        twist.linear.x = min(2.0, max(-2.0, lin_vel))  # Limit speed
        twist.angular.z = min(2.0, max(-2.0, ang_vel))
        self.publisher.publish(twist)
        
        if distance < 0.2:
            self.get_logger().info(f'Reached Waypoint {self.current_waypoint_index + 1}: ({goal_x}, {goal_y})')
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('All waypoints reached!')
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    waypoints = []
    num_waypoints = int(input("Enter number of waypoints: "))
    for i in range(num_waypoints):
        x, y = map(float, input(f"Enter waypoint {i+1} (x y): ").split())
        waypoints.append((x, y))
    
    node = TurtleController(waypoints)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
