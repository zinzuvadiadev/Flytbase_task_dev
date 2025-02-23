import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import Pose
import numpy as np
from filterpy.kalman import KalmanFilter

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        
        # Subscriber to noisy RT position
        self.subscription = self.create_subscription(
            Pose,
            '/rt_noisy_pose',  # Change topic accordingly
            self.pose_callback,
            10)
        self.subscription
        
        # Publisher for filtered position
        self.filtered_pose_pub = self.create_publisher(Pose, '/rt_filtered_pose', 10)
        
        # Kalman Filter setup
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.F = np.array([[1, 1, 0, 0],  # State transition matrix
                               [0, 1, 0, 0],
                               [0, 0, 1, 1],
                               [0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0],
                               [0, 0, 1, 0]])  # Measurement function
        self.kf.P *= 1000  # Covariance matrix
        self.kf.R = np.array([[0.1, 0], [0, 0.1]])  # Measurement noise
        self.kf.Q = np.eye(4) * 0.01  # Process noise
        self.kf.x = np.array([[0], [0], [0], [0]])  # Initial state
    
    def pose_callback(self, msg):
        z = np.array([[msg.x], [msg.y]])  # Noisy measurement
        self.kf.predict()
        self.kf.update(z)
        
        # Publish filtered pose
        filtered_msg = Pose()
        filtered_msg.x = self.kf.x[0][0]
        filtered_msg.y = self.kf.x[2][0]
        self.filtered_pose_pub.publish(filtered_msg)

        self.get_logger().info(f'Filtered Pose: ({filtered_msg.x}, {filtered_msg.y})')


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
