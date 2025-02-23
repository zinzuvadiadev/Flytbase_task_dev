#!/usr/bin/env python3
"""
This node drives the turtlesim turtle in a circle and publishes:
- Velocity commands on `/turtle1/cmd_vel`
- The current circle radius on `/circle_radius`
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class CircleTurtlePublisher(Node):
    def __init__(self, linear_speed, circle_radius):
        super().__init__('circle_turtle_publisher')

        # Store user inputs
        self.linear_speed = linear_speed
        self.circle_radius = circle_radius

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.radius_pub = self.create_publisher(Float32, '/custom_circle_radius', 10)

        # Timer to publish velocity
        self.cmd_timer = self.create_timer(0.05, self.cmd_vel_callback)
        # Timer to publish radius
        self.radius_timer = self.create_timer(2.0, self.publish_radius)

        self.get_logger().info(
            f"Circle Turtle Publisher started with radius={self.circle_radius}, linear_speed={self.linear_speed}"
        )

    def cmd_vel_callback(self):
        """Publishes only linear velocity, no angular velocity."""
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0  # Not publishing angular velocity
        self.cmd_vel_pub.publish(twist)

    def publish_radius(self):
        """Publishes the current radius as a Float32 message."""
        msg = Float32()
        msg.data = self.circle_radius
        self.radius_pub.publish(msg)
        self.get_logger().info(f"Published Radius: {self.circle_radius:.2f}")


def main(args=None):
    rclpy.init(args=args)

    # Take user input for linear speed and radius
    try:
        linear_speed = float(input("Enter linear speed: "))
        circle_radius = float(input("Enter circle radius: "))
    except ValueError:
        print("Invalid input! Please enter numerical values.")
        return

    node = CircleTurtlePublisher(linear_speed, circle_radius)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
