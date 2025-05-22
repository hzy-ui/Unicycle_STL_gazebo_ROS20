#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.timer = self.create_timer(0.1, self.move_robot)
        self.twist = Twist()
        self.laser_data = None

    def laser_callback(self, msg):
        """Callback to process laser scan data."""
        self.laser_data = msg.ranges

    def move_robot(self):
        """Control the robot based on laser scan data."""
        if self.laser_data is None:
            self.get_logger().warn('No laser data received yet')
            return

        # Define obstacle threshold distance (in meters)
        obstacle_threshold = 0.5

        # Split laser data into front, left, and right regions
        left = min(self.laser_data[:90])  # Laser data from 0° to 90°
        front = min(self.laser_data[90:270])  # Laser data from 90° to 270°
        right = min(self.laser_data[270:])  # Laser data from 270° to 360°

        self.get_logger().info(f"Left: {left:.2f}, Front: {front:.2f}, Right: {right:.2f}")

        # Basic obstacle avoidance logic
        if front < obstacle_threshold:
            # Obstacle ahead, turn left
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        elif left < obstacle_threshold:
            # Obstacle on the left, turn right
            self.twist.linear.x = 0.1
            self.twist.angular.z = -0.3
        elif right < obstacle_threshold:
            # Obstacle on the right, turn left
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.3
        else:
            # No obstacle, move forward
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0

        self.publisher_.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBot3Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

