#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped


class RobotTeleop(Node):
    def __init__(self):
        super().__init__('robot_teleop')
        
        # Publisher and Subscriber
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Drive parameters
        self.max_speed = 2.0               # m/s max linear speed
        self.max_steering_angle = -0.4      # radians (~23 degrees)
        self.max_acceleration = 1.0        # m/s²
        self.max_steering_rate = 0.5       # rad/s

        self.get_logger().info("Joystick Ackermann teleop node started")

    def joy_callback(self, msg: Joy):
        """
        Map joystick axes to AckermannDriveStamped command.
        Typically:
          - Left stick vertical axis = msg.axes[1] (forward/backward)
          - Right stick horizontal axis = msg.axes[3] (steering)
        """

        speed = msg.axes[1] * self.max_speed
        steering_angle = msg.axes[2] * self.max_steering_angle

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.acceleration = self.max_acceleration
        drive_msg.drive.steering_angle_velocity = self.max_steering_rate

        self.cmd_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
