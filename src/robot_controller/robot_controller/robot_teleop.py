#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class RobotTeleop(Node):
    def __init__(self):
        super().__init__('robot_teleop')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Velocity parameters
        self.linear_speed = 0.5   # m/s max linear speed
        self.angular_speed = 1.0  # rad/s max angular speed

        self.get_logger().info("Joystick teleop node started")

    def joy_callback(self, msg: Joy):
        # Typically:
        #   Left stick vertical axis = msg.axes[1] (forward/backward)
        #   Right stick horizontal axis = msg.axes[3] (turning)
        linear_x = msg.axes[1] * self.linear_speed
        angular_z = msg.axes[3] * self.angular_speed

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        self.cmd_pub.publish(twist)

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

