from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joystick driver node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Robot teleop node
        Node(
            package='robot_controller',
            executable='robot_teleop',
            name='robot_teleop',
            output='screen',
            parameters=[
                {'linear_speed': 0.5},
                {'angular_speed': 1.0}
            ]
        ),
    ])
