#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to your existing lio_sam launch file
    lio_sam_launch_dir = get_package_share_directory('lio_sam')
    lio_sam_launch_file = os.path.join(lio_sam_launch_dir, 'launch', 'run.launch.py')

    return LaunchDescription([
        # 1️⃣ Run the PointCloud converter
        ExecuteProcess(
            cmd=['ros2', 'run', 'isaac_sim_pointcloud_tool', 'converter'],
            output='screen'
        ),

        # 2️⃣ Launch LIO-SAM (Color)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lio_sam_launch_file)
        )
    ])
