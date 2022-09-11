#!/usr/bin/python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('slim_bridge')
    config_file = os.path.join(pkg_dir, 'config', 'initial_topics.yaml')
    print (config_file)
    slim_bridge = Node(
        package='slim_bridge', executable='bridge',
        parameters=[
            {'initial_topics_filepath': config_file}
        ],
        output='screen'
    )
    return LaunchDescription([slim_bridge])