import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

def generate_launch_description():


    self_pkg = get_package_share_directory('plant_monitor')

    config = os.path.join(self_pkg, 'config', 'config.yaml')

    detect = Node(
        package="plant_monitor", executable="check_ph.py",
        emulate_tty = True,
        parameters=[config]
    )  



    return LaunchDescription(
        [  
            detect,
        ]
    )