import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

def generate_launch_description():


    self_pkg = get_package_share_directory('regolife_vision')

    config = os.path.join(self_pkg, 'config', 'vision.yaml')

    detect = Node(
        package="regolife_vision", executable="detect_plants.py",
        emulate_tty = True,
        parameters=[config]
    )  



    return LaunchDescription(
        [  
            detect,
        ]
    )