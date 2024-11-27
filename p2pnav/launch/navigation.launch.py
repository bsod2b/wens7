import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav_package_launch_path =os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch')

    return LaunchDescription([
        # Chassis bringup
        Node(
            package='yahboomcar_bringup',
            executable='Ackman_driver_R2',
        ),
        # Lidar launch with Cartographer
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [nav_package_launch_path,'/map_cartographer_launch.py'])
        ),
        # TEB navigation
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [nav_package_launch_path,'/navigation_teb_launch.py'])
        ),
        # HMI listener and goal publisher
        Node(
            package='p2pnav',
            executable='send_goal',
        ),
    ])
