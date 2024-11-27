import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav_package_launch_path = os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch')

    return LaunchDescription([
        # Chassis bringup
        Node(
            package='yahboomcar_bringup',
            executable='Ackman_driver_R2',
        ),
        # Delay before starting Lidar launch with Cartographer
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(PythonLaunchDescriptionSource(
                    [nav_package_launch_path, '/map_cartographer_launch.py'])
                )
            ]
        ),
        # Delay before starting TEB navigation
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(PythonLaunchDescriptionSource(
                    [nav_package_launch_path, '/navigation_teb_launch.py'])
                )
            ]
        ),
        # Delay before starting HMI listener and goal publisher
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='p2pnav',
                    executable='send_goal',
                )
            ]
        ),
    ])
