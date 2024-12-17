import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_nav')
    # nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(package_path, 'maps', 'yahboomcar.yaml'))
    nav2_param_path = PathJoinSubstitution([
        package_path,
        'params',
        'teb_nav_params.yaml'
    ])
    bt_xml_path = PathJoinSubstitution([
        package_path,
        'params',
        'behavior_tree.xml'
    ])
    
    # Return the LaunchDescription object
    return LaunchDescription([
         # AMCL node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Recovery server
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Costmap converter
        Node(
            package='costmap_converter',
            executable='costmap_converter',
            name='costmap_converter',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Global costmap
        Node(
            package='nav2_costmap_2d',
            executable='global_costmap',
            name='global_costmap',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Local costmap
        Node(
            package='nav2_costmap_2d',
            executable='local_costmap',
            name='local_costmap',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}, {'default_bt_xml_filename': bt_xml_path}]
        ),

        # Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}, {'yaml_filename': LaunchConfiguration('map')}]
        ),

        # Map Saver
        Node(
            package='nav2_map_server',
            executable='map_saver',
            name='map_saver',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
    ])
