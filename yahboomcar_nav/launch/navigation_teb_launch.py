import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Get the package path
    package_path = get_package_share_directory('yahboomcar_nav')
    description_package_path = get_package_share_directory('yahboomcar_description')

    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false', 
        description='Use simulation time if true'
    )
    
    declare_map_yaml_arg = DeclareLaunchArgument(
        'map', 
        default_value=os.path.join(package_path, 'maps', 'yahboomcar.yaml'),
        description='Path to the map file for navigation'
    )
    
    declare_nav2_param_path_arg = DeclareLaunchArgument(
        'params_file', 
        default_value=PathJoinSubstitution([package_path, 'params', 'teb_nav_params.yaml']),
        description='Path to the parameter file for navigation'
    )
    
    declare_bt_xml_path_arg = DeclareLaunchArgument(
        'bt_xml_file', 
        default_value=PathJoinSubstitution([package_path, 'params', 'behavior_tree.xml']),
        description='Path to the behavior tree file for bt_navigator'
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_map_yaml_arg,
        declare_nav2_param_path_arg,
        declare_bt_xml_path_arg,
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{      
                'autostart': True,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'node_names': ['planner_server', 
                                'recoveries_server', 
                                'map_server', 
                                'amcl',
                                'bt_navigator',]
            }],
        ),
        # AMCL node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        ),

        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        ),

        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        ),

        # Recovery server
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'default_bt_xml_filename': LaunchConfiguration('bt_xml_file')},]
        ),

        # Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        ),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'yaml_filename': LaunchConfiguration('map')},]
        ),
    ])
