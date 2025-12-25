import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch.conditions import IfCondition

def generate_launch_description():
    robot_names = ['robot1']
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    multi_carto_nav2_dir = get_package_share_directory('multi_carto_nav2')
    carto_params_file = os.path.join(multi_carto_nav2_dir, 'config')
    
    for robot_name in robot_names:        
        nav2_params_file = os.path.join(multi_carto_nav2_dir, 'params', f'{robot_name}.yaml')
        
        robot_group = GroupAction([
            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),

            # 1. Static TF
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_footprint_to_base',
                output='screen',
                arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                           f'{robot_name}/base_footprint', f'{robot_name}/base_link']
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_base_to_laser',
                output='screen',
                arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                           f'{robot_name}/base_link', f'{robot_name}/laser_frame']
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_base_to_imu',
                output='screen',
                arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                           f'{robot_name}/base_link', f'{robot_name}/imu_frame']
            ),

            # 2. Cartographer SLAM
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': False}],
                arguments=[
                    '-configuration_directory', carto_params_file,
                    '-configuration_basename', f'{robot_name}.lua'
                ],
                remappings=[
                    ('scan', 'scan'),
                    ('imu', 'imu'),
                ]
            ),
            
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {'resolution': 0.05},
                    {'publish_period_sec': 1.0}
                ]
            ),
            
            # 3. Nav2 Container
            Node(
                package='rclcpp_components',
                executable='component_container_isolated',
                name='nav2_container',
                parameters=[nav2_params_file, {'autostart': True}],
                arguments=['--ros-args', '--log-level', 'info'],
                output='screen'
            ),

            # 4. Navigation2
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'False',
                    'params_file': nav2_params_file,
                    'namespace': robot_name,
                    'use_namespace': 'True',
                    'autostart': 'True',
                    'use_composition': 'True',
                    'container_name': 'nav2_container'
                }.items()
            ),

            # 5. Rviz2
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(multi_carto_nav2_dir, 'rviz', f'{robot_name}.rviz')],
                output='screen'
            ),
        ])
        
        ld = LaunchDescription()
        ld.add_action(robot_group)

    return ld
