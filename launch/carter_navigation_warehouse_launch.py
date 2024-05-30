
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    package_dir = get_package_share_directory("carter_navigation_warehouse")

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            package_dir, "maps", "carter_warehouse_navigation.yaml"
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            package_dir, "params", "carter_navigation_params.yaml"
        ),
    )

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(package_dir, "rviz2", "carter_navigation.rviz")

    planys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("plansys2_bringup"), 
            "launch", 
            "plansys2_bringup_launch.py")),
        launch_arguments={"model_file": os.path.join(
            package_dir, "pddl", "carter_navigation.pddl")}.items()
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_bringup_launch_dir, 
            "rviz_launch.py")),
        launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_dir}.items(),
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
        launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": param_dir}.items(),
    )
            
    move_cmd = Node(
        package='carter_navigation_warehouse',
        executable='move_action_node',
        name='move_action_node',
        output='screen',
        parameters=[])

    patrol_cmd = Node(
        package='carter_navigation_warehouse',
        executable='patrol_action_node',
        name='patrol_action_node',
        output='screen',
        parameters=[])
    
    pointcloud_to_laserscan_cmd = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in', ['/front_3d_lidar/point_cloud']),
                            ('scan', ['/scan'])],
                parameters=[{
                    'target_frame': 'front_3d_lidar',
                    'transform_tolerance': 0.01,
                    'min_height': -0.4,
                    'max_height': 1.5,
                    'angle_min': -1.5708,  # -M_PI/2
                    'angle_max': 1.5708,  # M_PI/2
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.3333,
                    'range_min': 0.05,
                    'range_max': 100.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0,
                    # 'concurrency_level': 1,
                }],
                name='pointcloud_to_laserscan'
    )

    

    ld = LaunchDescription()

    ld.add_action(planys2_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(move_cmd)
    ld.add_action(patrol_cmd)
    ld.add_action(pointcloud_to_laserscan_cmd)

    return ld