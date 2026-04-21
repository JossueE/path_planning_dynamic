import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('path_planning_dynamic'),'config','params.yaml')


    publisher_node_planner = launch_ros.actions.Node(
        package='path_planning_dynamic',
        executable='path_planning_node',
        name='path_planning_node',
        output='screen',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    pointcloud_clustering = launch_ros.actions.Node(
        package='path_planning_dynamic',
        executable='pointcloud_clustering_node',
        name='pointcloud_clustering_node',
        output='screen',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    pointcloud_roi = launch_ros.actions.Node(
        package='path_planning_dynamic',
        executable='pointcloud_roi_node',
        name='pointcloud_roi_node',
        output='screen',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    simu_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    return launch.LaunchDescription([
        simu_time,
        pointcloud_roi,
        pointcloud_clustering,
        publisher_node_planner,
        
    ])