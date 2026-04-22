import os
from pathlib import Path

import launch
import launch_ros
import yaml
from ament_index_python.packages import get_package_share_directory


def load_ros_parameters(config_path: str, node_name: str) -> dict:
    with open(config_path, 'r') as config_file:
        config = yaml.safe_load(config_file) or {}

    node_config = config.get(node_name) or config.get('/**') or {}
    return node_config.get('ros__parameters', node_config)


def resolve_map_path(raw_path: str, default_pkg_path: str) -> str:
    expanded_path = Path(os.path.expandvars(os.path.expanduser(raw_path)))
    if raw_path.startswith('package://'):
        package_name, _, relative_path = raw_path.removeprefix('package://').partition('/')
        if not package_name or not relative_path:
            raise ValueError(f'Invalid package URI: {raw_path}')
        return str((Path(get_package_share_directory(package_name)) / relative_path).resolve())

    if expanded_path.is_absolute():
        return str(expanded_path)

    default_pkg = Path(default_pkg_path).resolve()
    workspace_root = default_pkg.parents[3]
    install_root = default_pkg.parents[2]

    candidates = [
        default_pkg / expanded_path,
        workspace_root / expanded_path,
        install_root / expanded_path,
    ]

    for candidate in candidates:
        if candidate.exists():
            return str(candidate.resolve())

    return str((default_pkg / expanded_path).resolve())


def generate_launch_description():

    package_path = get_package_share_directory('path_planning_dynamic')
    paramsConfig = os.path.join(package_path, 'config', 'params.yaml')
    planner_params = load_ros_parameters(paramsConfig, 'path_planning_node')
    map_path = resolve_map_path(
        planner_params.get(
            'map_path',
            'package://autonomous_robot_simulation/utils/depot_lanelet2_map.osm',
        ),
        package_path,
    )


    publisher_node_planner = launch_ros.actions.Node(
        package='path_planning_dynamic',
        executable='path_planning_node',
        name='path_planning_node',
        output='screen',
        parameters=[paramsConfig, {'map_path': map_path}],
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
