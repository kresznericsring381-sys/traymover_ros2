"""Launch: Phase 2 full traymover autonomous navigation.

Brings up the full runtime chain:
  0. (optional) hardware bringup ¡ª base_serial + traymover_lidar
     (set bringup_hardware:=true for one-command start; default false).
  1. lidar_localization.launch.py ¡ª robot_state_publisher + FAST_LIO online
     (publishes /odom = camera_init->body twist) + static TFs stitching
     FAST_LIO frames into base_link + lidar_localization_ros2 (NDT_OMP,
     publishes map -> odom correction against the prebuilt PCD).
  2. nav2 navigation stack ¡ª map_server, planner_server, controller_server,
     bt_navigator, lifecycle_manager.
     controller cmd_vel goes directly to /cmd_vel.
  3. (optional) RViz ¡ª set launch_rviz:=true.

Common launch invocations:
  # Preferred for real-hardware test (single command, one window):
  ros2 launch traymover_robot_nav traymover_nav.launch.py \\
       bringup_hardware:=true launch_rviz:=true

  # Multi-terminal for debugging (this launch only; run hardware separately):
  # T1: ros2 launch turn_on_traymover_robot base_serial.launch.py
  # T2: ros2 launch turn_on_traymover_robot traymover_lidar.launch.py
  # T3: ros2 launch traymover_robot_nav traymover_nav.launch.py launch_rviz:=true

After startup, use RViz "2D Pose Estimate" to set initial pose, then "2D Goal
Pose" to send a navigation goal.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


NAV2_LIFECYCLE_NODES = [
    'map_server',
    'planner_server',
    'controller_server',
    'bt_navigator',
]

# Keep in sync with default_pcd in lidar_localization.launch.py and
# map_path in config/localization.yaml.
DEFAULT_PCD = ('/home/wheeltec/traymover_ros2/src/traymover_robot_slam/'
               'FAST_LIO/PCD/traymover.pcd')


def find_default_pcd() -> str:
    pcd_dir = Path('/home/wheeltec/traymover_ros2/src/traymover_robot_slam/FAST_LIO/PCD')
    candidates = sorted(pcd_dir.glob('*.pcd'), key=lambda p: p.stat().st_mtime, reverse=True)
    if candidates:
        return str(candidates[0])
    return DEFAULT_PCD


def _create_hardware_actions(context, *args, **kwargs):
    """Resolve hardware-only package paths only when hardware is requested."""
    bringup_hardware = LaunchConfiguration('bringup_hardware').perform(context)
    if bringup_hardware.lower() not in ('true', '1', 'yes', 'on'):
        return []

    bringup_share = get_package_share_directory('turn_on_traymover_robot')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'base_serial.launch.py')),
            launch_arguments={
                'odom_source_mode': 'none',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'traymover_lidar.launch.py')),
            launch_arguments={
                'enable_scan_bridge': 'false',
            }.items(),
        ),
    ]


def generate_launch_description():
    nav_share = get_package_share_directory('traymover_robot_nav')

    default_params = os.path.join(nav_share, 'config', 'nav2_params.yaml')
    default_map = os.path.join(nav_share, 'map', 'traymover_2d.yaml')
    default_rviz = os.path.join(nav_share, 'rviz', 'traymover_nav.rviz')
    default_bt = os.path.join(nav_share, 'behavior_trees', 'simple_navigate_to_pose.xml')
    default_through_poses_bt = os.path.join(
        nav_share, 'behavior_trees', 'simple_navigate_through_poses.xml')
    pointcloud_launch = os.path.join(nav_share, 'launch', 'navigation_pointcloud.launch.py')

    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    bringup_hardware = LaunchConfiguration('bringup_hardware')
    launch_rviz = LaunchConfiguration('launch_rviz')
    publish_fastlio_clouds = LaunchConfiguration('publish_fastlio_clouds')
    fastlio_frame_trace = LaunchConfiguration('fastlio_frame_trace')
    fastlio_min_effective_features = LaunchConfiguration('fastlio_min_effective_features')
    fastlio_max_update_translation = LaunchConfiguration('fastlio_max_update_translation')
    rviz_config = LaunchConfiguration('rviz_config')
    pcd_path = LaunchConfiguration('pcd_path')
    bt_xml = LaunchConfiguration('bt_xml')
    through_poses_bt_xml = LaunchConfiguration('through_poses_bt_xml')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': bt_xml,
                'default_nav_through_poses_bt_xml': through_poses_bt_xml,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    pointcloud_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pointcloud_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'input_cloud_topic': '/point_cloud_raw',
            'localization_cloud_topic': '/point_cloud_localization',
            'nav_cloud_topic': '/point_cloud_nav',
            'scan_topic': '/scan',
            'target_frame': 'base_link',
            'publish_scan': 'false',
        }.items(),
    )

    # 1) Navigation cloud preprocessing + Localization chain (RSP + NDT).
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'lidar_localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'pcd_path': pcd_path,
            'cloud_topic': '/point_cloud_localization',
            'max_map_odom_update_translation': '0.50',
            'max_map_odom_update_rotation': '0.25',
            'publish_fastlio_clouds': publish_fastlio_clouds,
            'fastlio_frame_trace': fastlio_frame_trace,
            'fastlio_min_effective_features': fastlio_min_effective_features,
            'fastlio_max_update_translation': fastlio_max_update_translation,
        }.items(),
    )

    # 2) Nav2 lifecycle nodes. Cmd_vel is intentionally direct and the BT is
    #    plan->follow only: no dynamic obstacle monitor, interlock topic,
    #    scan-based stop layer, or recovery behaviors.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
    )
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
    )
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
    )
    bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
    )
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': NAV2_LIFECYCLE_NODES,
        }],
    )

    # 3) Optional RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'bringup_hardware', default_value='false',
            description='Also start base_serial + traymover_lidar (true for one-command real-hardware test).'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='Also start RViz with the navigation profile.'),
        DeclareLaunchArgument(
            'publish_fastlio_clouds', default_value='false',
            description='Publish FAST_LIO path and registered clouds; odometry/TF remain enabled.'),
        DeclareLaunchArgument(
            'fastlio_frame_trace', default_value='true',
            description='Emit FAST_LIO per-frame stage diagnostics during navigation replay.'),
        DeclareLaunchArgument(
            'fastlio_min_effective_features', default_value='1000',
            description='Reject low-feature FAST_LIO updates during navigation replay; <=0 disables.'),
        DeclareLaunchArgument(
            'fastlio_max_update_translation', default_value='2.0',
            description='Reject FAST_LIO EKF corrections above this many meters during navigation replay; <=0 disables.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument(
            'bt_xml', default_value=default_bt,
            description='Behavior tree used for option-10 point-to-point navigation.'),
        DeclareLaunchArgument(
            'through_poses_bt_xml', default_value=default_through_poses_bt,
            description='Recovery-free through-poses tree loaded by bt_navigator.'),
        DeclareLaunchArgument(
            'pcd_path', default_value=find_default_pcd(),
            description='Prior PCD map loaded by lidar_localization_ros2.'),
        # Resolve this optional dependency after launch arguments are available.
        # Offline replay does not install or need the hardware bringup package.
        OpaqueFunction(function=_create_hardware_actions),
        pointcloud_pipeline,
        localization,
        map_server,
        planner,
        controller,
        bt,
        lifecycle_mgr,
        rviz,
    ])
