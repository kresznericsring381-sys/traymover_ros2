"""Launch: simple 3D point-cloud navigation (option 14).

Replaces the Nav2 + 2D PGM path of traymover_nav.launch.py with CMU's
autonomous_exploration_development_environment local stack:
  * terrain_analysis — builds /terrain_map from FAST_LIO /cloud_registered
    + /Odometry, labelling points by height relative to ground.
  * local_planner/localPlanner — picks a local motion primitive path toward
    /way_point.
  * local_planner/pathFollower — turns the chosen path into TwistStamped
    on /cmd_vel_stamped.

Shims we add to fit our chassis:
  * goal_pose_to_waypoint.py — RViz "2D Goal Pose" -> /way_point, with
    TF transform into camera_init (CMU's internal world frame).
  * twist_stamped_to_twist.py — pathFollower's TwistStamped ->
    /cmd_vel (Twist) for the base serial driver, gated until a goal arrives.
  * topic remaps — keep CMU's follower path isolated from FAST_LIO/NDT
    diagnostic paths, while preserving those paths for RViz inspection.

Reuses:
  * lidar_localization.launch.py — FAST_LIO + NDT map->odom correction
    against a prior PCD (so goal coords remain map-consistent).

Frame notes:
  * FAST_LIO /Odometry and /cloud_registered are both in `camera_init`.
    We feed them directly to CMU as /state_estimation and /registered_scan.
    CMU code is frame-agnostic (reads pose values, not header.frame_id),
    so internal consistency is what matters.
  * Bridge transforms goals from RViz's `map` frame into
    `waypoint_target_frame` (default: camera_init) so /way_point coords
    match /state_estimation pose coords.
  * CMU publishes local paths in a `vehicle` frame. We attach `vehicle`
    to base_link for visualization only; pathFollower still consumes the
    numeric path relative to the pose captured when the path arrived.

Common invocations:
  ros2 launch traymover_robot_nav traymover_3d_nav.launch.py \\
       launch_rviz:=true pcd_path:=/abs/path/to/map.pcd
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Match DEFAULT_PCD in traymover_nav.launch.py so switching between
# options picks the same map unless overridden.
DEFAULT_PCD = ('/home/wheeltec/traymover_ros2/src/traymover_robot_slam/'
               'FAST_LIO/PCD/traymover.pcd')


def find_default_pcd() -> str:
    pcd_dir = Path('/home/wheeltec/traymover_ros2/src/traymover_robot_slam/FAST_LIO/PCD')
    candidates = sorted(pcd_dir.glob('*.pcd'), key=lambda p: p.stat().st_mtime, reverse=True)
    if candidates:
        return str(candidates[0])
    return DEFAULT_PCD


def generate_launch_description():
    nav_share = get_package_share_directory('traymover_robot_nav')
    bringup_share = get_package_share_directory('turn_on_traymover_robot')
    local_planner_share = get_package_share_directory('local_planner')

    default_terrain_params = os.path.join(
        nav_share, 'config', 'terrain_analysis.yaml')
    default_planner_params = os.path.join(
        nav_share, 'config', 'local_planner.yaml')
    default_rviz = os.path.join(
        nav_share, 'rviz', 'traymover_3d_nav.rviz')
    pointcloud_launch = os.path.join(
        nav_share, 'launch', 'navigation_pointcloud.launch.py')
    paths_dir = os.path.join(local_planner_share, 'paths')

    use_sim_time = LaunchConfiguration('use_sim_time')
    pcd_path = LaunchConfiguration('pcd_path')
    bringup_hardware = LaunchConfiguration('bringup_hardware')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    terrain_params = LaunchConfiguration('terrain_params')
    planner_params = LaunchConfiguration('planner_params')
    waypoint_target_frame = LaunchConfiguration('waypoint_target_frame')
    local_path_topic = LaunchConfiguration('local_path_topic')
    fastlio_path_topic = LaunchConfiguration('fastlio_path_topic')
    localization_path_topic = LaunchConfiguration('localization_path_topic')

    # 0) Optional hardware bringup
    hw_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'base_serial.launch.py')),
        condition=IfCondition(bringup_hardware),
        launch_arguments={
            'odom_source_mode': 'none',
        }.items(),
    )
    hw_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'traymover_lidar.launch.py')),
        condition=IfCondition(bringup_hardware),
        launch_arguments={
            'enable_scan_bridge': 'false',
        }.items(),
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

    # 1) Localization chain: RSP + FAST_LIO + NDT map->odom.
    #    Produces /odom, /cloud_registered, and TF map->odom->camera_init->body.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'lidar_localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'pcd_path': pcd_path,
            'cloud_topic': '/point_cloud_localization',
            'fastlio_path_topic': fastlio_path_topic,
            'localization_path_topic': localization_path_topic,
            # Option 14 relies on FAST_LIO for short-term motion. NDT is only
            # allowed to make slow global corrections, otherwise repeated-wall
            # false matches can rotate map->odom with the robot.
            'max_map_odom_update_translation': '0.30',
            'max_map_odom_update_rotation': '0.12',
        }.items(),
    )

    # CMU localPlanner publishes nav_msgs/Path in frame_id="vehicle".
    # Add a visualization-only frame so RViz can transform /local_planner_path
    # and /free_paths into the fixed map frame. The planner/follower math does
    # not depend on this TF.
    static_base_to_vehicle = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_vehicle',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'vehicle'],
        output='screen',
    )

    # 2) CMU stack. Wait until localization lifecycle has activated
    #    (NDT publishes map->odom only after ~6-10 s), otherwise the
    #    bridge's TF lookup fails on the first goal.
    terrain_analysis = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        name='terrainAnalysis',
        output='screen',
        parameters=[terrain_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/state_estimation', '/odom'),
            ('/registered_scan', '/cloud_registered'),
        ],
    )
    local_planner = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[
            planner_params,
            {'pathFolder': paths_dir, 'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/state_estimation', '/odom'),
            ('/registered_scan', '/cloud_registered'),
            ('/path', local_path_topic),
        ],
    )
    path_follower = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[planner_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/state_estimation', '/odom'),
            ('/path', local_path_topic),
            ('/cmd_vel', '/cmd_vel_stamped'),
        ],
    )
    goal_bridge = Node(
        package='traymover_robot_nav',
        executable='goal_pose_to_waypoint.py',
        name='goal_pose_to_waypoint',
        output='screen',
        parameters=[{
            'goal_topic': '/goal_pose',
            'waypoint_topic': '/way_point',
            'target_frame': waypoint_target_frame,
            'use_sim_time': use_sim_time,
        }],
    )
    twist_shim = Node(
        package='traymover_robot_nav',
        executable='twist_stamped_to_twist.py',
        name='twist_stamped_to_twist',
        output='screen',
        parameters=[{
            'in_topic': '/cmd_vel_stamped',
            'out_topic': '/cmd_vel',
            'use_sim_time': use_sim_time,
        }],
    )

    cmu_stack_delayed = TimerAction(
        period=8.0,
        actions=[
            terrain_analysis,
            local_planner,
            path_follower,
            goal_bridge,
            twist_shim,
        ],
    )

    # 3) Optional RViz.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'pcd_path', default_value=find_default_pcd(),
            description='Prior PCD map loaded by lidar_localization_ros2.'),
        DeclareLaunchArgument(
            'bringup_hardware', default_value='false',
            description='Also start base_serial + traymover_lidar.'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='Also start RViz with the 3D nav profile.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument(
            'terrain_params', default_value=default_terrain_params,
            description='YAML overriding terrainAnalysis parameters.'),
        DeclareLaunchArgument(
            'planner_params', default_value=default_planner_params,
            description='YAML overriding localPlanner + pathFollower parameters.'),
        DeclareLaunchArgument(
            'waypoint_target_frame', default_value='camera_init',
            description='World frame used by CMU stack; RViz goals are '
                        'transformed into this frame before being republished.'),
        DeclareLaunchArgument(
            'local_path_topic', default_value='/local_planner_path',
            description='Isolated CMU localPlanner path consumed by pathFollower.'),
        DeclareLaunchArgument(
            'fastlio_path_topic', default_value='/fastlio_path',
            description='FAST_LIO diagnostic trajectory path shown in RViz.'),
        DeclareLaunchArgument(
            'localization_path_topic', default_value='/pcl_path',
            description='NDT/lidar_localization diagnostic trajectory path shown in RViz.'),
        hw_base,
        hw_lidar,
        pointcloud_pipeline,
        localization,
        static_base_to_vehicle,
        cmu_stack_delayed,
        rviz,
    ])
