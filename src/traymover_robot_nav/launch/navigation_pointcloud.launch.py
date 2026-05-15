"""Launch the navigation-specific point cloud preprocessing pipeline.

This keeps the raw 360 degree LiDAR topic untouched for mapping while creating
navigation-friendly derivatives:
  * /point_cloud_localization  - base_link-frame cloud with a compact near-rear
                                 exclusion box removed so a trailing operator
                                 perturbs NDT less while still preserving most
                                 side / rear walls for U-turn alignment.
  * /point_cloud_nav           - base_link-frame cloud retained for tools that
                                 still want a navigation cloud.

The LaserScan bridge is off by default. Option 10 intentionally avoids dynamic
obstacle and stop logic; consumers that still need /scan can pass
publish_scan:=true.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('traymover_robot_nav')
    use_sim_time = LaunchConfiguration('use_sim_time')
    input_cloud_topic = LaunchConfiguration('input_cloud_topic')
    localization_cloud_topic = LaunchConfiguration('localization_cloud_topic')
    nav_cloud_topic = LaunchConfiguration('nav_cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    target_frame = LaunchConfiguration('target_frame')
    publish_scan = LaunchConfiguration('publish_scan')

    pointcloud_preprocessor = Node(
        package='traymover_robot_nav',
        executable='pointcloud_nav_preprocessor.py',
        name='pointcloud_nav_preprocessor',
        output='screen',
        parameters=[{
            'input_topic': input_cloud_topic,
            'localization_topic': localization_cloud_topic,
            'nav_topic': nav_cloud_topic,
            'target_frame': target_frame,
            # Keep localization close to full 360 deg but carve out the
            # immediate follow-me zone behind the robot. The previous box was
            # so wide that sharp turns lost too much rear geometry.
            'rear_filter_min_x': -0.95,
            'rear_filter_max_x': -0.20,
            'rear_filter_half_width': 0.65,
            'rear_filter_min_z': -0.50,
            'rear_filter_max_z': 2.00,
            'use_sim_time': use_sim_time,
        }],
    )

    scan_bridge = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='traymover_navigation_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', nav_cloud_topic),
            ('scan', scan_topic),
        ],
        parameters=[{
            'target_frame': target_frame,
            'transform_tolerance': 0.05,
            'min_height': -0.25,
            'max_height': 1.05,
            # 270 deg scan: front + both sides for tighter cornering and less
            # planner blindness during large heading changes.
            'angle_min': -2.35619449,
            'angle_max': 2.35619449,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.30,
            'range_max': 60.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(publish_scan),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('input_cloud_topic', default_value='/point_cloud_raw'),
        DeclareLaunchArgument('localization_cloud_topic', default_value='/point_cloud_localization'),
        DeclareLaunchArgument('nav_cloud_topic', default_value='/point_cloud_nav'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_scan', default_value='false'),
        pointcloud_preprocessor,
        scan_bridge,
    ])
