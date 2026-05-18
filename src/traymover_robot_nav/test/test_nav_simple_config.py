#!/usr/bin/env python3
import copy
from pathlib import Path
import subprocess

import yaml


PKG_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PKG_ROOT.parent
WORKSPACE_ROOT = SRC_ROOT.parent


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def construct_mapping_without_duplicates(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise AssertionError(f'duplicate YAML key: {key}')
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    construct_mapping_without_duplicates,
)


def load_unique_yaml(path):
    with path.open('r', encoding='utf-8') as f:
        return yaml.load(f, Loader=UniqueKeyLoader)


def assert_close(actual, expected, tolerance=1.0e-9):
    assert abs(actual - expected) <= tolerance


def test_nav2_option10_has_no_dynamic_obstacle_stop_chain():
    params = load_unique_yaml(PKG_ROOT / 'config' / 'nav2_params.yaml')
    controller_params = params['controller_server']['ros__parameters']
    follow_path = controller_params['FollowPath']
    progress_checker = controller_params['progress_checker']
    bt_params = params['bt_navigator']['ros__parameters']
    local_costmap = params['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = params['global_costmap']['global_costmap']['ros__parameters']

    assert follow_path['plugin'] == (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    )
    assert follow_path['use_collision_detection'] is False
    assert follow_path['use_rotate_to_heading'] is True
    assert follow_path['allow_reversing'] is False
    assert 0.0 < follow_path['rotate_to_heading_min_angle'] <= 0.35
    assert 0.0 < follow_path['rotate_to_heading_angular_vel'] <= 0.45
    assert 0.0 < follow_path['max_angular_accel'] <= 0.8
    assert progress_checker['plugin'] == 'traymover_robot_nav::YawProgressChecker'
    assert progress_checker['required_movement_radius'] == 0.30
    assert 0.15 <= progress_checker['required_movement_angle'] <= 0.25
    assert progress_checker['movement_time_allowance'] == 8.0
    assert 'obstacle_layer' not in local_costmap['plugins']
    assert 'obstacle_layer' not in global_costmap['plugins']
    assert 'behavior_server' not in params
    assert 'waypoint_follower' not in params
    assert 'default_nav_to_pose_bt_xml' in bt_params
    assert 'default_nav_through_poses_bt_xml' in bt_params


def test_option10_launch_does_not_route_through_collision_monitor():
    launch_text = (PKG_ROOT / 'launch' / 'traymover_nav.launch.py').read_text(
        encoding='utf-8'
    )

    assert 'collision_monitor.launch.py' not in launch_text
    assert 'cmd_vel_nav' not in launch_text
    assert '/local_planner_path' not in launch_text
    assert '/fastlio_path' not in launch_text
    assert '/pcl_path' not in launch_text
    assert "'publish_scan': 'false'" in launch_text
    assert "'max_map_odom_update_translation': '0.50'" in launch_text
    assert "'max_map_odom_update_rotation': '0.25'" in launch_text
    assert "'behavior_server'" not in launch_text
    assert "'waypoint_follower'" not in launch_text


def test_lidar_localization_does_not_claim_nav2_map_topic_as_pointcloud():
    localization_source = (
        SRC_ROOT / 'traymover_robot_slam' / 'lidar_localization_ros2' /
        'src' / 'lidar_localization_component.cpp'
    ).read_text(encoding='utf-8')

    assert '"ndt_map"' in localization_source
    assert 'create_subscription<sensor_msgs::msg::PointCloud2>(\n    "map"' not in localization_source


def test_option14_isolates_cmu_path_from_localization_diagnostics():
    launch_text = (PKG_ROOT / 'launch' / 'traymover_3d_nav.launch.py').read_text(
        encoding='utf-8'
    )
    localization_launch = (
        PKG_ROOT / 'launch' / 'lidar_localization.launch.py'
    ).read_text(encoding='utf-8')

    assert "'local_path_topic', default_value='/local_planner_path'" in launch_text
    assert "'fastlio_path_topic', default_value='/fastlio_path'" in launch_text
    assert "'localization_path_topic', default_value='/pcl_path'" in launch_text
    assert "'waypoint_target_frame', default_value='camera_init'" in launch_text
    assert "'target_frame': waypoint_target_frame" in launch_text
    assert "'publish_scan': 'false'" in launch_text
    assert 'collision_monitor.launch.py' not in launch_text
    assert "'out_topic': '/cmd_vel'" in launch_text
    assert '/cmd_vel_nav' not in launch_text
    assert "'fastlio_path_topic': fastlio_path_topic" in launch_text
    assert "'localization_path_topic': localization_path_topic" in launch_text
    assert "'max_map_odom_update_translation': '0.30'" in launch_text
    assert "'max_map_odom_update_rotation': '0.12'" in launch_text
    assert "('/path', local_path_topic)" in launch_text
    assert launch_text.count("('/path', local_path_topic)") == 2
    assert 'static_base_to_vehicle' in launch_text
    assert "'base_link', 'vehicle'" in launch_text

    assert "'fastlio_path_topic', default_value='/path'" in localization_launch
    assert "'localization_path_topic', default_value='/path'" in localization_launch
    assert "'max_map_odom_update_translation', default_value='-1.0'" in localization_launch
    assert "'max_map_odom_update_rotation', default_value='-1.0'" in localization_launch
    assert "('/path', fastlio_path_topic)" in localization_launch
    assert "('/path', localization_path_topic)" in localization_launch


def test_option14_local_planner_is_simple_goal_following_first():
    params = load_unique_yaml(PKG_ROOT / 'config' / 'local_planner.yaml')
    planner = params['localPlanner']['ros__parameters']

    assert planner['autonomyMode'] is True
    assert planner['checkObstacle'] is False
    assert planner['checkRotObstacle'] is False


def test_option10_behavior_tree_has_no_recovery_actions():
    bt_paths = [
        PKG_ROOT / 'behavior_trees' / 'simple_navigate_to_pose.xml',
        PKG_ROOT / 'behavior_trees' / 'simple_navigate_through_poses.xml',
    ]

    for bt_path in bt_paths:
        bt_text = bt_path.read_text(encoding='utf-8')
        assert '<FollowPath ' in bt_text
        for recovery_action in ('Spin', 'Wait', 'BackUp', 'ClearEntireCostmap'):
            assert recovery_action not in bt_text

    assert '<ComputePathToPose ' in bt_paths[0].read_text(encoding='utf-8')
    assert '<ComputePathThroughPoses ' in bt_paths[1].read_text(encoding='utf-8')


def test_localization_yaml_has_unique_keys_and_far_wall_range():
    params = load_unique_yaml(PKG_ROOT / 'config' / 'localization.yaml')
    loc_params = params['/**']['ros__parameters']

    assert 100.0 <= loc_params['scan_max_range'] <= 120.0
    assert loc_params['far_point_boost_min_range'] <= 15.0
    assert loc_params['far_point_boost_factor'] == 2
    assert loc_params['voxel_leaf_size'] >= 0.12
    assert 0.9 <= loc_params['score_threshold'] < 2.0
    assert loc_params['ndt_align_interval_s'] >= 2.0


def test_nav2_tolerates_short_tf_jitter_without_recoveries():
    params = load_unique_yaml(PKG_ROOT / 'config' / 'nav2_params.yaml')
    bt_params = params['bt_navigator']['ros__parameters']
    controller_params = params['controller_server']['ros__parameters']
    follow_path = controller_params['FollowPath']
    progress_checker = controller_params['progress_checker']
    local_costmap = params['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = params['global_costmap']['global_costmap']['ros__parameters']

    assert bt_params['transform_tolerance'] >= 0.6
    assert controller_params['failure_tolerance'] >= 2.0
    assert controller_params['general_goal_checker']['yaw_goal_tolerance'] <= 0.30
    assert follow_path['desired_linear_vel'] <= 0.14
    assert follow_path['use_rotate_to_heading'] is True
    assert follow_path['allow_reversing'] is False
    assert follow_path['lookahead_dist'] >= 1.3
    assert follow_path['min_lookahead_dist'] >= 1.0
    assert follow_path['max_lookahead_dist'] >= 2.5
    assert follow_path['lookahead_time'] >= 2.5
    assert follow_path['regulated_linear_scaling_min_radius'] >= 3.0
    assert follow_path['regulated_linear_scaling_min_speed'] <= 0.04
    assert follow_path['transform_tolerance'] >= 0.6
    assert progress_checker['plugin'] == 'traymover_robot_nav::YawProgressChecker'
    assert progress_checker['required_movement_angle'] >= 0.15
    assert local_costmap['transform_tolerance'] >= 0.6
    assert global_costmap['transform_tolerance'] >= 0.6


def test_option15_fast_nav_configs_only_change_requested_speed_values():
    slow = load_unique_yaml(PKG_ROOT / 'config' / 'nav2_params.yaml')
    linear_2x = load_unique_yaml(PKG_ROOT / 'config' / 'nav2_params_linear_2x.yaml')
    linear_2_5x_turn_2x = load_unique_yaml(
        PKG_ROOT / 'config' / 'nav2_params_linear_2_5x_turn_2x.yaml'
    )

    speed_keys = (
        'desired_linear_vel',
        'min_approach_linear_velocity',
        'regulated_linear_scaling_min_speed',
        'rotate_to_heading_angular_vel',
    )

    def follow_path(params):
        return params['controller_server']['ros__parameters']['FollowPath']

    def without_speed_values(params):
        clone = copy.deepcopy(params)
        clone_follow_path = follow_path(clone)
        for key in speed_keys:
            clone_follow_path[key] = '<speed-mode-value>'
        return clone

    assert without_speed_values(linear_2x) == without_speed_values(slow)
    assert without_speed_values(linear_2_5x_turn_2x) == without_speed_values(slow)

    slow_follow_path = follow_path(slow)
    linear_2x_follow_path = follow_path(linear_2x)
    linear_2_5x_turn_2x_follow_path = follow_path(linear_2_5x_turn_2x)

    assert_close(
        linear_2x_follow_path['desired_linear_vel'],
        slow_follow_path['desired_linear_vel'] * 2.0,
    )
    assert_close(
        linear_2x_follow_path['min_approach_linear_velocity'],
        slow_follow_path['min_approach_linear_velocity'] * 2.0,
    )
    assert_close(
        linear_2x_follow_path['regulated_linear_scaling_min_speed'],
        slow_follow_path['regulated_linear_scaling_min_speed'] * 2.0,
    )
    assert_close(
        linear_2x_follow_path['rotate_to_heading_angular_vel'],
        slow_follow_path['rotate_to_heading_angular_vel'],
    )

    assert_close(
        linear_2_5x_turn_2x_follow_path['desired_linear_vel'],
        slow_follow_path['desired_linear_vel'] * 2.5,
    )
    assert_close(
        linear_2_5x_turn_2x_follow_path['min_approach_linear_velocity'],
        slow_follow_path['min_approach_linear_velocity'] * 2.5,
    )
    assert_close(
        linear_2_5x_turn_2x_follow_path['regulated_linear_scaling_min_speed'],
        slow_follow_path['regulated_linear_scaling_min_speed'] * 2.5,
    )
    assert_close(
        linear_2_5x_turn_2x_follow_path['rotate_to_heading_angular_vel'],
        slow_follow_path['rotate_to_heading_angular_vel'] * 2.0,
    )


def test_option15_launcher_uses_dedicated_speed_configs_without_changing_option10_case():
    launcher_text = (WORKSPACE_ROOT / 'scripts' / 'traymover.sh').read_text(
        encoding='utf-8'
    )

    assert '10) action_start_nav ;;' in launcher_text
    assert '15) action_start_nav_speed_modes ;;' in launcher_text
    assert '15) Start navigation with speed mode' in launcher_text
    assert 'nav2_params_linear_2x.yaml' in launcher_text
    assert 'nav2_params_linear_2_5x_turn_2x.yaml' in launcher_text
    assert "params_file:='${nav_params_file}'" in launcher_text


def test_option10_progress_checker_plugin_exports_yaw_progress():
    plugin_text = (PKG_ROOT / 'plugins.xml').read_text(encoding='utf-8')
    source_text = (PKG_ROOT / 'src' / 'yaw_progress_checker.cpp').read_text(
        encoding='utf-8'
    )

    assert 'traymover_robot_nav::YawProgressChecker' in plugin_text
    assert 'base_class_type="nav2_core::ProgressChecker"' in plugin_text
    assert 'required_movement_angle' in source_text
    assert 'rotated_enough(pose)' in source_text
    assert 'moved_enough(pose)' in source_text
    assert 'tf2::getYaw' not in source_text


def test_option10_progress_checker_plugin_has_no_runtime_undefined_symbols():
    plugin_lib = (
        WORKSPACE_ROOT / 'build' / 'traymover_robot_nav' /
        'libtraymover_robot_nav_progress_checker.so'
    )

    assert plugin_lib.exists(), (
        'build traymover_robot_nav before running this runtime symbol check'
    )

    result = subprocess.run(
        ['ldd', '-r', str(plugin_lib)],
        capture_output=True,
        text=True,
        check=False,
    )
    output = result.stdout + result.stderr

    assert result.returncode == 0, output
    assert 'undefined symbol' not in output
    assert 'not found' not in output


def test_rviz_shows_map_rainbow_and_ndt_alignment_overlay():
    rviz_text = (PKG_ROOT / 'rviz' / 'traymover_nav.rviz').read_text(
        encoding='utf-8'
    )

    assert 'Name: MapPCD' in rviz_text
    assert 'Value: /initial_map' in rviz_text
    assert 'Color Transformer: AxisColor' in rviz_text
    assert 'Use rainbow: true' in rviz_text
    assert 'Name: NDTAlignedScan' in rviz_text
    assert 'Value: /ndt_aligned_scan' in rviz_text
    assert 'Color: 255; 220; 0' in rviz_text
    assert 'Color Transformer: FlatColor' in rviz_text


def test_option14_rviz_exposes_3d_navigation_state():
    rviz_text = (PKG_ROOT / 'rviz' / 'traymover_3d_nav.rviz').read_text(
        encoding='utf-8'
    )

    for expected in (
        'Fixed Frame: map',
        'Name: MapPCD',
        'Value: /initial_map',
        'Name: RegisteredScan',
        'Value: /cloud_registered',
        'Name: NDTAlignedScan',
        'Value: /ndt_aligned_scan',
        'Name: TerrainMap',
        'Value: /terrain_map',
        'Name: LocalPath',
        'Value: /local_planner_path',
        'Name: FastLIOPath',
        'Value: /fastlio_path',
        'Name: PCLLocalizationPath',
        'Value: /pcl_path',
        'Name: FreePaths',
        'Value: /free_paths',
    ):
        assert expected in rviz_text


def test_initialpose_is_only_ndt_guess_until_valid_alignment():
    source = (
        SRC_ROOT
        / 'traymover_robot_slam'
        / 'lidar_localization_ros2'
        / 'src'
        / 'lidar_localization_component.cpp'
    ).read_text(encoding='utf-8')
    header = (
        SRC_ROOT
        / 'traymover_robot_slam'
        / 'lidar_localization_ros2'
        / 'include'
        / 'lidar_localization'
        / 'lidar_localization_component.hpp'
    ).read_text(encoding='utf-8')

    assert 'ndt_bootstrap_pending_' in header
    assert 'NDT-validated map->odom only' in header
    assert 'Initialpose accepted as NDT guess only' in source
    assert 'Seeded map' not in source
    assert 'from initialpose' not in source
    assert 'last_align_stamp_s_ = -1.0e9' in source
    assert 'last_align_stamp_s_ = now_s;' in source
    assert source.index('last_align_stamp_s_ = now_s;') < source.index(
        'registration_->align(*output_cloud, init_guess);'
    )
    assert 'waiting for valid NDT before publishing map->odom' in source
    assert 'if (!corrent_pose_with_cov_stamped_ptr_ || !have_good_pose_) {return;}' in source
    assert 'have_good_pose_ && !ndt_bootstrap_pending_' in source
    assert 'tf_msg.header.stamp = now();' in source
    assert 'ndt_aligned_scan_pub_' in header
    assert '"ndt_aligned_scan"' in source
    assert 'registration_->align(*output_cloud, init_guess);' in source
    assert 'ndt_aligned_scan_pub_->publish(aligned_scan_msg);' in source
    assert 'max_map_odom_update_translation' in source
    assert 'max_map_odom_update_rotation' in source
    assert 'NDT map→odom jump' in source
    assert source.index('NDT map→odom jump') < source.index(
        'ndt_aligned_scan_pub_->publish(aligned_scan_msg);'
    )
