#!/usr/bin/env python3
"""
QCar2 Mixer Launch File

Brings up the complete perception and control pipeline:
  1. Lane mapping: cartographer + mapping nodes (qcar2_LaneMapping-ACC)
  2. Path planning: planner nodes (qcar2_planner)
  3. Object detection: YoloV8 + filtering (qcar2_object_detections)
  4. Tracking: lane following + hybrid controller (qcar2_teleop)
  5. Mixer: final safety-aware command stage (qcar2_mixer)

Usage:
  ros2 launch qcar2_mixer qcar2_mixer.launch.py
  ros2 launch qcar2_mixer qcar2_mixer.launch.py enable_bridge:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for complete QCar2 pipeline with mixer."""
    
    # Get package directories
    mixer_dir = get_package_share_directory('qcar2_mixer')
    lane_mapping_dir = get_package_share_directory('lane_mapping_acc')
    planner_dir = get_package_share_directory('qcar2_planner')
    detections_dir = get_package_share_directory('qcar2_object_detections')
    tracking_dir = get_package_share_directory('qcar2_teleop')
    qcar2_nodes_dir = get_package_share_directory('qcar2_nodes')

    # Config files
    tracking_config_file = os.path.join(tracking_dir, 'config', 'qcar2_tracking_params.yaml')
    tracking_pid_file = os.path.join(tracking_dir, 'config', 'pid_tunedv3.yaml')
    lane_mapping_segmentation_config = os.path.join(lane_mapping_dir, 'config', 'segmentation_params.yaml')
    lane_mapping_nvblox_config = os.path.join(lane_mapping_dir, 'config', 'nvblox_static.yaml')
    planner_config_file = os.path.join(planner_dir, 'config', 'params.yaml')
    detections_config_file = os.path.join(detections_dir, 'config', 'detections_params.yaml')
    nav2_params_file = os.path.join(qcar2_nodes_dir, 'config', 'qcar2_slam_and_nav_virtual.yaml')

    # Launch arguments
    enable_bridge_arg = LaunchConfiguration('enable_bridge')
    # Parameters file
    mixer_params_file = os.path.join(mixer_dir, 'config', 'qcar2_mixer_params.yaml')

    return LaunchDescription([
        # ─────────────────────────────────────────────────────────
        # DECLARE ARGUMENTS
        # ─────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'enable_bridge',
            default_value='true',
            description='Enable hybrid bridge for Nav2 + lane fusion.'
        ),

        # ─────────────────────────────────────────────────────────
        # INCLUDED LAUNCHES
        # ─────────────────────────────────────────────────────────

        # 1. Lane Mapping (cartographer + occupancy grid)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lane_mapping_dir, 'launch', 'nvblox_road_mapping.launch.py')
            ),
            launch_arguments={
                'segmentation_config': lane_mapping_segmentation_config,
                'nvblox_config': lane_mapping_nvblox_config,
                'use_sim_time': 'false',
            }.items(),
        ),

        # 2. Planning
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planner_dir, 'launch', 'qcar2_mapping_planner.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
            }.items(),
        ),

        # 3. Object Detection
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(detections_dir, 'launch', 'qcar2_detections.launch.py')
            ),
            launch_arguments={
                'params_file': detections_config_file,
            }.items(),
        ),

        # 4. Tracking (lane following + optional hybrid controller)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tracking_dir, 'launch', 'qcar2_hybrid_nav_launch.py')
            ),
            launch_arguments={
                'enable_bridge': enable_bridge_arg,
                'config_file': tracking_config_file,
                'config_file_pid': tracking_pid_file,
                'params_file': nav2_params_file,
            }.items(),
        ),

        # ─────────────────────────────────────────────────────────
        # MIXER NODE
        # ─────────────────────────────────────────────────────────
        Node(
            package='qcar2_mixer',
            executable='qcar2_mixer_node',
            name='qcar2_mixer',
            output='screen',
            parameters=[mixer_params_file],
            emulate_tty=True,
        ),
    ])
