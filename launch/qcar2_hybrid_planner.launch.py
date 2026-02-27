import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ─── Get Package Directories ───
    teleop_dir = get_package_share_directory('qcar2_teleop')
    planner_dir = get_package_share_directory('qcar2_planner')
    mixer_dir = get_package_share_directory('qcar2_mixer')
    qcar2_nodes_dir = get_package_share_directory('qcar2_nodes')
    detections_dir = get_package_share_directory('qcar2_object_detections')

    # ─── Expose YAML Arguments at Top-Level ───
    # This allows overriding the configuration from this single launch file
    nav_params_file_arg = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(qcar2_nodes_dir, 'config', 'qcar2_slam_and_nav_virtual.yaml'),
        description='Path to the Nav2 parameters YAML file for mamalaunch'
    )
    
    tracking_config_arg = DeclareLaunchArgument(
        'tracking_config',
        default_value=os.path.join(teleop_dir, 'config', 'qcar2_tracking_params.yaml'),
        description='Path to tracking/lane configuration for mamalaunch'
    )
    
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml_path',
        default_value=os.path.join(planner_dir, 'config', 'mapV4uncertainty.yaml'),
        description='Path to the map YAML file for qcar2_planner_server'
    )

    detections_config_file_arg = DeclareLaunchArgument(
        'detections_config_file',
        default_value=os.path.join(detections_dir, 'config', 'qcar2_object_detections_params.yaml'),
        description='Path to the parameters YAML file for object detections'
    )

    mixer_params_file_arg = DeclareLaunchArgument(
        'mixer_params_file',
        default_value=os.path.join(mixer_dir, 'config', 'qcar2_mixer_params.yaml'),
        description='Path to the qcar2_mixer parameters YAML file'
    )
    
    output_motor_topic_arg = DeclareLaunchArgument(
        'output_motor_topic',
        default_value='/qcar2_motor_speed_cmd',
        description='Final motor command topic to publish hardware commands'
    )
    
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic', default_value='/scan'
    )
    lidar_obstacle_distance_arg = DeclareLaunchArgument(
        'lidar_obstacle_distance', default_value='0.3'
    )
    zebra_speed_reduction_arg = DeclareLaunchArgument(
        'zebra_speed_reduction', default_value='0.5'
    )
    person_wait_timeout_arg = DeclareLaunchArgument(
        'person_wait_timeout', default_value='5.0'
    )
    stop_sign_stop_time_arg = DeclareLaunchArgument(
        'stop_sign_stop_time', default_value='3.0'
    )
    stop_sign_forward_time_arg = DeclareLaunchArgument(
        'stop_sign_forward_time', default_value='2.0'
    )
    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz', default_value='30.0'
    )

    # ─── Launch Configurations ───
    nav_params_file = LaunchConfiguration('nav_params_file')
    tracking_config = LaunchConfiguration('tracking_config')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    detections_config_file = LaunchConfiguration('detections_config_file')
    mixer_params_file = LaunchConfiguration('mixer_params_file')
    output_motor_topic = LaunchConfiguration('output_motor_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')
    lidar_obstacle_distance = LaunchConfiguration('lidar_obstacle_distance')
    zebra_speed_reduction = LaunchConfiguration('zebra_speed_reduction')
    person_wait_timeout = LaunchConfiguration('person_wait_timeout')
    stop_sign_stop_time = LaunchConfiguration('stop_sign_stop_time')
    stop_sign_forward_time = LaunchConfiguration('stop_sign_forward_time')
    rate_hz = LaunchConfiguration('rate_hz')

    # ─── 1) Include mamalaunch (from qcar2_teleop) ───
    mamalaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_dir, 'launch', 'mamalaunch.py')
        ),
        launch_arguments={
            'params_file': nav_params_file,
            'tracking_config': tracking_config,
            'enable_bridge': 'false',
        }.items()
    )

    # ─── 2) Include hybrid_switch_launch (from qcar2_teleop) ───
    hybrid_switch_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_dir, 'launch', 'hybrid_switch_launch.py')
        ),
        launch_arguments={
            'motor_cmd_topic': '/hybrid/motor',
        }.items()
    )

    # ─── 3) Include qcar2_planner_server (from qcar2_planner) ───
    qcar2_planner_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_dir, 'launch', 'qcar2_planner_server.launch.py')
        ),
        launch_arguments={
            'map_yaml_path': map_yaml_path,
        }.items()
    )

    # ─── 4) Object Detection ───
    qcar2_detections = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detections_dir, 'launch', 'qcar2_detections.launch.py')
        ),
        launch_arguments={
            'params_file': detections_config_file,
        }.items(),
    )

    return LaunchDescription([
        # Arguments
        nav_params_file_arg,
        tracking_config_arg,
        map_yaml_arg,
        detections_config_file_arg,
        mixer_params_file_arg,
        output_motor_topic_arg,
        lidar_topic_arg,
        lidar_obstacle_distance_arg,
        zebra_speed_reduction_arg,
        person_wait_timeout_arg,
        stop_sign_stop_time_arg,
        stop_sign_forward_time_arg,
        rate_hz_arg,
        
        # Launches
        mamalaunch,
        hybrid_switch_launch,
        qcar2_planner_server,
        qcar2_detections,
    
        # ─────────────────────────────────────────────────────────
        # MIXER NODE
        # ─────────────────────────────────────────────────────────
        Node(
            package='qcar2_mixer',
            executable='qcar2_mixer_node',
            name='qcar2_mixer',
            output='screen',
            parameters=[
                mixer_params_file,
                {
                    'output_motor_topic': output_motor_topic,
                    'lidar_topic': lidar_topic,
                    'lidar_obstacle_distance': lidar_obstacle_distance,
                    'zebra_speed_reduction_factor': zebra_speed_reduction,
                    'person_wait_timeout': person_wait_timeout,
                    'stop_sign_stop_time': stop_sign_stop_time,
                    'stop_sign_forward_time': stop_sign_forward_time,
                    'rate_hz': rate_hz,
                }
            ],
            emulate_tty=True,
        ),
    ])
