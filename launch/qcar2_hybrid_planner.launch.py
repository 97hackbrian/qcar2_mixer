import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

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
        default_value=os.path.join(qcar2_nodes_dir, 'config', 'qcar2_slam_and_nav.yaml'),
        description='Path to the Nav2 parameters YAML file for mamalaunch'
    )
    
    # OLD_TRACKING_DISABLED_NAV2_ONLY — tracking_config ya no se usa en modo Nav2-only
    # tracking_config_arg = DeclareLaunchArgument(
    #     'tracking_config',
    #     default_value=os.path.join(teleop_dir, 'config', 'qcar2_tracking_params.yaml'),
    #     description='Path to tracking/lane configuration for mamalaunch'
    # )
    
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml_path',
        default_value=os.path.join(planner_dir, 'config', 'map_ros.yaml'),
        description='Path to the map YAML file for qcar2_planner_server'
    )

    detections_config_file_arg = DeclareLaunchArgument(
        'detections_config_file',
        default_value=os.path.join(detections_dir, 'config', 'detections_params.yaml'),
        description='Path to the parameters YAML file for object detections'
    )

    mixer_params_file_arg = DeclareLaunchArgument(
        'mixer_params_file',
        default_value=os.path.join(mixer_dir, 'config', 'qcar2_mixer_params.yaml'),
        description='Path to the qcar2_mixer parameters YAML file'
    )
    
    # ─── Launch Configurations ───
    nav_params_file = LaunchConfiguration('nav_params_file')
    # OLD_TRACKING_DISABLED_NAV2_ONLY
    # tracking_config = LaunchConfiguration('tracking_config')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    detections_config_file = LaunchConfiguration('detections_config_file')
    mixer_params_file = LaunchConfiguration('mixer_params_file')

    # ─── 1) OLD: mamalaunch (from qcar2_teleop) ───
    # OLD_MAMALAUNCH_DISABLED_NAV2_ONLY — Reemplazado por qcar2_nav2_only_launch.py
    # mamalaunch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(teleop_dir, 'launch', 'mamalaunch.py')
    #     ),
    #     launch_arguments={
    #         'params_file': nav_params_file,
    #         'tracking_config': tracking_config,
    #         'enable_bridge': 'false',
    #     }.items()
    # )

    # ─── 1-NEW) Nav2-Only Launch (from qcar2_teleop) ───
    # Incluye Nav2 bringup + nav2_qcar2_converter.
    # El converter publica MotorCommands a "qcar2_motor_speed_cmd" por defecto.
    # Lo envolvemos en un GroupAction con SetRemap para redirigir esa salida
    # a /nav2/motor_cmd, que es lo que el mixer leerá como input.
    # Cadena: Nav2 → /cmd_vel → converter → /nav2/motor_cmd → mixer → /qcar2_motor_speed_cmd → hardware
    nav2_only_launch = GroupAction([
        SetRemap(src='qcar2_motor_speed_cmd', dst='/nav2/motor_cmd'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_dir, 'launch', 'qcar2_nav2_only_launch.py')
            ),
            launch_arguments={
                'params_file': nav_params_file,
            }.items()
        ),
    ])

    # ─── 2) OLD: hybrid_switch_launch (from qcar2_teleop) ───
    # OLD_HYBRID_SWITCH_DISABLED_NAV2_ONLY — Ya no se necesita hybrid switch en modo Nav2-only
    # hybrid_switch_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(teleop_dir, 'launch', 'hybrid_switch_launch.py')
    #     ),
    #     launch_arguments={
    #         'motor_cmd_topic': '/hybrid/motor',   # OLD_HYBRID_SWITCH_DISABLED_NAV2_ONLY
    #     }.items()
    # )

    # ─── 3) Include qcar2_planner_server (from qcar2_planner) ───
    qcar2_planner_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_dir, 'launch', 'qcar2_overlay_planner.launch.py')
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
        # OLD_TRACKING_DISABLED_NAV2_ONLY
        # tracking_config_arg,
        map_yaml_arg,
        detections_config_file_arg,
        mixer_params_file_arg,
        
        # Launches
        # OLD_MAMALAUNCH_DISABLED_NAV2_ONLY
        # mamalaunch,
        nav2_only_launch,          # ← NUEVO: Nav2-only (reemplaza mamalaunch)
        # OLD_HYBRID_SWITCH_DISABLED_NAV2_ONLY
        # hybrid_switch_launch,
        qcar2_planner_server,
        qcar2_detections,
    
        # ─────────────────────────────────────────────────────────────
        # MIXER NODE
        # ─────────────────────────────────────────────────────────────
        # NOTA: input_motor_topic cambiado de /hybrid/motor a /nav2/motor_cmd
        # porque ahora el converter (dentro de qcar2_nav2_only_launch)
        # publica MotorCommands allí (vía SetRemap).
        Node(
            package='qcar2_mixer',
            executable='qcar2_mixer_node',
            name='qcar2_mixer',
            output='screen',
            parameters=[mixer_params_file],
            emulate_tty=True,
        ),

        # ─────────────────────────────────────────────────────────────
        # LED SEQUENCE NODE
        # ─────────────────────────────────────────────────────────────
        Node(
            package='qcar2_mixer',
            executable='led_sequence_node',
            name='led_sequence_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
