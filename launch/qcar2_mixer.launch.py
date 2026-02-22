from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Optional params file for qcar2_mixer node (structure placeholder).'
        ),
        DeclareLaunchArgument(
            'output_motor_topic',
            default_value='/qcar2_motor_speed_cmd',
            description='Final motor command topic for QCar2 hardware interface.'
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/scan',
            description='Lidar topic consumed by future mixer logic.'
        ),
        DeclareLaunchArgument(
            'input_motor_cmd_topic',
            default_value='/lane/motor_cmd',
            description='Primary MotorCommands input topic for future mixer logic.'
        ),
        DeclareLaunchArgument(
            'person_detection_topic',
            default_value='/detections/person',
            description='Person detection topic from qcar2_object_detections.'
        ),
        DeclareLaunchArgument(
            'stop_sign_topic',
            default_value='/detections/stop_sign',
            description='Stop sign detection topic from qcar2_object_detections.'
        ),
        DeclareLaunchArgument(
            'traffic_light_topic',
            default_value='/detections/traffic_light',
            description='Traffic light detection topic from qcar2_object_detections.'
        ),
        DeclareLaunchArgument(
            'zebra_crossing_topic',
            default_value='/detections/zebra_crossing',
            description='Zebra crossing detection topic from qcar2_object_detections.'
        ),
    ])
