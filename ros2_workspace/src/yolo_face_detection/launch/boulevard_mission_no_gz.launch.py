#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument('model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.4'),
        DeclareLaunchArgument('device', default_value='cpu'),
        
        # Camera bridge (Gazebo is already running from PX4)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=[
                '/face_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            remappings=[
                ('/face_camera', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info')
            ],
            output='screen'
        ),
        
        # Face detector (start after bridge has time to initialize)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='yolo_face_detection',
                    executable='mission_face_detector',
                    name='mission_face_detector',
                    output='screen',
                    parameters=[{
                        'model_path': LaunchConfiguration('model_path'),
                        'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                        'device': LaunchConfiguration('device'),
                        'image_topic': '/camera/image_raw',
                        'min_detection_size': 30
                    }]
                )
            ]
        ),
    ])
