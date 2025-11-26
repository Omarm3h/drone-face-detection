#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    home = os.path.expanduser('~')
    world_file = os.path.join(home, 'drone_face_detection/worlds/boulevard.sdf')
    drone_model = os.path.join(home, 'drone_face_detection/models/x500/model.sdf')
    
    return LaunchDescription([
        
        DeclareLaunchArgument('model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.4'),
        DeclareLaunchArgument('device', default_value='cpu'),
        
        # 1. Start Gazebo with world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
            name='gazebo'
        ),
        
        # 2. Wait then spawn drone
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gz', 'model',
                        '--spawn-file', drone_model,
                        '--model-name', 'x500',
                        '--model-pose', '-50', '0', '0.3', '0', '0', '0'
                    ],
                    output='screen',
                    name='spawn_drone'
                )
            ]
        ),
        
        # 3. Camera bridge
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='camera_bridge',
                    arguments=[
                        '/face_camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
                    ],
                    remappings=[
                        ('/face_camera/image', '/camera/image_raw')
                    ],
                    output='screen'
                )
            ]
        ),
        
        # 4. Face detector
        TimerAction(
            period=10.0,
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
