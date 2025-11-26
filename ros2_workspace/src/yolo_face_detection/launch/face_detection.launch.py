from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.5'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('camera_id', default_value='0'),
        
        Node(
            package='yolo_face_detection',
            executable='camera_publisher',
            name='camera_publisher',
            parameters=[{'camera_id': LaunchConfiguration('camera_id')}]
        ),
        
        Node(
            package='yolo_face_detection',
            executable='face_detector',
            name='face_detector',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'device': LaunchConfiguration('device')
            }]
        )
    ])
