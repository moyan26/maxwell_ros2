from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    mock_mode_arg = DeclareLaunchArgument(
        'mock_mode',
        default_value='true',
        description='Enable mock mode for testing without hardware'
    )
    
    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='/dev/video0',
        description='Camera device name'
    )
    
    mock_pattern_arg = DeclareLaunchArgument(
        'mock_pattern',
        default_value='color_bar',
        description='Mock pattern: color_bar, noise, or gradient'
    )

    # 创建节点
    camera_node = Node(
        package='camera_driver',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'mock_mode': LaunchConfiguration('mock_mode'),
            'device_name': LaunchConfiguration('device_name'),
            'mock_pattern': LaunchConfiguration('mock_pattern'),
            'image_width': 640,
            'image_height': 480,
            'fps': 30,
            'frame_id': 'camera_link'
        }]
    )

    return LaunchDescription([
        mock_mode_arg,
        device_name_arg,
        mock_pattern_arg,
        camera_node
    ])
