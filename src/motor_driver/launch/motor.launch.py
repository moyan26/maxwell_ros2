from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mock_mode_arg = DeclareLaunchArgument(
        'mock_mode',
        default_value='true',
        description='Enable mock mode for testing without hardware'
    )

    motor_node = Node(
        package='motor_driver',
        executable='motor_node',
        name='motor_node',
        output='screen',
        parameters=[{
            'mock_mode': LaunchConfiguration('mock_mode'),
            'device_name': '/dev/ttyUSB0',
            'baudrate': 3000000,
            'control_rate': 100.0,
            'max_speed': 360.0,
            'joint_names': [
                'jhead1', 'jhead2',
                'jlshoulder1', 'jlelbow',
                'jrshoulder1', 'jrelbow',
                'jlhip3', 'jlhip2', 'jlhip1', 'jlknee', 'jlankle2', 'jlankle1',
                'jrhip3', 'jrhip2', 'jrhip1', 'jrknee', 'jrankle2', 'jrankle1'
            ]
        }]
    )

    return LaunchDescription([
        mock_mode_arg,
        motor_node
    ])
