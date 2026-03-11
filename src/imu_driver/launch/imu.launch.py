from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    dev_name_arg = DeclareLaunchArgument(
        'dev_name',
        default_value='/dev/ttyTHS2',
        description='IMU serial device name'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate'
    )
    
    imu_node = Node(
        package='imu_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'dev_name': LaunchConfiguration('dev_name'),
            'baudrate': LaunchConfiguration('baudrate'),
            'pitch_range': [-30.0, 40.0],
            'roll_range': [-40.0, 40.0],
            'frame_id': 'imu_link',
        }],
    )
    
    return LaunchDescription([
        dev_name_arg,
        baudrate_arg,
        imu_node,
    ])
