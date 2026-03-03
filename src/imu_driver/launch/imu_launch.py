from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
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
    
    # 创建 IMU 节点
    imu_node = Node(
        package='imu_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'dev_name': LaunchConfiguration('dev_name'),
            'baudrate': LaunchConfiguration('baudrate'),
            'pitch_range': [-30.0, 40.0],  # 不摔倒的俯仰角范围
            'roll_range': [-40.0, 40.0],   # 不摔倒的横滚角范围
            'frame_id': 'imu_link',
        }],
        # 如果需要 root 权限访问串口，取消下面的注释
        # prefix=['sudo -E'],
    )
    
    return LaunchDescription([
        dev_name_arg,
        baudrate_arg,
        imu_node,
    ])
