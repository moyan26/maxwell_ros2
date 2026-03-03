from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    team_number_arg = DeclareLaunchArgument(
        'team_number',
        default_value='1',
        description='Team number'
    )
    
    player_number_arg = DeclareLaunchArgument(
        'player_number',
        default_value='1',
        description='Player number'
    )
    
    mock_mode_arg = DeclareLaunchArgument(
        'mock_mode',
        default_value='false',
        description='Enable mock mode for testing'
    )

    # Game Controller 接收节点
    game_controller_node = Node(
        package='game_controller_driver',
        executable='game_controller_node',
        name='game_controller_node',
        output='screen',
        parameters=[{
            'team_number': LaunchConfiguration('team_number'),
            'player_number': LaunchConfiguration('player_number'),
            'receive_port': 3838,
            'return_port': 3939,
            'send_alive': True,
            'alive_interval': 1.0
        }]
    )

    # Mock 模式下的模拟发送器
    game_controller_mock = Node(
        package='game_controller_driver',
        executable='game_controller_mock',
        name='game_controller_mock',
        output='screen',
        parameters=[{
            'team_number': LaunchConfiguration('team_number'),
            'state_interval': 10.0  # 状态切换间隔（秒）
        }],
        condition=IfCondition(LaunchConfiguration('mock_mode'))
    )

    return LaunchDescription([
        team_number_arg,
        player_number_arg,
        mock_mode_arg,
        game_controller_node,
        game_controller_mock
    ])
