from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 传感器层
        Node(package='imu_driver',             executable='imu_node',             name='imu_node',
             parameters=[{'mock_mode': True}]),
        Node(package='camera_driver',          executable='camera_node',          name='camera_node',
             parameters=[{'mock_mode': True}]),
        Node(package='motor_driver',           executable='motor_node',           name='motor_node',
             parameters=[{'mock_mode': True}]),
        # 状态层
        Node(package='maxwell_world',          executable='worldmodel_node',      name='world_node'),
        # 决策层
        Node(package='maxwell_behavior',       executable='fsm_node',             name='fsm_node'),
        # 执行层
        Node(package='maxwell_motion',         executable='walk_node',            name='walk_node'),
    ])
