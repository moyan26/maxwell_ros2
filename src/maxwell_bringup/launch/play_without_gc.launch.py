from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 传感器层
        Node(package='maxwell_sensors', executable='imu_node',       name='imu_node'),
        Node(package='maxwell_sensors', executable='camera_node',    name='camera_node'),
        Node(package='maxwell_sensors', executable='motor_node',     name='motor_node'),

        # 感知层
        Node(package='maxwell_vision',  executable='vision_node',    name='vision_node'),

        # 状态层
        Node(package='maxwell_world',   executable='world_node',     name='world_node'),

        # 决策层
        Node(package='maxwell_behavior',executable='fsm_node',       name='fsm_node'),

        # 执行层
        Node(package='maxwell_motion',  executable='walk_node',      name='walk_node'),
    ])
