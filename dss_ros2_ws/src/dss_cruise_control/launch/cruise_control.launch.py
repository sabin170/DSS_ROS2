from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dss_cruise_control',
            executable='cruise_control_node',
            name='cruise_control_node',
            output='screen',
            parameters=[
                {'kp': 0.5},
                {'ki': 0.1}, 
                {'kd': 0.05},
                {'max_throttle': 0.8},
                {'max_brake': 0.8},
                {'control_frequency': 10.0}
            ],
            remappings=[
                # 필요시 토픽 리매핑
            ]
        )
    ])
