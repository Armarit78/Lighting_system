from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Central AI Hub Node
        Node(
            package='central_ai_hub',
            executable='central_ai_hub',
            output='screen'
        ),
        # Room Control Node
        Node(
            package='room_control_nodes',
            executable='room_control_node',
            output='screen'
        ),
        # Multi-Sensor Array Node
        Node(
            package='multi_sensor_array',
            executable='multi_sensor_node',
            output='screen'
        ),
        # Sensor Control GUI Node
        Node(
            package='sensor_control_gui',
            executable='sensor_control_gui',
            output='screen'
        ),
    ])

