from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            namespace='',
            executable='game_controller_node',
            name='joy_package'
        ),
        Node(
            package='robot_state_publisher',
            namespace='',
            executable='state_publish',
            name='state_publish'
        ),
        Node(
            package='robot_cmd_subscriber',
            namespace='',
            executable='cmd_subscriber',
            name='cmd_subscriber'
        )
    ])