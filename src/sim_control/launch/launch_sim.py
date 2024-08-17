from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    control_node = Node(
        package='sim_control',
        executable='sim_control',
        name='sim_control_node',
        parameters=[
            {'CONTROL_TOPIC': '/control'},
            {'STOP_KEY': 's'}
        ]
    )

    ld.add_action(control_node)

    return ld
