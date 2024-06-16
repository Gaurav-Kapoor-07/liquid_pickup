from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='liquid_pickup',
            executable='liquid_pickup_node',
            name='bt_node',
            namespace='summit',
            # prefix=['gdbserver localhost:3000'],
            # prefix='gdb -ex "run" --args',
            output='screen',
            # remappings=[('/robot_description', '/summit/robot_description'), ('/robot_description_semantic', '/summit/robot_description_semantic')],
        ),
    ])
