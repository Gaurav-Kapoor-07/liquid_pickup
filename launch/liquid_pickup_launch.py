from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='liquid_pickup',
            executable='liquid_pickup_node',
            name='bt_node',
            namespace='summit',
            output='screen',
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
    ])
