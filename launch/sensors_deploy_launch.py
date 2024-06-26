from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='liquid_pickup',
            executable='liquid_pickup_node',
            name='bt_node',
            namespace='summit',
            parameters=[{"bt_xml": "sensors_deploy.xml"}, {"use_sim_time": True}],
            output='screen',
            emulate_tty=True,
        ),
    ])
