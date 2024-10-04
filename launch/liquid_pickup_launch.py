from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='liquid_pickup',
            executable='liquid_pickup_node',
            name='bt_node',
            namespace='summit',
            # parameters=[{"bt_xml": "liquid_pickup.xml"}, {"use_sim_time": True}],
            parameters=[{"bt_xml": "liquid_pickup.xml"}, {"use_sim_time": False}],
            output='screen',
            remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static')],
            emulate_tty=True,
        ),
    ])
