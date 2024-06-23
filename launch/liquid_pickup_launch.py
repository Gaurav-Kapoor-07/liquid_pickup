from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='liquid_pickup',
            executable='liquid_pickup_node',
            name='bt_node',
            namespace='summit',
            parameters=[{"bt_xml": "liquid_pickup.xml"}, {"use_sim_time": True}],
            output='screen',
            remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static')],
            emulate_tty=True,
        ),

        # for testing
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_liquid_frame',
            namespace='summit',
            parameters=[{'use_sim_time': True}],
            arguments = ['--x', '1.0', '--y', '0.01', '--z', '1.0', '--roll', '0.0', '--pitch', '1.57', '--yaw', '0.0', '--frame-id', 'base_footprint', '--child-frame-id', 'liquid'],
            remappings=[('/tf_static', '/summit/tf_static')],
        ),
    ])
