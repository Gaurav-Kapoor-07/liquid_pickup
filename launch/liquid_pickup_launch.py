from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        Node(
            package='liquid_pickup',
            executable='liquid_pickup_node',
            name='bt_node',
            namespace='summit',
            output='screen',
            remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static')],
        ),
        
        # for testing
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_liquid_frame',
            namespace='summit',
            # parameters=[{'use_sim_time': use_sim_time}],
            arguments = ['--x', '1.0', '--y', '0.0', '--z', '1.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'base_footprint', '--child-frame-id', 'liquid'],
            remappings=[('/tf_static', '/summit/tf_static')],
        ),
    ])
