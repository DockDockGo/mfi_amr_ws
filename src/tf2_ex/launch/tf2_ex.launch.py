from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace= "turtle1",
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),
        Node(
            namespace= "turtle2",
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),
        Node(
            package='tf2_ex',
            executable='tf2_practice_node',
            name='tf_broadcaster',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['-0.5', '-0.5', '0.0', '0.0', '0.0', '0.0', 'turtle1', 'turtle2']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-0.5', '-0.5', '1.0', '0.0', '0.0', '0.0', 'turtle1', 'drone']
        ),
    ])
    