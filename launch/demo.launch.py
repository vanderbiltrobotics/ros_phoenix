"""Launch a talker and a listener in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            node_name='talon_container',
            node_namespace='',
            package='ros_phoenix',
            node_executable='phoenix_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ros_phoenix',
                    node_plugin='ros_phoenix::TalonComponent',
                    node_name='front_left',
                    parameters=[{"id": 0}]),
                ComposableNode(
                    package='ros_phoenix',
                    node_plugin='ros_phoenix::TalonComponent',
                    node_name='front_right',
                    parameters=[{"id": 1}]),
                ComposableNode(
                    package='ros_phoenix',
                    node_plugin='ros_phoenix::TalonComponent',
                    node_name='back_left',
                    parameters=[{"id": 2}]),
                ComposableNode(
                    package='ros_phoenix',
                    node_plugin='ros_phoenix::TalonComponent',
                    node_name='back_right',
                    parameters=[{"id": 3}])
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
