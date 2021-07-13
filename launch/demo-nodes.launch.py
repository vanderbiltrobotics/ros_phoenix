from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launches talons for basic 4-wheel drive."""
    return LaunchDescription(
        [
            Node(
                package="ros_phoenix",
                executable="talon_node",
                name="front_left",
                parameters=[{"id": 15, "invert": True}],
            ),
            Node(
                package="ros_phoenix",
                executable="talon_node",
                name="front_right",
                parameters=[{"id": 18, "invert": False}],
            ),
            Node(
                package="ros_phoenix",
                executable="talon_node",
                name="back_left",
                parameters=[{"id": 10, "invert": True}],
            ),
            Node(
                package="ros_phoenix",
                executable="talon_node",
                name="back_right",
                parameters=[{"id": 19, "invert": False}],
            ),
        ]
    )
