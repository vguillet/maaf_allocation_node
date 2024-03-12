from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchContext


def generate_launch_description():
    nodes = []

    for i in range(3):
         nodes.append(
            Node(
                namespace=f"Turtle_{i}",
                package="maaf_allocation_node",
                executable="maaf_allocation_node",
                output="screen",
                parameters=[
                    {
                        "id": f"Turtle_{i}",
                        "name": f"Turtle_{i}"
                    }
                ]
            )
        )

    return LaunchDescription(nodes)
