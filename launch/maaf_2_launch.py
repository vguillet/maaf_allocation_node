from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchContext


def generate_launch_description():
    nodes = []

    for i in range(2):
        ref = "Operator" if i == 0 else f"Turtle_{i}"

        nodes.append(
            Node(
                namespace=ref,
                package="maaf_allocation_node",
                executable="maaf_allocation_node",
                output="screen",
                parameters=[
                    {
                        "id": ref,
                        "name": ref
                    }
                ]
            )
        )

    return LaunchDescription(nodes)
