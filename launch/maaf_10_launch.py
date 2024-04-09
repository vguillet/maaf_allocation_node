from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchContext


def generate_launch_description():
    nodes = []

    # > Declare scenario config launch argument
    nodes.append(
        DeclareLaunchArgument(
            'scenario_id',
            default_value='free_scenario',
            description='The scenario configuration to use for the simulation'
        )
    )

    for i in range(10):
         nodes.append(
            Node(
                namespace=f"Turtle_{i+1}",
                package="maaf_allocation_node",
                executable="maaf_allocation_node",
                output="screen",
                parameters=[
                    {
                        "id": f"Turtle_{i+1}",
                        "name": f"Turtle_{i+1}",
                        "scenario_id": LaunchConfiguration("scenario_id"),
                    }
                ]
            )
        )

    return LaunchDescription(nodes)
