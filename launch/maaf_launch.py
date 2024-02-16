from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    # -> Declare the launch configuration variable
    nodes.append(
        DeclareLaunchArgument(
            'agent_count',
            default_value='1',
            description='Number of agents'
        )
    )
    
    # > Get the value of agent_count from the launch file
    agent_count = LaunchConfiguration('agent_count')
    
    # > Convert the launch configuration to an integer
    agent_count_int = int(agent_count.perform(None))

    # -> Add the nodes to the launch description
    for i in range(agent_count_int):
        nodes.append(
            Node(
                namespace=f"Turtle_{i + 1}",
                package="maaf_allocation_node",
                executable="maaf_allocation_node",
                output="screen",
                parameters=[
                    {
                        "id": f"Turtle_{i + 1}",
                        "name": f"Turtle_{i + 1}",
                        "class": "Base",
                        "hierarchy_level": 0,
                        "affiliations": [],
                        "specs": {},
                        "skillset": {},
                        "bid_evaluation_function": None,
                        "fleet": [],
                        "task_log": []
                    }
                ]
            )
        )

    return LaunchDescription(nodes)
