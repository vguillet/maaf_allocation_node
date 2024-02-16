from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchContext


def generate_launch_description():
    nodes = []

    # context = LaunchContext()
    #
    #
    # # -> Declare the launch configuration variable
    # nodes.append(
    #     DeclareLaunchArgument(
    #         name="agent_count",
    #         default_value="1",
    #         description="Number of agents"
    #     )
    # )
    #
    # # > Get the value of agent_count from the launch file
    # agent_count = LaunchConfiguration("agent_count")
    #
    # print("!!!!!!!!!!!!!!!", int(agent_count.perform(context)))
    #
    # # > Convert the launch configuration to an integer
    # agent_count_int = int(agent_count.perform(context))

    # -> Add the nodes to the launch description
    for i in range(1):
        nodes.append(
            Node(
                namespace=f"Turtle_{i + 1}",
                package="maaf_allocation_node",
                executable="maaf_allocation_node",
                output="screen",
                parameters=[
                    {
                        "id": f"Turtle_{i + 1}",
                        "name": f"Turtle_{i + 1}"
                    }
                ]
            )
        )

    return LaunchDescription(nodes)
