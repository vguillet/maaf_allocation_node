
##################################################################################################################
"""
This module contains the graph_weighted_manhattan_distance_bid function, which calculates the weighted Manhattan distance between a task and a list of agents.

RUN CONTEXT: Simulation
"""

# Built-in/Generic Imports
import random

# Libs
from networkx import astar_path, shortest_path

# Own modules
from maaf_tools.datastructures.task.Task import Task
from maaf_tools.datastructures.task.TaskLog import TaskLog

from maaf_tools.datastructures.agent.Agent import Agent
from maaf_tools.datastructures.agent.Fleet import Fleet
from maaf_tools.datastructures.agent.AgentState import AgentState


##################################################################################################################


def graph_weighted_manhattan_distance_bid(
        # Tasks
        task: Task,
        tasklog: TaskLog,

        # Agents
        agent_lst: list[Agent], 
        fleet: Fleet,

        environment,
        logger,

        *args,
        **kwargs
    ) -> list[dict]:
    """
    Calculate the weighted Manhattan distance between a task and a list of agents.

    :param task: The task to calculate the distance to.
    :param tasklog: The task log to store the path for the current bid.

    :param agent_lst: The list of agents to calculate the distance from.
    :param fleet: The fleet of agents to calculate the distance from.

    :param environment: The environment graph to calculate the distance in.
    :param logger: The logger to log messages to.

    :return: A list of dictionaries containing the agent ID and the weighted Manhattan distance to the task.
    """

    bids = []

    # -> Check the agents skillset against the task instructions
    valid_agents = []

    for agent in agent_lst:
        if agent.has_skill(task.type):
            valid_agents.append(agent)
        else:
            bids.append({
                "agent_id": agent.id,
                "bid": 0,
                "allocation": 0
            })

    # -> Calculate the weighted Manhattan distance for all valid agents
    for agent in valid_agents:
        # -> Agent node
        agent_node = (agent.state.x, agent.state.y)

        # -> Task node
        task_node = (task.instructions["x"], task.instructions["y"])

        # -> If agent on a node in the path for the current bid, reuse and trim the path
        current_path = tasklog.get_sequence_path(
            node_sequence=["agent", task.id],
            requirement=None
        )

        if current_path:
            if agent_node in current_path:
                path = current_path[current_path.index(agent_node):]
                compute_path = False

            else:
                compute_path = True
        else:
            compute_path = True

        if compute_path:
            # -> Find the weigthed Manhattan distance between the agent and the task
            path = shortest_path(environment["graph"], agent_node, task_node)
            # path = astar_path(environment["graph"], agent_node, task_node, weight="weight")

        # > Store path to agent task log
        tasklog.add_path(
            source_node="agent",
            target_node=task.id,
            path={
                "id": f"{agent.id}_{task.id}",
                "path": path,
                "requirements": ["ground"]
            },
            two_way=False,
            selection="latest"
        )

        # -> Calculate the total distance
        total_distance = random.uniform(0.0000000000001, 0.000000001)    # Start with random tiny number to avoid division by zero and ties in allocation
        # for i in range(len(path) - 1):
        #     total_distance += environment["graph"][path[i]][path[i + 1]]["weight"]

        total_distance += len(path) -1

        # -> Add bid to the list
        bids.append({
            "agent_id": agent.id,
            "bid": 1/total_distance,
            "allocation": 0
        })

    return bids
