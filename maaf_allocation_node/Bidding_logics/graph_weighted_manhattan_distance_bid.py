
##################################################################################################################

# Built-in/Generic Imports
import random

# Libs
from networkx import astar_path

# Own modules
from maaf_allocation_node.fleet_dataclasses import Agent, Fleet
from maaf_allocation_node.task_dataclasses import Task

##################################################################################################################


def graph_weighted_manhattan_distance_bid(task: Task, agent_lst: list[Agent], env, logger) -> list[dict]:
    """
    Calculate the weighted Manhattan distance between a task and a list of agents.

    :param task: The task to calculate the distance to.
    :param agent_lst: The list of agents to calculate the distance from.

    :return: A list of dictionaries containing the agent ID and the weighted Manhattan distance to the task.
    """

    if env is None:
        # -> Return 0 bids for all agents as the environment is not available
        # logger.info("WARNING: Environment not available")
        return [{"agent_id": agent.id, "bid": 0} for agent in agent_lst]

    # logger.info(f"Calculating weighted Manhattan distance for task {task.id}")

    bids = []

    # -> Check the agents skillset against the task instructions
    valid_agents = []

    for agent in agent_lst:
        if task.type in agent.skillset:
            valid_agents.append(agent)
        else:
            bids.append({
                "agent_id": agent.id,
                "bid": 0
            })

    # -> Calculate the weighted Manhattan distance for all valid agents
    for agent in valid_agents:
        # -> Agent node
        agent_node = (agent.state.x, agent.state.y)

        # -> Task node
        task_node = (task.instructions["x"], task.instructions["y"])

        # -> Find the weigthed Manhattan distance between the agent and the task
        path = astar_path(env["graph"], agent_node, task_node, weight="weight")

        # -> Calculate the total distance
        total_distance = 0
        for i in range(len(path) - 1):
            total_distance += env["graph"][path[i]][path[i + 1]]["weight"] + random.uniform(0, 0.1)

        # -> Add bid to the list
        bids.append({
            "agent_id": agent.id,
            "bid": total_distance
        })

    # logger.info(f"Bids calculated: {bids}")

    return bids
