
##################################################################################################################

# Built-in/Generic Imports
import random

# Libs
from networkx import astar_path

# Own modules
from maaf_allocation_node.fleet_dataclasses import Agent, Fleet
from maaf_allocation_node.task_dataclasses import Task

##################################################################################################################


def anticipated_action_task_interceding_agent(
        task: Task,
        agent_lst: list[Agent],
        shared_bids_b,
        environment,
        logger
    ) -> list[dict]:
    """
    For the provided agent, magnify the bid for the task if the agent has the skillset for the task

    :param task: The task to calculate the distance to.
    :param agent_lst: The list of agents to calculate the distance from.
    :param shared_bids_b: The current bids for the task.
    :param environment: The environment graph to calculate the distance in.
    :param logger: The logger to log messages to.

    :return: A list of dictionaries containing the agent ID and the weighted Manhattan distance to the task.
    """

    if task.type != "GOTO":
        return []

    bids = []

    # -> Check the agents skillset against the task instructions
    valid_agents = []

    for agent in agent_lst:
        # -> If the goto leads to a task ACTION_1 and the agent has the ACTION_1 skillset
        if task.instructions["ACTION_AT_LOC"] == "ACTION_1" and agent.has_skill("ACTION_1"):
            valid_agents.append(agent)

        # -> Elif the goto leads to a task ACTION_2 and the agent has the ACTION_2 skillset
        elif task.instructions["ACTION_AT_LOC"] == "ACTION_2" and agent.has_skill("ACTION_2"):
            valid_agents.append(agent)

        # -> Else, do not intercede (bid 0)
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

        # -> Find the weigthed Manhattan distance between the agent and the task
        path = astar_path(environment["graph"], agent_node, task_node, weight="weight")

        # > Get path x and y
        path_x = [node[0] for node in path]
        path_y = [node[1] for node in path]

        # > Store path to task local
        task.shared["path"] = {
            "x": path_x,
            "y": path_y
        }

        # -> Calculate the total distance
        total_distance = random.uniform(00.00000000001, 0.1)
        for i in range(len(path) - 1):
            total_distance += environment["graph"][path[i]][path[i + 1]]["weight"]

        # -> Add bid to the list
        bids.append({
            "agent_id": agent.id,
            "bid": 1/total_distance + 1,
            "allocation": 0
        })

    return bids
