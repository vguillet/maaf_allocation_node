
##################################################################################################################

# Built-in/Generic Imports
import random
from pprint import pformat

# Libs
from networkx import astar_path, shortest_path

# Own modules
from maaf_allocation_node.fleet_dataclasses import Agent, Fleet
from maaf_allocation_node.task_dataclasses import Task
from maaf_allocation_node.node_config import *

##################################################################################################################


def anticipated_action_task_interceding_agent(
        task: Task,
        agent_lst: list[Agent],
        shared_bids_b,
        environment,
        logger,
        *args,
        **kwargs
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

    bids = []

    # -> If task is not a GOTO task, only consider self agent
    if task.type != "GOTO":
        if kwargs["self_agent"].has_skill(task.type):
            valid_agents = [kwargs["self_agent"]]
        else:
            bids.append({
                "agent_id": kwargs["self_agent"].id,
                "bid": 0,
                "allocation": 0
            })

            return bids

    else:
        # -> Check the agents skillset against the task instructions
        valid_agents = []

        for agent in agent_lst:
            # -> If the goto leads to a task ACTION_1 and the agent has the ACTION_1 skillset and the agent is in the intercession_targets list
            if task.instructions["ACTION_AT_LOC"] in ["ACTION_1", "NO_TASK"] and agent.has_skill("ACTION_1") and "ACTION_1" in intercession_targets:
                valid_agents.append(agent)

            # -> Elif the goto leads to a task ACTION_2 and the agent has the ACTION_2 skillset and the agent is in the intercession_targets list
            elif task.instructions["ACTION_AT_LOC"] == ["ACTION_2", "NO_TASK"] and agent.has_skill("ACTION_2") and "ACTION_2" in intercession_targets:
                valid_agents.append(agent)

            # -> Else, do not bid
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
        path = shortest_path(environment["graph"], agent_node, task_node)
        # path = astar_path(environment["graph"], agent_node, task_node)
        # path = astar_path(environment["graph"], agent_node, task_node, weight="weight")

        # > Get path x and y
        path_x = [node[0] for node in path]
        path_y = [node[1] for node in path]

        # > Store path to task local
        task.shared["path"] = {
            "x": path_x,
            "y": path_y
        }

        # -> Calculate the total distance
        total_distance = random.uniform(0.00000000001, 0.0000001)    # Start with random tiny number to avoid division by zero and ties in allocation

        # for i in range(len(path) - 1):
        #     total_distance += environment["graph"][path[i]][path[i + 1]]["weight"]

        total_distance += len(path) -1

        if task.type == "GOTO":
            # -> Add bias
            total_distance = total_distance / 10

        # logger.info(f"{task.id}: {agent_node}->{task_node}: {len(path)-1}, bid: {1/total_distance}")

        # -> Add bid to the list
        bids.append({
            "agent_id": agent.id,
            "bid": 1/total_distance,
            "allocation": 0
        })

    # logger.info(f"Task {task.id} has instructions: {task.instructions}")

    # for i, agent in enumerate(agent_lst):
    #     logger.info(f"Agent {agent.id} has skillset: {agent.skillset}")


    return bids
