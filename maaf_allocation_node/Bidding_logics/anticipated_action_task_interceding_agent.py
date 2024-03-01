
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

    if task.type == "GOTO":
        return []

    bids = []

    # -> Check the agents skillset against the task instructions
    valid_agents = []

    for agent in agent_lst:
        # -> If the task id is even and the agent has the ACTION_1 skillset
        if int(task.id) % 2 == 0 and agent.has_skill("ACTION_1"):
            valid_agents.append(agent)

        # -> Elif the task id is odd and the agent has the ACTION_2 skillset
        elif int(task.id) % 2 != 0 and agent.has_skill("ACTION_2"):
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
        # -> Add bid to the list
        bids.append({
            "agent_id": agent.id,
            "bid": 100 + random.uniform(00.00000000001, 0.1),
            "allocation": 0
        })

    # logger.info(f"Bids calculated: {bids}")

    return bids
