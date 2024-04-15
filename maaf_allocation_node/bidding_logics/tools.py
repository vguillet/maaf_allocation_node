
##################################################################################################################

# Built-in/Generic Imports
import random
from pprint import pformat

# Libs
import networkx as nx

# Own modules
try:
    from maaf_tools.datastructures.task.Task import Task
    from maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.datastructures.agent.AgentState import AgentState

    from maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid
    from maaf_tools.tools import *

except ImportError:
    from maaf_tools.maaf_tools.datastructures.task.Task import Task
    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.agent.AgentState import AgentState

    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid
    from maaf_tools.maaf_tools.tools import *

##################################################################################################################


def get_valid_agent_list(task: Task, agent_lst: list[Agent]) -> list[Agent]:
    """
    Filter agents unable to take on the task

    :param task: The task to calculate the bid for.
    :param agent_lst: The list of agents to consider for the bid.

    :return: A list of agents that can take on the task.
    """
    # -> Check the agents skillset against the task instructions
    valid_agents = []

    for agent in agent_lst:
        # > If agent has skills for the task, add to valid agents
        if agent.has_skill(task.type):
            valid_agents.append(agent)

        # > Else, do not bid
        else:
            pass

    return valid_agents


def get_valid_agents_for_follow_up_task(task: Task, agent_lst: list[Agent], intercession_targets: list[str]) -> list[Agent]:
    """
    Filter agents unable to take on the task

    :param task: The task to calculate the bid for.
    :param agent_lst: The list of agents to consider for the bid.
    :param intercession_targets: The list of tasks that the agent can intercede for.

    :return: A list of agents that can take on the task.
    """
    # -> Check the agents skillset against the task instructions
    valid_agents = []

    if task.instructions["ACTION_AT_LOC"] in intercession_targets:

        for agent in agent_lst:
            # -> If the goto leads to a task ACTION_1 and the agent has the ACTION_1 skillset and the agent is in the intercession_targets list
            if task.instructions["ACTION_AT_LOC"] in ["ACTION_1", "NO_TASK"] and agent.has_skill("ACTION_1"):
                valid_agents.append(agent)

            # -> Elif the goto leads to a task ACTION_2 and the agent has the ACTION_2 skillset and the agent is in the intercession_targets list
            elif task.instructions["ACTION_AT_LOC"] in ["ACTION_2", "NO_TASK"] and agent.has_skill("ACTION_2"):
                valid_agents.append(agent)

            # -> Else, do not bid
            else:
                pass

    return valid_agents
