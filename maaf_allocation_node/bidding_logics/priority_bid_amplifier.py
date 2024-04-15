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

    from maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import \
        graph_weighted_manhattan_distance_bid
    from maaf_allocation_node.bidding_logics.tools import *
    from maaf_tools.tools import *

except ImportError:
    from maaf_tools.maaf_tools.datastructures.task.Task import Task
    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.agent.AgentState import AgentState

    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import \
        graph_weighted_manhattan_distance_bid
    from maaf_allocation_node.maaf_allocation_node.bidding_logics.tools import *
    from maaf_tools.maaf_tools.tools import *


##################################################################################################################


def priority_bid_amplifier(
        # > Self parameters
        self_agent: Agent,
        task: Task,
        agent_lst: list[Agent],
        intercession_targets: list[str],
        logger,

        # > States
        fleet: Fleet,
        shared_bids_b,
        shared_bids_priority_beta,

        # > Parameters
        amplification_factor: int = 1000,

        *args,
        **kwargs
        ) -> list[dict]:
    """
    Calculate the bid for the provided agent as the inverse of the weighted Manhattan distance between the agent and the task.
    Magnify the bid for the task if the agent has the skillset for the task

    # ----- Self parameters
    :param self_agent: The agent for which to calculate the bid.
    :param task: The task to calculate the bid for.
    :param agent_lst: The list of agents to consider for the bid.
    :param intercession_targets: The list of tasks that the agent can intercede for.
    :param logger: The logger to log the bid.

    # ----- States
    :param shared_bids_b: The shared bids for the agents.
    :param shared_bids_priority_beta: The priority shared bids for the agents.

    # ----- Parameters
    :param amplification_factor: The factor to amplify the bid with.

    :return: A list of dictionaries containing the agent(s) ID(s) and corresponding bid and allocation.
    """

    # -> Check the agents skillset against the task instructions
    valid_agents = get_valid_agent_list(
        task=task,
        agent_lst=agent_lst
    )

    # -> Check the agents skillset against the follow up task instructions
    valid_agents = get_valid_agents_for_follow_up_task(
        task=task,
        agent_lst=valid_agents,
        intercession_targets=intercession_targets
    )

    # -> Magnifiy the existing bid it has a priority bellow the local priority
    bids = []

    for agent in valid_agents:
        # -> Check if the agent has a priority bellow the local priority
        logger.info(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> {shared_bids_b.loc[task.id, agent.id]} - {self_agent.hierarchy_level}>{shared_bids_priority_beta.loc[task.id, agent.id]}={self_agent.hierarchy_level > shared_bids_priority_beta.loc[task.id, agent.id]}")
        if self_agent.hierarchy_level > shared_bids_priority_beta.loc[task.id, agent.id]:
            # -> Get the agent's bid
            agent_bid = shared_bids_b.loc[task.id, agent.id]

            # -> Amplify the bid
            amplified_bid = agent_bid * amplification_factor

            if amplified_bid > 0:
                bids.append({
                    "agent_id": agent.id,
                    "bid": amplified_bid,
                    "allocation": 0
                })

        # -> Check if bid present in local bids
        elif agent.id in fleet[agent.id].shared["local_bids_c"].columns and task.id in fleet[agent.id].shared["local_bids_c"].index:
            # -> Check if base agent bid has changed
            if shared_bids_b.loc[task.id, agent.id] != fleet[agent.id].shared["local_bids_c"].loc[task.id, agent.id] * amplification_factor:
                # -> Get the agent's bid
                agent_bid = fleet[agent.id].shared["local_bids_c"].loc[task.id, agent.id]

                # -> Amplify the bid
                amplified_bid = agent_bid * amplification_factor

                if amplified_bid > 0:
                    bids.append({
                        "agent_id": agent.id,
                        "bid": amplified_bid,
                        "allocation": 0
                    })

    logger.info(f"For task {task},\npriority bid amplifier: {pformat(bids)}")

    return bids
