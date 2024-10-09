
##################################################################################################################

# Built-in/Generic Imports

# Libs

# Own modules
try:
    from maaf_tools.datastructures.task.Task import Task
    from maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.datastructures.agent.AgentState import AgentState

    from maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid
    from maaf_allocation_node.bidding_logics.priority_bid_amplifier import priority_bid_amplifier
    from maaf_allocation_node.bidding_logics.tools import *
    from maaf_tools.tools import *

except ImportError:
    from maaf_tools.maaf_tools.datastructures.task.Task import Task
    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.agent.AgentState import AgentState

    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid
    from maaf_allocation_node.maaf_allocation_node.bidding_logics.priority_bid_amplifier import priority_bid_amplifier
    from maaf_allocation_node.maaf_allocation_node.bidding_logics.tools import *
    from maaf_tools.maaf_tools.tools import *

##################################################################################################################

SHALLOW = 1
DEEP = 2


def interceding_skill_based_bid_amplifier(
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

        interventionism: float,
        *args,
        **kwargs
        ) -> (list[dict], dict):
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
    :param fleet: The fleet of agents.
    :param shared_bids_b: The shared bids for the agents.
    :param shared_bids_priority_beta: The priority shared bids for the agents.
    """

    # -> Check if task intervention has been established
    if "intervention" not in task.shared.keys():
        # -> Establish if intervention performed
        # > Compute boolean with specific probabilities
        intervention = random.choices([True, False], weights=[interventionism, 1 - interventionism], k=1)[0]

        # > Store result in task local field
        task.shared["intervention"] = intervention

    # -> If no intervention, return empty bids
    if not task.shared["intervention"]:
        return [], {}

    # -> Get bids
    magnified_bids = priority_bid_amplifier(
        # > Self parameters
        self_agent=self_agent,
        task=task,
        agent_lst=agent_lst,
        intercession_targets=intercession_targets,
        logger=logger,

        # > States
        fleet=fleet,
        shared_bids_b=shared_bids_b,
        shared_bids_priority_beta=shared_bids_priority_beta,

        # > Parameters
        # amplification_factor=1000
    )

    # -> Reformat bids for CBBA algorithm
    bids = []

    for bid in magnified_bids:
        bids.append({
            "agent_id": bid["agent_id"],
            "bid": bid["bid"],
            "allocation": bid["allocation"],
            # "bid_depth": SHALLOW,
            "insertion_loc": 0,
            "bid_depth": DEEP,
            }
        )

    return bids, {}
