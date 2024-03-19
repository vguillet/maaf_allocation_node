
##################################################################################################################

# Built-in/Generic Imports
import random
from pprint import pformat

# Libs
from networkx import astar_path, shortest_path

# Own modules
from maaf_tools.datastructures.task.Task import Task
from maaf_tools.datastructures.task.TaskLog import TaskLog

from maaf_tools.datastructures.agent.Agent import Agent
from maaf_tools.datastructures.agent.Fleet import Fleet
from maaf_tools.datastructures.agent.AgentState import AgentState

from maaf_allocation_node.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid

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

    # -> Compute base bid for self
    bids.append(
        graph_weighted_manhattan_distance_bid(
            task=task,
            agent_lst=[kwargs["self_agent"]],
            shared_bids_b=shared_bids_b,
            environment=environment,
            logger=logger,
            self_agent=kwargs["self_agent"],  # TODO: Cleanup
            intercession_targets=kwargs["intercession_targets"]  # TODO: Cleanup
        )[0]
    )

    # -> Filter agents unable to take on task
    filtered_agents = []

    for agent in agent_lst:
        if agent.has_skill(task.type):
            filtered_agents.append(agent)
        # -> Else, do not bid
        else:
            pass

    agent_lst = filtered_agents

    # -> Check the agents skillset against the task instructions
    valid_agents = []

    # if task.type in kwargs["intercession_targets"]:
    if task.instructions["ACTION_AT_LOC"] in kwargs["intercession_targets"]:

        for agent in agent_lst:
            # -> If the goto leads to a task ACTION_1 and the agent has the ACTION_1 skillset and the agent is in the intercession_targets list
            if task.instructions["ACTION_AT_LOC"] in ["ACTION_1", "NO_TASK"] and agent.has_skill("ACTION_1"):
            # if task.instructions["ACTION_AT_LOC"] in ["ACTION_1"] and not agent.has_skill("ACTION_1"):
                valid_agents.append(agent)

            # -> Elif the goto leads to a task ACTION_2 and the agent has the ACTION_2 skillset and the agent is in the intercession_targets list
            elif task.instructions["ACTION_AT_LOC"] in ["ACTION_2", "NO_TASK"] and agent.has_skill("ACTION_2"):
            # elif task.instructions["ACTION_AT_LOC"] in ["ACTION_2"] and not agent.has_skill("ACTION_2"):
                valid_agents.append(agent)

            # -> Else, do not bid
            else:
                pass

    # -> Calculate the weighted Manhattan distance for all valid agents
    for agent in valid_agents:
        # -> Agent node
        agent_node = (agent.state.x, agent.state.y)

        # -> Task node
        task_node = (task.instructions["x"], task.instructions["y"])

        # -> Find the weigthed Manhattan distance between the agent and the task
        path = shortest_path(environment["graph"], agent_node, task_node)
        # path = astar_path(environment["graph"], agent_node, task_node, weight="weight")

        # > Get path x and y
        path_x = [node[0] for node in path]
        path_y = [node[1] for node in path]

        # > Store path to agent local
        task.local["path_for_current_bid"] = path

        # -> Calculate the total distance
        total_distance = random.uniform(0.0000000000001, 0.000000001)    # Start with random tiny number to avoid division by zero and ties in allocation

        # for i in range(len(path) - 1):
        #     total_distance += environment["graph"][path[i]][path[i + 1]]["weight"]

        total_distance += len(path) -1

        # -> Add bias
        total_distance = total_distance / 10000

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
