
##################################################################################################################

# Built-in/Generic Imports
import random
from pprint import pformat
from copy import deepcopy

# Libs
import networkx as nx

# Own modules
from maaf_tools.datastructures.task.Task import Task
from maaf_tools.datastructures.task.TaskLog import TaskLog

from maaf_tools.datastructures.agent.Agent import Agent
from maaf_tools.datastructures.agent.Fleet import Fleet
from maaf_tools.datastructures.agent.AgentState import AgentState

from maaf_tools.tools import *

##################################################################################################################

SHALLOW = 0
DEEP = 1


def graph_weighted_manhattan_distance_bundle_bid(
        # > Base parameters
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
    For the provided agent and task, calculate the marginal gain achieved from inserting a task into the plan
     at the different positions

    :param task: The task to calculate the marginal gains for.
    :param tasklog: The task log to store the path for the current bid.

    :param agent_lst: The list of agents to calculate the bid from.
    :param fleet: The fleet of agents to calculate the bid from.

    :param environment: The environment graph to calculate the distances in if necessary.
    :param logger: The logger to log messages to.

    :return: A list of dictionaries containing the agent(s) ID(s) and corresponding marginal gains per insertion position.
    
    ex.: [{
        "agent_id": "agent_1",
        "marginal_gains": {
            0: {
                "value": ...,
                "allocation": ... (0/1/2),
                "bids_depth": ... (0/1)
                },
            1: {...},
            2: {...},
            3: {...}
            }
        }, 
        ...
        }]
    
    """

    bids = []

    # -> Check the agents skillset against the task instructions
    valid_agents = []

    for agent in agent_lst:
        # > If agent has skills for the task, add to valid agents
        if agent.has_skill(task.type):
            valid_agents.append(agent)

        # > Else, do not bid (return 0 for all insertion positions)
        else:
            marginal_gains = {}
            for i in range(len(agent.plan)):
                marginal_gains[i] = {
                    "value": 0,
                    "allocation": 0,
                    "bids_depth": 0
                }

            bids.append({
                "agent_id": agent.id,
                "marginal_gains": marginal_gains
            })

    # -> Calculate the weighted Manhattan distance for all valid agents
    for agent in valid_agents:
        # -> Agent node
        agent_node = (agent.state.x, agent.state.y)

        # -> Calculate the marginal gains for the each insertion position
        marginal_gains = {}

        for i in range(len(agent.plan) + 1):
            # -> Construct the new plan
            new_plan = deepcopy(agent.plan)

            # > Insert the task at the insertion position
            new_plan.add_task(task, position=i)

            # -> Verify that all necessary paths have been computed
            for source_node, target_node in zip([agent.id] + new_plan.task_sequence[:-1], new_plan.task_sequence[1:]):
                # -> If agent on a node in the path for the current bid, reuse and trim the path
                current_path = tasklog.get_path(
                    source=source_node,
                    target=target_node,
                    requirement=None,       # TODO: Fix once path requirements have been implemented
                    selection="shortest"
                )

                if current_path:
                    current_path = current_path["path"]

                    if agent_node in current_path:
                        path = current_path[current_path.index(agent_node):]
                        compute_path = False

                    else:
                        compute_path = True
                else:
                    compute_path = True

                if compute_path:
                    # -> Task node
                    task_node = (task.instructions["x"], task.instructions["y"])

                    # -> Find the weigthed Manhattan distance between the agent and the task
                    path = environment["all_pairs_shortest_paths"][agent_node][task_node]
                    # path = nx.shortest_path(environment["graph"], agent_node, task_node)
                    # path = nx.astar_path(environment["graph"], agent_node, task_node, weight="weight")

                # > Store path to agent task log
                tasklog.add_path(
                    source_node=agent.id,
                    target_node=task.id,
                    path={
                        "id": f"{agent.id}_{task.id}",
                        "path": path,
                        "requirements": ["ground"]       # TODO: Fix once path requirements have been implemented
                    },
                    two_way=False,
                    selection="latest"
                )

            # -> Calculate the marginal gain of the new plan
            marginal_cost = consistent_random(
                string=agent.id + task.id,
                min_value=0.0000000000001,
                max_value=0.000000001
            )  # Start with random tiny number to avoid division by zero and ties in allocation

            # > Calculate the size difference between the new and current path
            marginal_cost += len(new_plan.path) - len(agent.plan.path)

            # > Store the marginal gain
            marginal_gains[i] = {
                "value": 1/marginal_cost,
                "allocation": 0,
                "bids_depth": SHALLOW
            }

        # -> Add bid to the list
        bids.append({
            "agent_id": agent.id,
            "marginal_gains": marginal_gains
        })

    return bids
