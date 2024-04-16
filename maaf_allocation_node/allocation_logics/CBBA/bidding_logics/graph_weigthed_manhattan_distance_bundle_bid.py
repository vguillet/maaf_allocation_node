
##################################################################################################################

# Built-in/Generic Imports
import random
from pprint import pformat
from copy import deepcopy

# Libs
import networkx as nx

# Own modules
try:
    from maaf_tools.datastructures.task.Task import Task
    from maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.datastructures.agent.AgentState import AgentState

    from maaf_allocation_node.bidding_logics.tools import *
    from maaf_tools.tools import *

except ModuleNotFoundError:
    from maaf_tools.maaf_tools.datastructures.task.Task import Task
    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.agent.AgentState import AgentState

    from maaf_allocation_node.maaf_allocation_node.bidding_logics.tools import *
    from maaf_tools.maaf_tools.tools import *

##################################################################################################################

SHALLOW = 0
DEEP = 1


def graph_weighted_manhattan_distance_bundle_bid(
        # > Self parameters
        task: Task,
        agent_lst: list[Agent],
        logger,

        # > States
        environment,
        fleet: Fleet,
        tasklog: TaskLog,

        *args,
        **kwargs
        ) -> list[dict]:
    """
    For the provided agent and task, calculate the marginal gain achieved from inserting a task into the plan
     at the different positions

    # ----- Self parameters
    :param task: The task to calculate the marginal gains for.
    :param agent_lst: The list of agents to calculate the bid from.
    :param logger: The logger to log messages to.

    # ----- States
    :param environment: The environment graph to calculate the distances in if necessary.
    :param fleet: The fleet of agents to calculate the bid from.
    :param tasklog: The task log to store the path for the current bid.

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

    # -> Check the agents skillset against the task instructions
    valid_agents = get_valid_agent_list(task=task, agent_lst=agent_lst)

    # -> Calculate the weighted Manhattan distance for all valid agents
    bids = []

    for agent in valid_agents:
        # -> Agent node
        agent_node = (agent.state.x, agent.state.y)

        # -> Calculate the marginal gains for the each insertion position
        marginal_gains = {}

        for i in range(len(agent.plan) + 1):
            # -> Construct the new plan
            new_plan = agent.plan.clone()

            # > Insert the task at the insertion position
            new_plan.add_task(task, position=i)

            # -> Verify that all necessary paths have been computed
            for source_node, target_node in new_plan.get_node_pairs(agent_id=agent.id):
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
                    # -> Get nodes position
                    # > Source node
                    if source_node in fleet:
                        source_node_loc = (fleet[source_node].state.x, fleet[source_node].state.y)
                    else:
                        source_node_loc = (tasklog[source_node].instructions["x"], tasklog[source_node].instructions["y"])

                    # > Target node
                    if target_node in fleet:
                        target_node_loc = (fleet[target_node].state.x, fleet[target_node].state.y)
                    else:
                        target_node_loc = (tasklog[target_node].instructions["x"], tasklog[target_node].instructions["y"])

                    # -> Find the weigthed Manhattan distance between the agent and the task
                    path = environment["all_pairs_shortest_paths"][source_node_loc][target_node_loc]
                    # path = nx.shortest_path(environment["graph"], agent_node, task_node)
                    # path = nx.astar_path(environment["graph"], agent_node, task_node, weight="weight")

                # > Store path to agent task log
                tasklog.add_path(
                    source_node=source_node,
                    target_node=target_node,
                    path={
                        "id": f"{agent.id}_{task.id}",
                        "path": path,
                        "requirements": ["ground"]       # TODO: Fix once path requirements have been implemented
                    },
                    two_way=False,
                    selection="latest"
                )

            # -> Calculate the marginal gain of the new plan
            marginal_cost_noise = consistent_random(
                string=agent.id + task.id,
                min_value=0.0000000000001,
                max_value=0.000000001
            )   # Start with random tiny number to avoid division by zero and ties in allocation

            # > Calculate the size difference between the new and current path
            plan_path, _, _ = agent.plan.get_path(
                agent_id=agent.id,
                tasklog=tasklog,
                requirement=None,       # TODO: Fix once path requirements have been implemented
                selection="shortest"
            )

            new_plan_path, _, _ = new_plan.get_path(
                agent_id=agent.id,
                tasklog=tasklog,
                requirement=None,       # TODO: Fix once path requirements have been implemented
                selection="shortest"
            )

            # path_cost = 1/len(plan_path) if len(plan_path) > 0 else 0
            # new_path_cost = 1/len(new_plan_path) if len(new_plan_path) > 0 else 0

            # marginal_cost = marginal_cost_noise + (new_path_cost - path_cost)/len(new_plan)
            #
            # if len(plan_path) == 0:
            #     marginal_cost = 1/marginal_cost_noise

            # if len(new_plan_path)-len(plan_path) == 0:
            #     marginal_cost = 1/marginal_cost_noise
            # else:
            #     # marginal_cost = 1/(len(new_plan_path) - len(plan_path) + marginal_cost_noise + 1) * 1/((i+1)*1)
            #     # marginal_cost = 1/(len(new_plan_path) - len(plan_path) + marginal_cost_noise + 1) * 1/len(new_plan)
            # marginal_cost = 1/(marginal_cost_noise + len(new_plan_path) - len(plan_path) + 1)
            marginal_cost = 1/(marginal_cost_noise + len(new_plan_path) - len(plan_path))
            # if task.id == "0":
            #     logger.info(f"      > Agent pos: {agent.id}")
            #     logger.info(f"      Path extension: {len(new_plan_path) - len(plan_path)}, old path: {len(plan_path)}, new path: {len(new_plan_path)}")
            #     logger.info(f"      Marginal cost: {marginal_cost}")

            # > Store the marginal gain
            marginal_gains[i] = {
                "value": marginal_cost,
                "allocation": 0,
                "bids_depth": SHALLOW
            }

        # -> Add bid to the list
        bids.append({
            "agent_id": agent.id,
            "marginal_gains": marginal_gains
        })

    return bids
