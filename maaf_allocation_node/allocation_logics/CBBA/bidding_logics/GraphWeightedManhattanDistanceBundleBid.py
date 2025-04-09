
##################################################################################################################

# Built-in/Generic Imports

# Libs

# Own modules
try:
    from maaf_tools.datastructures.task.Task import Task
    from maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.datastructures.organisation.Organisation import Organisation
    from maaf_tools.datastructures.agent.AgentState import AgentState

    from maaf_allocation_node.bidding_logics.tools import *
    from maaf_allocation_node.bidding_logics.BiddingLogic import BiddingLogic
    from maaf_tools.tools import *

except ModuleNotFoundError:
    from maaf_tools.maaf_tools.datastructures.task.Task import Task
    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.organisation.Organisation import Organisation
    from maaf_tools.maaf_tools.datastructures.agent.AgentState import AgentState

    from maaf_allocation_node.maaf_allocation_node.bidding_logics.tools import *
    from maaf_allocation_node.maaf_allocation_node.bidding_logics.BiddingLogic import BiddingLogic
    from maaf_tools.maaf_tools.tools import *

##################################################################################################################

SHALLOW = 1
DEEP = 2


class GraphWeightedManhattanDistanceBundleBid(BiddingLogic):
    def __init__(self):
        super().__init__(
            type="CBBA",
            name="GraphWeightedManhattanDistanceBundleBid",
            description="A bidding logic that determines the optimal insertion position for a task in an agent's plan using the weighted Manhattan distance to determine marginal gains."
        )

    @staticmethod
    def compute_bids(
            # > Self parameters
            task: Task,
            agent_lst: list[Agent],
            logger,

            # > States
            environment,
            fleet: Fleet,
            organisation: Organisation,
            tasklog: TaskLog,

            bids_cache: dict = {},
            *args,
            **kwargs
            ) -> (list[dict], dict):
        """
        For the provided agent and task, calculate the marginal gain achieved from inserting a task into the plan
         at the different positions

        # ----- Self parameters
        :param task: The task to calculate the marginal gains for.
        :param agent_lst: The list of agents to calculate the bid for.
        :param logger: The logger to log messages to.

        # ----- States
        :param environment: The environment graph to calculate the distances in if necessary.
        :param fleet: The fleet of agents to calculate the bid from.
        :param organisation: The organisation to calculate the bid from.
        :param tasklog: The task log to store the path for the current bid.

        # ----- Optional parameters
        :param bids_cache: A dictionary to cache bids for agents. This is used to avoid recalculating bids for the same agent and task.
        :param args: Additional positional arguments.
        :param kwargs: Additional keyword arguments.

        :return: 1) A list of dictionaries containing the agent(s) ID(s) and corresponding marginal gains per insertion position. 2) Bids cache
        ex.: [{
            "agent_id": "agent_1",
            "insertion_loc": ...,
            "bid": ...,
            "allocation": ... (0/1/2),
            "bid_depth": ... (0/1)
            }]
        """

        logger.info(f"Calculating bids for task {task.id} ({task.type}) with {len(agent_lst)} agents.")

        logger.info(f"Task instructions: {task.instructions}")

        for agent in agent_lst:
            logger.info(f"Agent {agent.id} with skillset {agent.skillset}")

        logger.info(f"Valid agents for task {task.id}: {[agent.id for agent in agent_lst]}")

        # -> Calculate the weighted Manhattan distance for all valid agents
        bids = []

        for agent in agent_lst:
            # -> Agent node
            agent_node = (agent.state.x, agent.state.y)

            # ----- MEMOIZATION LOGIC
            # -> Construct the key for the marginal gains cache
            key = (agent_node, tuple(agent.plan.task_sequence), task.id)

            # -> If the marginal gains for the current agent and task have been calculated before, reuse the result
            if key in bids_cache.keys():
                bids.append(bids_cache[key])
                # logger.info(f"Reusing marginal gains for agent {agent.id} and task {task.id} from cache.")
                continue

            # -----

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

                        if source_node in fleet and source_node in current_path:
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
                            source_node_loc = (int(fleet[source_node].state.x), int(fleet[source_node].state.y))  # TODO: SET AS AGENT NODE
                        else:
                            source_node_loc = (int(tasklog[source_node].instructions["x"]), int(tasklog[source_node].instructions["y"]))    # TODO: Check typing if using continuous parameters

                        # > Target node
                        if target_node in fleet:
                            target_node_loc = (fleet[target_node].state.x, fleet[target_node].state.y)
                        else:
                            target_node_loc = (tasklog[target_node].instructions["x"], tasklog[target_node].instructions["y"])

                        # -> Find the weigthed Manhattan distance between the agent and the task
                        # logger.info(f"Path from {source_node_loc} to {target_node_loc}")
                        # logger.info(f"Nodes: {environment['all_pairs_shortest_paths'].keys()}")
                        # logger.info(f'{source_node_loc}->{target_node_loc}, {environment["all_pairs_shortest_paths"][source_node_loc].keys()}')
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
                marginal_gain_noise = consistent_random(
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

                # path_gain = 1/len(plan_path) if len(plan_path) > 0 else 0
                # new_path_gain = 1/len(new_plan_path) if len(new_plan_path) > 0 else 0

                # marginal_gain = marginal_gain_noise + (new_path_gain - path_gain)/len(new_plan)
                #
                # if len(plan_path) == 0:
                #     marginal_gain = 1/marginal_gain_noise

                # if len(new_plan_path)-len(plan_path) == 0:
                #     marginal_gain = 1/marginal_gain_noise
                # else:
                #     # marginal_gain = 1/(len(new_plan_path) - len(plan_path) + marginal_gain_noise + 1) * 1/((i+1)*1)
                #     # marginal_gain = 1/(len(new_plan_path) - len(plan_path) + marginal_gain_noise + 1) * 1/len(new_plan)
                # marginal_gain = 1/(marginal_gain_noise + len(new_plan_path) - len(plan_path))

                # marginal_gain = 1/(len(new_plan_path) + new_plan_actions_gain) - 1/(len(plan_path) + plan_actions_gain) + (1/marginal_gain_noise if agent_node == (task.instructions["x"], task.instructions["y"]) else marginal_gain_noise)
                # new_plan_actions_gain = len(new_plan) - (1 if agent_node == (task.instructions["x"], task.instructions["y"]) else 0)

                if agent_node == (task.instructions["x"], task.instructions["y"]) and i == 0:
                    marginal_gain = 1/marginal_gain_noise

                else:
                    plan_actions_gain = len(agent.plan)
                    new_plan_actions_gain = len(new_plan)
                    try:
                        # marginal_gain = 1 / (len(new_plan_path) + new_plan_actions_gain - len(plan_path) - plan_actions_gain) + marginal_gain_noise
                        marginal_gain = 1 / (len(new_plan_path) + new_plan_actions_gain) + marginal_gain_noise

                    except:
                        logger.warning(f"Agent {agent.id} ({agent_node}) for task {task.id} inserted at {i}:"
                                       f"\n    - Old Plan: {agent.plan} ({len(plan_path)} distance steps + {len(agent.plan)} actions) = {len(plan_path) + len(agent.plan)}"
                                       f"\n    - New Plan: {new_plan} ({len(new_plan_path)} distance steps + {len(new_plan)} actions) = {len(new_plan_path) + len(new_plan)}"
                                       f"\n        - Old: {plan_path}"
                                       f"\n        - New: {new_plan_path}"
                                       )

                # > Store the marginal gain
                marginal_gains[i] = {
                    "bid": marginal_gain,
                    "allocation": 0,
                    # "bid_depth": SHALLOW
                    "bid_depth": DEEP
                }

            # -> Find largest marginal gain and loc
            max_marginal_gain = None
            max_insertion_loc = None

            for insertion_loc, marginal_gain in marginal_gains.items():
                if max_marginal_gain is None:
                    max_marginal_gain = marginal_gain
                    max_insertion_loc = insertion_loc

                elif marginal_gain["bid"] > max_marginal_gain["bid"]:
                    max_marginal_gain = marginal_gain
                    max_insertion_loc = insertion_loc

            if max_marginal_gain is not None:
                # -> Add bid to the list
                bid = {
                    "agent_id": agent.id,
                    "insertion_loc": max_insertion_loc,
                    "bid": max_marginal_gain["bid"],
                    "allocation": max_marginal_gain["allocation"],
                    "bid_depth": max_marginal_gain["bid_depth"]
                    }

                bids.append(bid)

                # -> Store in the marginal gains cache
                bids_cache[key] = bid

        logger.info(f"{bids}, {bids_cache}")

        return bids, bids_cache


if __name__ == "__main__":
    # Test the GraphWeightedManhattanDistanceBundleBid class

    bidding_method = GraphWeightedManhattanDistanceBundleBid()

    bidding_method.compute_bids(
        task=None,
        agent_lst=None,
        logger=None,
        environment=None,
        fleet=Fleet(),
        tasklog=TaskLog(),
        bids_cache={}
    )
