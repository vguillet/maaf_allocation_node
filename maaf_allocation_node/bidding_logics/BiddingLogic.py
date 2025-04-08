
##################################################################################################################

# Built-in/Generic Imports
from abc import ABC, abstractmethod
import inspect
from functools import wraps

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
    from maaf_tools.tools import *

except ModuleNotFoundError:
    from maaf_tools.maaf_tools.datastructures.task.Task import Task
    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.organisation.Organisation import Organisation
    from maaf_tools.maaf_tools.datastructures.agent.AgentState import AgentState

    from maaf_allocation_node.maaf_allocation_node.bidding_logics.tools import *
    from maaf_tools.maaf_tools.tools import *

##################################################################################################################


class BiddingLogic(ABC):
    def __init__(self, type: str, name: str, description: str):

        self.type = type
        self.name = name
        self.description = description

        # -> Monkey patch the __compute_bids method to add the verify_compute_bids decorator
        self.child_compute_bids = self.compute_bids
        self.compute_bids = self.__compute_bids

    @staticmethod
    def verify_compute_bids(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            # Retrieve the function signature and bind the arguments.
            sig = inspect.signature(func)
            bound_args = sig.bind(*args, **kwargs)
            bound_args.apply_defaults()

            # Validate key input parameters.
            task = bound_args.arguments.get("task")
            agent_lst = bound_args.arguments.get("agent_lst")
            fleet = bound_args.arguments.get("fleet")
            organisation = bound_args.arguments.get("organisation")
            tasklog = bound_args.arguments.get("tasklog")
            bids_cache = bound_args.arguments.get("bids_cache")

            if not isinstance(task, Task):
                raise TypeError(f"Parameter 'task' must be an instance of Task (task = {task}).")

            if not isinstance(agent_lst, list):
                raise TypeError(f"Parameter 'agent_lst' must be a list of Agent instances (agent_lst = {agent_lst}).")
            for agent in agent_lst:
                if not isinstance(agent, Agent):
                    raise TypeError(f"Each element in 'agent_lst' must be an instance of Agent (agent = {agent}).")

            if not isinstance(fleet, Fleet):
                raise TypeError(f"Parameter 'fleet' must be an instance of Fleet (fleet = {fleet}).")

            if not isinstance(organisation, Organisation):
                raise TypeError(f"Parameter 'organisation' must be an instance of Organisation (organisation = {organisation}).")

            if not isinstance(tasklog, TaskLog):
                raise TypeError(f"Parameter 'tasklog' must be an instance of TaskLog (tasklog = {tasklog}).")

            if bids_cache is not None and not isinstance(bids_cache, dict):
                raise TypeError(f"Parameter 'bids_cache' must be a dictionary (bids_cache = {bids_cache}).")

            # Execute the original function.
            result = func(*args, **kwargs)

            # Validate the output format: should be a tuple with two elements.
            if not isinstance(result, tuple) or len(result) != 2:
                raise TypeError(
                    "Return value must be a tuple with two elements: (list of bid dictionaries, bids cache dictionary).")

            bid_list, bids_cache_output = result
            if not isinstance(bid_list, list):
                raise TypeError("The first element of the return tuple must be a list of bid dictionaries.")
            if not isinstance(bids_cache_output, dict):
                raise TypeError("The second element of the return tuple must be a dictionary (bids cache).")

            # Validate each bid dictionary.
            required_keys = {"agent_id", "marginal_gains"}
            for bid in bid_list:
                if not isinstance(bid, dict):
                    raise TypeError("Each bid must be a dictionary.")
                missing_keys = required_keys - bid.keys()
                if missing_keys:
                    raise ValueError(f"Bid dictionary is missing required keys: {missing_keys}")
                if not isinstance(bid["agent_id"], str):
                    raise TypeError("The 'agent_id' in each bid dictionary must be a string.")
                if not isinstance(bid["marginal_gains"], list):
                    raise TypeError("The 'marginal_gains' in each bid dictionary must be a list.")
                for gain in bid["marginal_gains"]:
                    if not isinstance(gain, (int, float)):
                        raise TypeError("Each value in 'marginal_gains' must be an int or float.")
            return result

        return wrapper

    @verify_compute_bids
    def __compute_bids(self,
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
                       *args, **kwargs
                       ):
        """
        This is a wrapper function that calls the compute_bids method (through the monkey patched child_compute_bid).
        It is used to ensure that the compute_bids method is called with the correct parameters.
        """
        return self.child_compute_bids(
            # > Self parameters
            task=task,
            agent_lst=agent_lst,
            logger=logger,

            # > States
            environment=environment,
            fleet=fleet,
            organisation=organisation,
            tasklog=tasklog,

            bids_cache=bids_cache,
            *args, **kwargs
        )

    @staticmethod
    @abstractmethod
    def compute_bids(# > Self parameters
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
        pass
