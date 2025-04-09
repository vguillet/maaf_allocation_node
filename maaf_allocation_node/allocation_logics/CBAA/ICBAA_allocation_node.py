
##################################################################################################################

"""
This module contains the MAAF allocation node class, which is a ROS2 node that implements the CBAA algorithm for task allocation.
"""

# Built-in/Generic Imports
import time
from typing import List, Optional, Tuple
from copy import deepcopy
from pprint import pprint, pformat

import pandas as pd

# ROS2 Imports

# Local Imports
try:
    from maaf_config.maaf_config import *

    from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
    from maaf_allocation_node.maaf_agent import MAAFAgent
    from maaf_allocation_node.allocation_logics.ICB_agent import ICBAgent

    from maaf_tools.datastructures.task.TaskLog import TaskLog
    from maaf_tools.datastructures.task.Task import Task

    from maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.tools import *

    from maaf_allocation_node.allocation_logics.CBAA.bidding_logics.anticipated_action_task_interceding_agent import anticipated_action_task_interceding_agent
    from maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid

except ModuleNotFoundError:
    from maaf_config.maaf_config.maaf_config import *

    from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
    from maaf_allocation_node.maaf_allocation_node.maaf_agent import MAAFAgent
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.ICB_agent import ICBAgent

    from maaf_tools.maaf_tools.datastructures.task.Task import Task
    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.maaf_tools.tools import *

    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBAA.bidding_logics.anticipated_action_task_interceding_agent import anticipated_action_task_interceding_agent
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid

##################################################################################################################


class ICBAANode(ICBAgent):
    def __init__(self):
        # ---- Init parent class
        ICBAgent.__init__(
            self,
            node_name="ICBAA_node",
            skillset=None
        )

        # ----------------------------------- Agent allocation states
        # -> Setup additional CBAA-specific allocation states
        self._setup_allocation_additional_states()

        # -> Initialise previous state hash
        self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

        # ----------------------------------- Confirm initialisation
        # -> Set self state as online
        self.agent.set_online_state(online=True)

        # -> Initial publish to announce the agent to the fleet and share initial state
        time.sleep(2)
        self.publish_allocation_state_msg()

    # ============================================================== PROPERTIES
    def _setup_allocation_additional_states(self) -> None:
        """
        Setup additional method-dependent allocation states for the agents
        """
        # ---- Local state
        """
        Task list x of size N_t: 
        - 0 if not assigned
        - 1 if assigned

        List is a pandas dataframe of size N_t, initialized as zero matrix, with task ids as index
        """
        self.task_list_x = pd.DataFrame(
            np.zeros((self.Task_count_N_t, 1)),
            index=self.tasklog.ids_pending,
            columns=["task_list_x"]
        )

    # ============================================================== PROPERTIES
    # ---------------- Self state
    @property
    def local_allocation_state(self) -> dict:
        """
        Local allocation state at current time step (not serialised)
        !!!! All entries must be dataframes !!!!

        :return: dict
        """
        return {
            "task_list_x": self.task_list_x,

            # > Base local allocation states
            "local_bids_c": self.local_bids_c,
            "local_allocations_d": self.local_allocations_d
        }

    # ============================================================== METHODS
    # ---------------- Callbacks

    # ---------------- Processes
    def update_situation_awareness(self,
                                   tasklog: Optional[TaskLog],
                                   fleet: Optional[Fleet]
                                   ) -> Tuple[bool, bool]:
        """
        Update local states with new tasks and fleet dicts. Add new tasks and agents to local states and extend
        local states with new rows and columns for new tasks and agents. Remove tasks and agents from local states if
        they are no longer in the task list or fleet.

        :param tasklog: Task log to merge
        :param fleet: Fleet to merge

        :return: Tuple of bools (task_state_change, fleet_state_change)
        """

        def add_agent(agent: Agent) -> None:
            """
            Add new agent to local fleet and extend local states with new columns for new agent
            """

            # -> Add a column to all relevant allocation lists and matrices with new agent
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            # > Remove task_list_x from the state (not needed for this operation)
            state.pop("task_list_x")

            # > Remove winning_bids_y from the state (not needed for this operation)
            state.pop("winning_bids_y")

            for matrix in state.values():
                matrix[agent.id] = pd.Series(np.zeros(self.Task_count_N_t), index=self.tasklog.ids_pending)

        def remove_agent(agent: Agent) -> None:
            """
            Remove agent from local fleet and all relevant allocation lists and matrices
            """

            # TODO: Figure out how to deal with removing bids from the winning bid matrix (full reset vs bid owner tracking). For now, reset winning bids matrix (full reset)
            self.winning_bids_y = pd.Series(np.zeros(self.Task_count_N_t), index=self.tasklog.ids_pending)

            # -> Remove agent from all allocation lists and matrices
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            # > Remove task_list_x from the state (not needed for this operation)
            state.pop("task_list_x")

            # > Remove winning_bids_y from the state (not needed for this operation)
            state.pop("winning_bids_y")

            for matrix in state.values():
                if agent.id in matrix.columns:
                    matrix.drop(columns=agent.id, inplace=True)

        def add_task(task: Task) -> None:
            """
            Add new task to local tasks and extend local states with new rows for new task
            """
            # -> Add a row to all allocation lists and matrices with new task
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            # > Remove task_list_x from the state (operation performed separately)
            state.pop("task_list_x")
            self.task_list_x.loc[task.id] = 0

            # > Remove winning_bids_y from the state (operation performed separately)
            state.pop("winning_bids_y")
            self.winning_bids_y.loc[task.id] = 0

            for matrix in state.values():
                matrix.loc[task.id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)

        def terminate_task(task: Task) -> None:
            """
            Flag task as terminated in task log and remove all relevant allocation lists and matrices rows
            """

            # -> Cancel goal if task is assigned to self
            if self.task_list_x.loc[task.id, "task_list_x"] == 1:
                # -> Cancel goal
                self._drop_task(
                    task_id=task.id,
                    reset=False,
                    traceback="Task termination",
                    logger=False,
                )

            # -> Remove task from all allocation lists and matrices
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            for matrix in state.values():
                if task.id in matrix.index:
                    matrix.drop(index=task.id, inplace=True)

        # ---- Merge received fleet into local one
        fleet_state_change = self.fleet.merge(
            fleet=fleet,
            add_agent_callback=add_agent,
            remove_agent_callback=remove_agent,
            fleet_state_change_callback=None,
            prioritise_local=False,
            logger=self.get_logger()
        )

        # ---- Merge received task list into local one
        task_state_change = self.tasklog.merge(
            tasklog=tasklog,
            add_task_callback=add_task,
            terminate_task_callback=terminate_task,
            tasklog_state_change_callback=self._compute_bids,
            prioritise_local=False,
            logger=self.get_logger()
        )

        # # TODO: Cleanup
        # if self.bid_evaluation_function is anticipated_action_task_interceding_agent:
        #     for task in self.tasklog.tasks_pending:
        #         # -> Compute bids
        #         self._bid(task=task, agent_lst=self.fleet.agents_active)

        return task_state_change, fleet_state_change

    def update_shared_states(self,
                             shared_bids_b: pd.DataFrame,
                             shared_bids_priority_beta: pd.DataFrame,
                             shared_allocations_a: pd.DataFrame,
                             shared_allocations_priority_alpha: pd.DataFrame,
                             *args,
                             **kwargs
                             ):
        """
        Update local states with received states from the fleet

        :param shared_bids_b: Task bids matrix b received from the fleet
        :param shared_bids_priority_beta: Task bids priority matrix beta received from the fleet
        :param shared_allocations_a: Task allocations matrix a received from the fleet
        :param shared_allocations_priority_alpha: Task allocations priority matrix alpha received from the fleet
        """

        # ------------------------------------------------- Intercession
        tasks_ids = list(shared_bids_b.index)
        agent_ids = list(shared_bids_b.columns)

        # -> For each task ...
        for task_id in tasks_ids:
            # -> If the task has been terminated, skip
            if task_id not in self.tasklog.ids_pending:
                continue

            # -> for each agent ...
            for agent_id in agent_ids:
                # if agent_id not in self.fleet.ids_active:
                #     continue

                # > If there is a shared bid, merge it into the current bids
                if shared_bids_b.loc[task_id, agent_id] > 0:
                    # -> Priority merge with reset received current bids b into local current bids b
                    # > Determine correct matrix values
                    shared_bids_b_ij_updated, shared_bids_priority_beta_ij_updated = self.priority_merge(
                        # Logging
                        task_id=task_id,
                        agent_id=agent_id,

                        # Merging
                        matrix_updated_ij=self.shared_bids_b.loc[task_id, agent_id],
                        matrix_source_ij=shared_bids_b.loc[task_id, agent_id],
                        priority_updated_ij=self.shared_bids_priority_beta.loc[task_id, agent_id],
                        priority_source_ij=shared_bids_priority_beta.loc[task_id, agent_id],

                        # Reset
                        currently_assigned=self.agent.plan.has_task(task_id=task_id),
                        reset=True
                    )

                    # > Update local states
                    self.shared_bids_b.loc[task_id, agent_id] = shared_bids_b_ij_updated
                    self.shared_bids_priority_beta.loc[task_id, agent_id] = shared_bids_priority_beta_ij_updated

                # -> Priority merge received current allocations a into local current allocations a
                # > Determine correct matrix values
                shared_allocations_a_ij_updated, shared_allocations_priority_alpha_ij_updated = self.priority_merge(
                    # Logging
                    task_id=task_id,
                    agent_id=agent_id,

                    # Merging
                    matrix_updated_ij=self.shared_allocations_a.loc[task_id, agent_id],
                    matrix_source_ij=shared_allocations_a.loc[task_id, agent_id],
                    priority_updated_ij=self.shared_allocations_priority_alpha.loc[task_id, agent_id],
                    priority_source_ij=shared_allocations_priority_alpha.loc[task_id, agent_id],

                    # Reset
                    currently_assigned=self.agent.plan.has_task(task_id=task_id),
                    reset=True
                )

                # > Update local states
                self.shared_allocations_a.loc[task_id, agent_id] = shared_allocations_a_ij_updated
                self.shared_allocations_priority_alpha.loc[task_id, agent_id] = shared_allocations_priority_alpha_ij_updated

    def _compute_bids(self) -> None:
        """
        Conditionally compute bids for all tasks based on the current state and the state the current
        bids were computed in. If the current state is different from the state the current bids were
        computed in, recompute the bids.
        """
        # ... for each task
        for task in self.tasklog.tasks_pending:
            # -> Check bid reference state
            compute_bids = False

            # > If bids were never computed for the task, compute bids
            if "bid(s)_reference_state" not in task.local.keys():
                compute_bids = True

            # > If bids are outdated and recompute on state change is enabled, compute bids # TODO: Review
            elif task.local["bid(s)_reference_state"] != self.agent.state and self.scenario.recompute_bids_on_state_change:
                compute_bids = True

            # -> Re-compute bids if necessary
            if compute_bids:
                # -> List agents to compute bids for
                # TODO: Clean up
                if self.bid_evaluation_function is anticipated_action_task_interceding_agent:
                    agent_lst = self.fleet.agents_active
                else:
                    agent_lst = [self.agent]

                # -> Compute bids
                self._bid(task=task, agent_lst=agent_lst)

    def _bid(self, task: Task, agent_lst: list[Agent]) -> list[dict]:
        """
        Bid for a task

        :param task: Task object
        :param agent_lst: List of agents to compute bids for

        :return: Bid(s) list, with target agent id and corresponding bid and allocation action values
        """

        if self.environment is None:
            # -> Return 0 bids for all agents as the environment is not available
            self.get_logger().warning("!!!!!! WARNING: Environment not available")
            return []

        # -> If no bid evaluation function, return empty list
        if self.bid_evaluation_function is None:
            return []

        # -> Compute bids
        task_bids, _ = self.bid_evaluation_function(
            # > Self parameters
            self_agent=self.agent,
            task=task,
            agent_lst=agent_lst,
            intercession_targets=self.scenario.intercession_targets,
            logger=self.get_logger(),

            # > States
            **self.get_state(
                environment=True,
                state_awareness=True,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )
        )

        # -> Store bids to local bids matrix
        for bid in task_bids:
            # > Bid
            self.local_bids_c.loc[task.id, bid["agent_id"]] = bid["bid"]

            # > Allocation
            self.local_allocations_d.loc[task.id, bid["agent_id"]] = bid["allocation"]

        # -> Set task bid reference state
        task.local["bid(s)_reference_state"] = deepcopy(self.agent.state)

        return task_bids

    def update_task(self,
                    task: Task,
                    winning_bids_y: pd.DataFrame,
                    *args,
                    **kwargs
                    ) -> None:
        """
        Update current task based on received winning bids and updated allocation intercession from the fleet

        :param task: task id
        :param winning_bids_y: Winning bids list y received from the fleet
        """

        if not self.agent.plan.has_task(task_id=task.id):
            return

        # -> Create set of agents with imposed allocations
        agents_with_imposed_allocations = set(
            self.shared_allocations_a.loc[task.id, self.shared_allocations_a.loc[task.id] == 1].index.to_list()
        )

        # -> If there are imposed allocations, find the imposed allocation with the highest priority
        if len(agents_with_imposed_allocations) > 0:
            # > Find agent with the highest priority
            winning_agent = self.shared_allocations_priority_alpha.loc[
                task.id, agents_with_imposed_allocations].idxmax()

        else:
            # -> Compare received winning bids with local winning bids
            winning_agent = self.id if self.winning_bids_y.loc[task.id, "winning_bids_y"] >= \
                                       winning_bids_y.loc[task.id, "winning_bids_y"] else None

            # TODO: Fix self.winning_bids_y.loc[task.id] > winning_bids_y.loc[task.id] scenario. For now it is assumed that two agents cannot have the same bid for the same task

        # -> Update task list x
        # > If the winning agent is not the agent, remove the task from the task list
        if winning_agent is not self.id:
            # -> Cancel goal
            self._drop_task(
                task_id=task.id,
                reset=False,
                traceback="Task update",
                logger=True
            )

    def update_allocation(self,
                          reset_assignment: bool = False,
                          *args,
                          **kwargs
                          ) -> None:
        """
        Select a task to bid for based on the current state
        1. Merge local states with shared states and update local states if necessary
        2. If no task is assigned to self, select a task

        :param reset_assignment: Flag to reset the current assignment
        """
        # ---- Merge local states with shared states
        # -> If own states have changed, update local states (optimisation to avoid unnecessary updates)
        if self.allocation_state_change:
            # > For each task ...
            for task_id in self.tasklog.ids_pending:
                # > For each agent ...
                for agent_id in self.fleet.ids_active:
                    # > If there is a local bid, merge it into the current bids
                    if self.local_bids_c.loc[task_id, agent_id] > 0:
                        # -> Priority merge local bids c into current bids b
                        # > Determine correct matrix values
                        shared_bids_b_ij_updated, shared_bids_priority_beta_ij_updated = self.priority_merge(
                            # Logging
                            task_id=task_id,
                            agent_id=agent_id,

                            # Merging
                            matrix_updated_ij=self.shared_bids_b.loc[task_id, agent_id],
                            matrix_source_ij=self.local_bids_c.loc[task_id, agent_id],
                            priority_updated_ij=self.shared_bids_priority_beta.loc[task_id, agent_id],
                            priority_source_ij=self.hierarchy_level,

                            # Reset
                            currently_assigned=self.agent.plan.has_task(task_id),
                            # currently_assigned=bool(self.task_list_x.loc[task_id, "task_list_x"]),
                            reset=False
                        )

                        # > Update local states
                        self.shared_bids_b.loc[task_id, agent_id] = shared_bids_b_ij_updated
                        self.shared_bids_priority_beta.loc[task_id, agent_id] = shared_bids_priority_beta_ij_updated

                    # -> Merge local intercessions into allocation intercession
                    if self.hierarchy_level > self.shared_allocations_priority_alpha.loc[task_id, agent_id]:
                        allocation_state = self.action_to_allocation_state(
                            action=self.local_allocations_d.loc[task_id, agent_id]
                        )

                        if allocation_state is not None:
                            self.shared_allocations_a.loc[task_id, agent_id] = allocation_state

        # ---- Select task
        # -> If reset assignment, reset the task list
        if reset_assignment:
            # -> Unassign task if currently assigned
            if self.task_list_x["task_list_x"].sum() != 0:
                # -> Find currently assigned task
                task_id = self.task_list_x[self.task_list_x["task_list_x"] == 1].index[0]

                # -> Cancel goal
                self._drop_task(
                    task_id=task_id,
                    reset=True,     # Reset y to zero for the task
                    traceback="Reset assignment",
                    logger=True
                )

        # -> If no task is assigned to self, select a task
        if self.task_list_x["task_list_x"].sum() == 0:
            # -> Create list of valid tasks (pandas series of task ids initialised as 0)
            valid_tasks_list_h = pd.DataFrame(
                np.zeros(self.Task_count_N_t),
                index=self.tasklog.ids_pending,
                columns=["valid_tasks_list_h"]
            )

            # > For each task ...
            for task_id in self.tasklog.ids_pending:
                # -> Create set of agents with imposed allocations
                agents_with_imposed_allocations = list(set(self.shared_allocations_a.loc[task_id, self.shared_allocations_a.loc[task_id] == 1].index))

                # > If there are imposed allocations, check if self is imposed allocation with the highest priority
                if len(agents_with_imposed_allocations) > 0:
                    # -> Find agent with the highest priority
                    winning_agent = self.shared_allocations_priority_alpha.loc[task_id, agents_with_imposed_allocations].idxmax()

                    # TODO: Adjust to handle conflicting imposed allocations. For now assumed that allocation intercession is conflict-free

                    # > If self is the winning agent, add the task to the task list
                    if winning_agent == self.id:
                        valid_tasks_list_h[task_id, "valid_tasks_list_h"] = 1
                else:
                    # > If there are no imposed allocations, check if self has the highest bid and is not blacklisted
                    valid_task = int(self.shared_allocations_a.loc[task_id, self.id] != -1 and self.shared_bids_b.loc[task_id, self.id] > self.winning_bids_y.loc[task_id, "winning_bids_y"])

                    valid_tasks_list_h.loc[task_id, "valid_tasks_list_h"] = valid_task

            # -> If there are valid tasks, select the task with the highest bid
            if valid_tasks_list_h["valid_tasks_list_h"].sum() > 0:
                # -> Select task with the highest bid
                # > Get index of all the tasks for which the valid_tasks_list_h is 1
                valid_tasks = valid_tasks_list_h[valid_tasks_list_h["valid_tasks_list_h"] == 1].index.to_list()

                # > Get valid task with largest bid (using shared bids !!!)
                selected_task_id = self.shared_bids_b.loc[valid_tasks, self.id].idxmax()

                # -> Add task to the task list
                self.task_list_x.loc[selected_task_id, "task_list_x"] = 1

                # -> Update winning bids
                self.winning_bids_y.loc[selected_task_id, "winning_bids_y"] = self.shared_bids_b.loc[selected_task_id, self.id]

                # if RUN_MODE == SIM: # TODO: Enable once comm sim setup

                # -> Add task to plan
                self.agent.add_task_to_plan(
                    tasklog=self.tasklog,
                    task=selected_task_id,
                    bid=self.shared_bids_b.loc[selected_task_id, self.id],
                    logger=self.get_logger()
                )

                # > Publish goal msg
                self.publish_goal_msg(meta_action="update", traceback="Select task")

    def _drop_task(self,
                   task_id: str,
                   reset: bool = False,
                   traceback: str = None,
                   logger=True,
                   *args,
                   **kwargs
                   ) -> None:
        """
        Drop a task from the bundle list and plan

        :param task_id: Id of task to drop
        :param reset: Flag to reset the winning bids y for the task
        :param traceback: Reason for dropping the task
        :param logger: Whether to log the task drop
        """

        if reset:
            # > Reset winning bids
            self.winning_bids_y.loc[task_id, "winning_bids_y"] = 0

        if "bid(s)_reference_state" in self.tasklog[task_id].local.keys():
            del self.tasklog[task_id].local["bid(s)_reference_state"]

        # -> Update local x state
        self.task_list_x.loc[task_id, "task_list_x"] = 0

        # -> Remove the task from the plan
        self.agent.remove_task_from_plan(
            tasklog=self.tasklog,
            task=task_id,
            logger=self.get_logger() if logger else None
        )

        # -> Publish goal msg
        self.publish_goal_msg(meta_action="update", traceback=traceback)

    # >>>> Prints
    def print_state(
            self,
            situation_awareness: bool = False,
            local_allocation_state: bool = False,
            shared_allocation_state: bool = False
    ):
        """
        Print the state of the agent

        :param situation_awareness: Flag to print situation awareness
        :param local_allocation_state: Flag to print local allocation state
        :param shared_allocation_state: Flag to print shared allocation state

        :return: str
        """

        print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Agent {self.id} state:")
        if situation_awareness:
            print("----- Situation awareness")
            pprint(self.tasklog.asdict())
            pprint(self.fleet.asdict())

        if local_allocation_state:
            print("----- Local allocation state")
            print("Task list x:")
            print(self.task_list_x)

            print("\nLocal bids c:")
            print(self.local_bids_c)

            print("\nLocal allocations d:")
            print(self.local_allocations_d)

        if shared_allocation_state:
            print("----- Shared allocation state")
            print("Winning bids y:")
            print(self.winning_bids_y)

            print("\n------------")
            print("Current bids b:")
            print(self.shared_bids_b)

            # print("\nCurrent bids priority beta:")
            # print(self.shared_bids_priority_beta)
            #
            # print("\n------------")
            # print("Current allocations a:")
            # print(self.shared_allocations_a)
            #
            # print("\nCurrent allocations priority alpha:")
            # print(self.shared_allocations_priority_alpha)

        print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")