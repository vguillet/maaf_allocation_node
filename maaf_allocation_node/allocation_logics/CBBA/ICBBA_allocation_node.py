
##################################################################################################################

"""
This module contains the MAAF allocation node class, which is a ROS2 node that implements the CBAA algorithm for task allocation.
"""

# Built-in/Generic Imports
import os
import time
from random import randint
from json import dumps, loads
from typing import List, Optional, Tuple
from copy import deepcopy
from datetime import datetime
from pprint import pprint, pformat

from tabulate import tabulate
import pandas as pd
import numpy as np

# ROS2 Imports
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import networkx as nx

# Local Imports
try:
    from orchestra_config.orchestra_config import *  # KEEP THIS LINE, DO NOT REMOVE
    from orchestra_config.sim_config import *

    from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
    from maaf_allocation_node.maaf_agent import MAAFAgent
    from maaf_allocation_node.allocation_logics.ICB_agent import ICBAgent

    from maaf_tools.datastructures.task.TaskLog import TaskLog
    from maaf_tools.datastructures.task.Task import Task

    from maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.tools import *

    from maaf_allocation_node.allocation_logics.CBBA.bidding_logics.interceding_skill_based_bid_amplifier import interceding_skill_based_bid_amplifier
    from maaf_allocation_node.allocation_logics.CBBA.bidding_logics.graph_weigthed_manhattan_distance_bundle_bid import graph_weighted_manhattan_distance_bundle_bid
    from .update_logics.CBBA_update_decisions import _update_decision

except ModuleNotFoundError:
    from orchestra_config.orchestra_config.orchestra_config import *  # KEEP THIS LINE, DO NOT REMOVE
    from orchestra_config.orchestra_config.sim_config import *

    from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
    from maaf_allocation_node.maaf_allocation_node.maaf_agent import MAAFAgent
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.ICB_agent import ICBAgent

    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog
    from maaf_tools.maaf_tools.datastructures.task.Task import Task

    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.maaf_tools.tools import *

    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBBA.bidding_logics.interceding_skill_based_bid_amplifier import interceding_skill_based_bid_amplifier
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBBA.bidding_logics.graph_weigthed_manhattan_distance_bundle_bid import graph_weighted_manhattan_distance_bundle_bid
    from .update_logics.CBBA_update_decisions import _update_decision

##################################################################################################################

SHALLOW = 1 # CANNOT BE ZERO, will clash with priority merge logic otherwise
DEEP = 2


class ICBBANode(ICBAgent):
    def __init__(self):
        # ---- Init parent class
        ICBAgent.__init__(
            self,
            node_name="ICBBA_node",
            skillset=None
        )

        # ----------------------------------- Agent allocation states
        # -> Setup additional CBAA-specific allocation states
        self._setup_allocation_additional_states()

        # -> Initialise previous state hash
        self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

        # self.add_on_pose_update_listener(lambda: self.update_allocation(reset_assignment=True))
        # self.add_on_pose_update_listener(lambda: self.update_allocation(reset_assignment=False))
        # self.add_on_pose_update_listener(self.check_publish_state_change)

        # ----------------------------------- Confirm initialisation
        # -> Initial publish to announce the agent to the fleet and share initial state
        time.sleep(2)
        self.publish_allocation_state_msg()

        self.publish_goal_msg(meta_action="empty", traceback="Initialisation")

    # ============================================================== PROPERTIES
    def _setup_allocation_additional_states(self) -> None:
        """
        Setup additional method-dependent allocation states for the agents
        """
        # ---- Local state
        """
        Bids depth e of the agent. Used to track the depth of the bids for each agent/task combination
        > SHALLOW (0)
        > DEEP    (1)  

        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.bids_depth_e = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.tasklog.ids_pending,
            columns=self.fleet.ids_active,
        )

        """
        Winning agent list k. Used to track the winning agent for each task in winning bids y
        
        Matrix is a pandas dataframe of size N_t x 1, initialized as a matrix of empty strings, with task ids as index
        """
        self.winning_agents_k = pd.DataFrame(
            np.array(["" for _ in range(self.Task_count_N_t)]),
            index=self.tasklog.ids_pending,
            columns=["winning_agents_k"]
        )

        """
        Last update s. Used to track the last update time for each agent in the fleet
        
        Matrix is a pandas dataframe of size N_u x 1, initialized as zero matrix, with agent ids as index
        """
        self.last_update_s = pd.DataFrame(
            np.zeros(self.Agent_count_N_u),
            index=self.fleet.ids_active,
            columns=["last_update_s"]
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
            "path_p": pd.DataFrame(self.agent.plan.task_sequence, columns=["path_p"]),
            "bundle_b": pd.DataFrame(self.agent.plan.task_bundle, columns=["bundle_b"]),

            # > Base local allocation states
            "local_bids_c": self.local_bids_c,
            "local_allocations_d": self.local_allocations_d
        }

    @property
    def additional_shared_states(self) -> dict:
        """
        Additional shared states to be published to the fleet
        """
        return {
            "winning_agents_k": self.winning_agents_k,
            "last_update_s": self.last_update_s,
            "bids_depth_e": self.bids_depth_e
        }

    @property
    def allocation_state_hash_exclusion(self) -> List[str]:
        """
        List of allocation states to exclude from the state hash
        """
        return ["last_update_s"]

    # ============================================================== METHODS
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

            # > Remove task_path_p from the state (not needed for this operation)
            state.pop("path_p")

            # > Remove bundle_b from the state (not needed for this operation)
            state.pop("bundle_b")

            # > Remove winning_bids_y from the state (not needed for this operation)
            state.pop("winning_bids_y")

            # > Remove winning_agents_k from the state (not needed for this operation)
            state.pop("winning_agents_k")

            # > Remove last_update_s from the state (operation performed separately)
            state.pop("last_update_s")
            self.last_update_s.loc[agent.id] = 0

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

            # > Remove task_path_p from the state (not needed for this operation)
            state.pop("path_p")

            # > Remove bundle_b from the state (not needed for this operation)
            state.pop("bundle_b")

            # > Remove winning_bids_y from the state (not needed for this operation)
            state.pop("winning_bids_y")

            # > Remove winning_agents_k from the state (not needed for this operation)
            state.pop("winning_agents_k")

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

            # > Remove task_path_p from the state (not needed for this operation)
            state.pop("path_p")

            # > Remove bundle_b from the state (not needed for this operation)
            state.pop("bundle_b")

            # > Remove winning_bids_y from the state (operation performed separately)
            state.pop("winning_bids_y")
            self.winning_bids_y.loc[task.id] = 0

            # > Remove winning_agents_k from the state (operation performed separately)
            state.pop("winning_agents_k")
            self.winning_agents_k.loc[task.id] = ""

            # > Remove last_update_s from the state (not needed for this operation)
            state.pop("last_update_s")

            for matrix in state.values():
                matrix.loc[task.id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)

        def terminate_task(task: Task) -> None:
            """
            Flag task as terminated in task log and remove all relevant allocation lists and matrices rows
            """

            # -> If self agent terminated task and task is current task id, only remove task
            if task.termination_source_id == self.id and task.id == self.agent.plan.current_task_id:
                self._drop_task(task_id=task.id, reset=True, forward=False, motive="termination")

            # -> Cancel goal if task is assigned to self
            elif task.id in self.agent.plan:
                self._drop_task(task_id=task.id, reset=True, forward=True, motive="termination")

            # -> Remove task from all allocation lists and matrices
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            # > Remove task_path_p from the state (operation performed separately)
            state.pop("path_p")

            # > Remove bundle_b from the state (operation performed separately)
            state.pop("bundle_b")

            # > Remove last_update_s from the state (not needed for this operation)
            state.pop("last_update_s")

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
            logger=self.get_logger(),
            id=self.agent.id
        )

        # ---- Merge received task list into local one
        task_state_change = self.tasklog.merge(
            tasklog=tasklog,
            add_task_callback=add_task,
            terminate_task_callback=terminate_task,
            tasklog_state_change_callback=None,
            prioritise_local=False,
            logger=self.get_logger()
        )

        return task_state_change, fleet_state_change

    def update_shared_states(self,
                             agent: Agent,

                             last_update_s: pd.DataFrame,

                             shared_bids_b: pd.DataFrame,
                             bids_depth_e: pd.DataFrame,
                             shared_bids_priority_beta: pd.DataFrame,

                             shared_allocations_a: pd.DataFrame,
                             shared_allocations_priority_alpha: pd.DataFrame,
                             *args,
                             **kwargs
                             ):
        """
        Update local states with received states from the fleet

        :param agent: The agent that sent the message

        :param last_update_s: Task last update matrix s received from the fleet

        :param shared_bids_b: Task bids matrix b received from the fleet
        :param bids_depth_e: Task bids depth matrix e received from the fleet
        :param shared_bids_priority_beta: Task bids priority matrix beta received from the fleet

        :param shared_allocations_a: Task allocations matrix a received from the fleet
        :param shared_allocations_priority_alpha: Task allocations priority matrix alpha received from the fleet
        """

        # ------------------------------------------------- Intercession
        # -> Get task and agent ids
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

                # ----- Priority merge with reset received current bids b into local current bids b
                if shared_bids_b.loc[task_id, agent_id] > 0:
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

                    # ----- Priority merge received bids_depth_e into local bids_depth_e
                    # > Determine correct matrix values
                    bids_depth_e_ij_updated, _ = self.priority_merge(
                            # Logging
                            task_id=task_id,
                            agent_id=agent_id,

                            # Merging
                            matrix_updated_ij=self.bids_depth_e.loc[task_id, agent_id],
                            matrix_source_ij=bids_depth_e.loc[task_id, agent_id],
                            priority_updated_ij=self.shared_bids_priority_beta.loc[task_id, agent_id],
                            priority_source_ij=shared_bids_priority_beta.loc[task_id, agent_id],

                            # Reset
                            currently_assigned=self.agent.plan.has_task(task_id=task_id),
                            reset=True
                            )

                    # > Update local states
                    self.bids_depth_e.loc[task_id, agent_id] = bids_depth_e_ij_updated

                    # -> Update local shared bids priority beta last
                    self.shared_bids_priority_beta.loc[task_id, agent_id] = shared_bids_priority_beta_ij_updated

                # ----- Priority merge received current allocations a into local current allocations a
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

    def update_task(self,
                    task: Task,
                    agent: Agent,
                    winning_bids_y: pd.DataFrame,
                    winning_agents_k: pd.DataFrame,
                    last_update_s: pd.DataFrame,
                    *args,
                    **kwargs
                    ):

        # -> Create set of agents with imposed allocations
        agents_with_imposed_allocations = set(
            self.shared_allocations_a.loc[task.id, self.shared_allocations_a.loc[task.id] == 1].index.to_list()
        )

        # -> If there are imposed allocations, find the imposed allocation with the highest priority
        if len(agents_with_imposed_allocations) > 0:
            # > Find agent with the highest priority
            winning_agent = self.shared_allocations_priority_alpha.loc[
                task.id, agents_with_imposed_allocations].idxmax()
                # TODO: Complete

        else:
            # -> Get update decision
            update_decision = _update_decision(
                k_agent_id=agent.id,
                k_winning_agent_id=winning_agents_k.loc[task.id, "winning_agents_k"],
                k_winning_bid_y_kj=winning_bids_y.loc[task.id, "winning_bids_y"],
                k_timestamp_matrix=last_update_s,

                i_agent_id=self.agent.id,
                i_winning_agent_id=self.winning_agents_k.loc[task.id, "winning_agents_k"],
                i_winning_bid_y_ij=self.winning_bids_y.loc[task.id, "winning_bids_y"],
                i_timestamp_matrix=self.last_update_s
            )

            # self.get_logger().info(f"Update decision: {update_decision}")

            # -> Apply update decision
            if update_decision == "update":
                # > Update winning bids y
                self.winning_bids_y.loc[task.id, "winning_bids_y"] = winning_bids_y.loc[task.id, "winning_bids_y"]

                # > Update winning agents k
                self.winning_agents_k.loc[task.id, "winning_agents_k"] = winning_agents_k.loc[task.id, "winning_agents_k"]

            elif update_decision == "reset":
                # > Reset winning bids y
                self.winning_bids_y.loc[task.id, "winning_bids_y"] = 0

                # > Reset winning agents k
                self.winning_agents_k.loc[task.id, "winning_agents_k"] = ""

            elif update_decision == "leave":
                pass

            else:
                raise ValueError("Invalid update decision")

            # -> Drop task and all tasks following it if decision was update or reset
            if update_decision in ["update", "reset"]:
                if self.agent.plan.has_task(task_id=task.id):
                    starting_index = self.agent.plan.task_bundle.index(task.id)

                    # > If tasks are present after the current task ...
                    if starting_index < len(self.agent.plan.task_bundle) - 1:
                        starting_index += 1

                        # -> Update all tasks following the current task
                        tasks_dropped = self.agent.plan.task_bundle[starting_index:]

                        for task_id in tasks_dropped:
                            # > Reset winning bids
                            self.winning_agents_k.loc[task_id, "winning_agents_k"] = ""

                            # > Reset winning agents
                            self.winning_bids_y.loc[task_id, "winning_bids_y"] = 0

                    self._drop_task(
                        task_id=task.id,
                        reset=False,
                        forward=True,
                        motive=update_decision,
                    )

    def update_allocation(self,
                          reset_assignment: bool = False,
                          agent: Optional[Agent] = None,
                          last_update_s: Optional[pd.DataFrame] = None,
                          *args,
                          **kwargs
                          ) -> None:
        """
        Select a task to bid for based on the current state
        1. Merge local states with shared states and update local states if necessary
        2. If no task is assigned to self, select a task
        -- If agent + last_update_s are provided
        3. Update local s

        :param reset_assignment: Flag to reset the current assignment
        :param agent: Agent object
        :param last_update_s: Task last update matrix s received from the fleet
        """

        # -------------------------------- Interceding agent logic
        if self.bid_evaluation_function is interceding_skill_based_bid_amplifier:

            # for agent in self.fleet.agents_active:
            #     self.get_logger().info(f"Agent {agent.id}: \n{agent.shared}")

            # -> Compute new bids
            for task in self.tasklog.tasks_pending:
                self._bid(task=task, agent_lst=self.fleet.agents_active)

            # -> Update b
            for task in self.tasklog.tasks_pending:
                for agent in self.fleet.agents_active:
                    # > Determine correct matrix values
                    if self.local_bids_c.loc[task.id, agent.id] > 0:
                        shared_bids_b_ij_updated, shared_bids_priority_beta_ij_updated = self.priority_merge(
                            # Logging
                            task_id=task.id,
                            agent_id=agent.id,

                            # Merging
                            matrix_updated_ij=self.shared_bids_b.loc[task.id, agent.id],
                            matrix_source_ij=self.local_bids_c.loc[task.id, agent.id],
                            priority_updated_ij=self.shared_bids_priority_beta.loc[task.id, agent.id],
                            priority_source_ij=self.hierarchy_level,

                            # Reset
                            currently_assigned=None,
                            reset=False
                        )

                        # > Update local states
                        self.shared_bids_b.loc[task.id, agent.id] = shared_bids_b_ij_updated
                        self.shared_bids_priority_beta.loc[task.id, agent.id] = shared_bids_priority_beta_ij_updated

        # if reset_assignment and self.agent.plan.current_task_id is not None:
        #     self._drop_task(task_id=self.agent.plan.current_task_id, reset=True, forward=True)

        # ---- Merge local states with shared states
        # -> If own states have changed, update local states (optimisation to avoid unnecessary updates)
        elif self.allocation_state_change:
            while len(self.agent.plan) < len(self.tasklog.tasks_pending):
                # # -> Limit bundle sizes to 5
                # if len(self.agent.plan) == 5:
                #     break

                # -> Calculate bids
                # > List tasks not in plan
                remaining_tasks = [task for task in self.tasklog.tasks_pending if task.id not in self.agent.plan]

                # > For each task not in the plan ...
                for task in remaining_tasks:
                    # -> Compute bids
                    self._bid(task=task, agent_lst=[self.agent])   # TODO: Review to better integrate intercession

                # self.local_bids_c = self.local_bids_c.sort_index().sort_index(axis=1)
                # self.shared_bids_b = self.shared_bids_b.sort_index().sort_index(axis=1)
                # self.get_logger().info(f"\nShared bids: \n{self.shared_bids_b}\nLocal bids: \n{self.local_bids_c}")

                # -> Merge local bids c into shared bids b
                # > For each task ...
                for task_id in self.tasklog.ids_pending:
                    # > For each agent ...
                    for agent_id in self.fleet.ids_active:
                        # -> Priority merge local bids c into current bids b
                        if self.local_bids_c.loc[task_id, agent_id] > 0:
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
                                currently_assigned=None,
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

                        # -> Merge current bids b into local bids c according to intercession depth e
                        if self.bids_depth_e.loc[task_id, agent_id] == DEEP:
                            self.local_bids_c.loc[task_id, agent_id] = self.shared_bids_b.loc[task_id, agent_id]

                # ---- Select task
                # -> Create list of valid tasks (pandas series of task ids initialised as 0)
                valid_tasks_list_h = pd.DataFrame(
                    np.zeros(len(remaining_tasks)),
                    index=[task.id for task in remaining_tasks],
                    columns=["valid_tasks_list_h"]
                )

                # > For each task ...
                for task in remaining_tasks:
                    # -> Create set of agents with imposed allocations
                    agents_with_imposed_allocations = list(
                        set(self.shared_allocations_a.loc[task.id, self.shared_allocations_a.loc[task.id] == 1].index))

                    # > If there are imposed allocations, check if self is imposed allocation with the highest priority
                    if len(agents_with_imposed_allocations) > 0:
                        # -> Find agent with the highest priority
                        winning_agent = self.shared_allocations_priority_alpha.loc[
                            task.id, agents_with_imposed_allocations].idxmax()

                        # TODO: Adjust to handle conflicting imposed allocations. For now assumed that allocation intercession is conflict-free

                        # > If self is the winning agent, add the task to the task list
                        if winning_agent == self.id:
                            valid_tasks_list_h[task.id, "valid_tasks_list_h"] = 1
                    else:
                        # > If there are no imposed allocations, check if self has the highest bid and is not blacklisted
                        valid_task = int(
                            self.shared_allocations_a.loc[task.id, self.id] != -1 and self.shared_bids_b.loc[task.id, self.id] >
                            self.winning_bids_y.loc[task.id, "winning_bids_y"])

                        valid_tasks_list_h.loc[task.id, "valid_tasks_list_h"] = valid_task

                # -> If there are valid tasks, select the task with the highest bid
                if valid_tasks_list_h["valid_tasks_list_h"].sum() > 0:
                    # -> Select task with the highest bid
                    # > Get index of all the tasks for which the valid_tasks_list_h is 1
                    valid_tasks = valid_tasks_list_h[valid_tasks_list_h["valid_tasks_list_h"] == 1].index.to_list()

                    # > Get valid task with largest bid (using local bids !!!)
                    selected_task_id = self.local_bids_c.loc[valid_tasks, self.id].idxmax()

                    # -> Limit bundle sizes to 5
                    # > If max bundle size reached and selected task not at agent location
                    if len(self.agent.plan) >= 3 and self.agent.state.pos != [self.tasklog[selected_task_id].instructions["x"], self.tasklog[selected_task_id].instructions["y"]]:
                        # -> Break while loop
                        break

                    # self.get_logger().warning(f"Task {selected_task_id}: bid {self.local_bids_c.loc[selected_task_id, self.id]}")

                    # -> Update winning bids
                    self.winning_bids_y.loc[selected_task_id, "winning_bids_y"] = self.shared_bids_b.loc[selected_task_id, self.id]

                    # -> Update winning agents
                    self.winning_agents_k.loc[selected_task_id, "winning_agents_k"] = self.agent.id

                    # -> Add task to plan
                    self.agent.add_task_to_plan(
                        tasklog=self.tasklog,
                        task=selected_task_id,
                        position=self.agent.local["insertions"][selected_task_id],
                        bid=f"c:{round(self.local_bids_c.loc[selected_task_id, self.id], 4)}, b:{round(self.shared_bids_b.loc[selected_task_id, self.id], 4)}",
                        logger=self.get_logger()
                    )

                    # > Publish goal msg
                    self.publish_goal_msg(meta_action="update", traceback="Select task")

                # -> If there are no valid tasks, break the while loop
                else:
                    break

        # ---- Update local states
        if agent is not None and last_update_s is not None:
            # -> Merge received last update s into local last update s, keep the latest timestamps
            merged_df = self.last_update_s.combine_first(last_update_s)
            self.last_update_s["last_update_s"] = merged_df["last_update_s"].combine(last_update_s["last_update_s"], max)

            # -> Update last update s of received agent to latest
            if agent.state.timestamp > self.last_update_s.loc[agent.id, "last_update_s"]:
                self.last_update_s.loc[agent.id, "last_update_s"] = agent.state.timestamp

    def _bid(self, task: Task, agent_lst: list[Agent]) -> list[dict]:
        """
        Find the largest marginal gain achieved from inserting a task into the plan at the most beneficial position

        :param task: Task to bid for
        :param agent_lst: List of agents to compute bids for

        :return: List of dictionaries containing the agent(s) ID(s) and corresponding marginal gains according to insertion position
        """

        if self.environment is None:
            # -> Return 0 bids for all agents as the environment is not available
            self.get_logger().warning("!!!!!! WARNING: Environment not available")
            return []

        # -> If no bid evaluation function, return empty list
        if self.bid_evaluation_function is None:
            return []

        # -> Compute the marginal gains for the agent
        agents_marginal_gains = self.bid_evaluation_function(
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
            # tasklog=self.tasklog,
            # fleet=self.fleet,
            # environment=self.environment,
        )

        # > For each agent ...
        for agent_bid_dict in agents_marginal_gains:
            agent_id = agent_bid_dict["agent_id"]

            max_marginal_gain = None
            insertion_loc = None

            for i, marginal_gain in agent_bid_dict["marginal_gains"].items():
                if max_marginal_gain is None:
                    max_marginal_gain = marginal_gain
                    insertion_loc = i

                elif marginal_gain["value"] > max_marginal_gain["value"]:
                    max_marginal_gain = marginal_gain
                    insertion_loc = i

            # -> Store bid to local bids matrix
            if max_marginal_gain is not None:
                # > Bid
                self.local_bids_c.loc[task.id, agent_id] = max_marginal_gain["value"]

                # > Bid depth
                self.bids_depth_e.loc[task.id, agent_id] = max_marginal_gain["bids_depth"]

                # > Allocation
                self.local_allocations_d.loc[task.id, agent_id] = max_marginal_gain["allocation"]

                # -> Store insertion to agent local
                if "insertions" not in self.fleet[agent_id].local.keys():
                    self.fleet[agent_id].local['insertions'] = {}

                self.fleet[agent_id].local['insertions'][task.id] = insertion_loc

        return agents_marginal_gains

    def _drop_task(self,
                   task_id: str,
                   reset: bool = False,
                   forward: bool = True,     # Must default to true to work with base priority merge method
                   traceback: str = None,
                   motive: Optional[str] = None,
                   logger=True
                   ) -> None:
        """
        Drop a task from the bundle list and plan

        :param task_id: Id of task to drop
        :param reset: Flag to reset the winning bids y for the task
        :param forward: Whether to drop the task and all following tasks in the plan
        :param traceback: The reason for dropping the task
        """

        if reset:
            # > Reset winning bids
            self.winning_bids_y.loc[task_id, "winning_bids_y"] = 0

            # > Reset winning agents
            self.winning_agents_k.loc[task_id, "winning_agents_k"] = ""

        if reset and forward:
            # -> Update all tasks following the current task
            tasks_dropped = self.agent.plan.task_bundle[self.agent.plan.task_bundle.index(task_id):]

            for task_id in tasks_dropped:
                # > Reset winning bids
                self.winning_agents_k.loc[task_id, "winning_agents_k"] = ""

                # > Reset winning agents
                self.winning_bids_y.loc[task_id, "winning_bids_y"] = 0

        # > Remove the task from the plan
        self.agent.remove_task_from_plan(
            tasklog=self.tasklog,
            task=task_id,
            forward=forward,
            motive=motive,
            logger=self.get_logger() if logger else None
        )

        # > Publish goal msg
        self.publish_goal_msg(meta_action="update", traceback=traceback)

    # >>>> Prints
