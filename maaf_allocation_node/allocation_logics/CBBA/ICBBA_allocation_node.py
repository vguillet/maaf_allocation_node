
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
from orchestra_config.orchestra_config import *     # KEEP THIS LINE, DO NOT REMOVE
from orchestra_config.sim_config import *

from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
from maaf_allocation_node.maaf_agent import MAAFAgent
from maaf_allocation_node.allocation_logics.ICB_agent import ICBAgent

from maaf_tools.datastructures.task.Task import Task
from maaf_tools.datastructures.task.TaskLog import TaskLog

from maaf_tools.datastructures.agent.Agent import Agent
from maaf_tools.datastructures.agent.Fleet import Fleet
from maaf_tools.datastructures.agent.Plan import Plan

from maaf_tools.tools import *

from maaf_allocation_node.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid
from maaf_allocation_node.bidding_logics.anticipated_action_task_interceding_agent import anticipated_action_task_interceding_agent

##################################################################################################################


class ICBBANode(ICBAgent):
    def __init__(self):
        # ---- Init parent class
        ICBAgent.__init__(
            self,
            node_name="CBBAwI_allocation_node",
            skillset=None
        )

        # ----------------------------------- Agent allocation states
        # -> Setup additional CBAA-specific allocation states
        self._setup_allocation_additional_states()

        # -> Initialise previous state hash
        self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

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
        return
        # """
        # Local path list p of the agent
        #
        # List is a pandas dataframe with variable size, initialised empty, with task ids ordered in the sequence the agent will execute them.
        # Corresponds to the task_bundle of the agent's plan.
        # """
        # self.task_path_p = pd.DataFrame(
        #     columns=["task_path_p"]
        # )
        #
        # """
        # Local bundle b of the agent
        #
        # List is a pandas dataframe with variable size, initialised empty, with task ids ordered in the sequence the agent added them to its plan
        # """
        # self.bundle_b = pd.DataFrame(
        #     columns=["bundle_b"]
        # )

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

            for matrix in state.values():
                matrix.loc[task.id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)

        def terminate_task(task: Task) -> None:
            """
            Flag task as terminated in task log and remove all relevant allocation lists and matrices rows
            """

            # -> If self agent terminated task and task is current task id, only remove task
            if task.termination_source_id == self.id and task.id == self.agent.plan.current_task_id:
                self.drop_task(task_id=task.id, forward=False)

            # -> Cancel goal if task is assigned to self
            elif task.id in self.agent.plan:
                self.drop_task(task_id=task.id, forward=True)

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
            tasklog_state_change_callback=self.compute_bids,
            prioritise_local=False,
            logger=self.get_logger()
        )

        return task_state_change, fleet_state_change

    def update_shared_states(
            self,
            received_shared_bids_b,
            received_shared_bids_priority_beta,
            received_shared_allocations_a,
            received_shared_allocations_priority_alpha
    ):
        """
        Update local states with received states from the fleet

        :param received_shared_bids_b: Task bids matrix b received from the fleet
        :param received_shared_bids_priority_beta: Task bids priority matrix beta received from the fleet
        :param received_shared_allocations_a: Task allocations matrix a received from the fleet
        :param received_shared_allocations_priority_alpha: Task allocations priority matrix alpha received from the fleet
        """

        raise NotImplementedError

    def update_allocation(self, reset_assignment: bool = False) -> None:
        """
        Select a task to bid for based on the current state
        1. Merge local states with shared states and update local states if necessary
        2. If no task is assigned to self, select a task

        :param reset_assignment: Flag to reset the current assignment
        """
        # ---- Merge local states with shared states
        # -> If own states have changed, update local states (optimisation to avoid unnecessary updates)
        if self.allocation_state_change:
            while len(self.agent.plan) < len(self.tasklog.pending_tasks):
                # > List tasks not in plan
                remaining_tasks = [task for task in self.tasklog.pending_tasks if task.id not in self.agent.plan]

                # > For each task not in the plan ...
                for task in remaining_tasks:
                    # -> Compute bids
                    self.bid(task=task, agent=self.agent)   # TODO: Review to better integrate intercession

    def bid(self, task: Task, agent: Agent) -> float:
        """
        Find the largest marginal gain achieved from inserting a task into the plan at the most beneficial position

        :param task: Task to bid for
        :param agent: Agent to bid for
        """

        if self.env is None:
            # -> Return 0 bids for all agents as the environment is not available
            self.get_logger().warning("!!!!!! WARNING: Environment not available")
            return []

        # -> If no bid evaluation function, return empty list
        if self.bid_evaluation_function is None:
            return []


    def drop_task(self,
                  task_id: str,
                  reset: bool = False,
                  forward: bool = True, # Defaults to true to work with base priority merge method
                  traceback: str = None,
                  logger=True
                  ) -> None:
        """
        Drop a task from the bundle list and plan

        :param task_id: Id of task to drop
        :param reset: Flag to reset the winning bids y for the task
        :param forward: Whether to drop the task and all following tasks in the plan
        :param traceback: The reason for dropping the task
        :param logger: Whether to log the task drop
        """

        if reset:
            # > Reset winning bids
            self.winning_bids_y.loc[task_id, "winning_bids_y"] = 0

        # > Remove the task from the plan
        self.agent.remove_task_from_plan(
            tasklog=self.tasklog,
            task=task_id,
            forward=forward,
            logger=self.get_logger() if logger else None
        )

        # > Publish goal msg
        self.publish_goal_msg(meta_action="update", traceback=traceback)

    # >>>> Prints
