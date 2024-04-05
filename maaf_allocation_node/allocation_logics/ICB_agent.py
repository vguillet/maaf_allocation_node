
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
from abc import ABC, abstractmethod

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

from maaf_tools.datastructures.task.Task import Task
from maaf_tools.datastructures.task.TaskLog import TaskLog

from maaf_tools.datastructures.agent.Agent import Agent
from maaf_tools.datastructures.agent.Fleet import Fleet
from maaf_tools.datastructures.agent.Plan import Plan

from maaf_tools.tools import *

##################################################################################################################


class ICBAgent(MAAFAgent):
    def __init__(self,
                 node_name: str,
                 id: str = None,
                 name: str = None,
                 skillset: List[str] = None,
                 bid_estimator=None
                 ):
        # ---- Init parent class
        MAAFAgent.__init__(
            self,
            node_name=node_name,
            id=id,
            name=name,
            skillset=skillset,
            bid_estimator=bid_estimator
        )

    # ============================================================== PROPERTIES

    # ============================================================== METHODS
    # ---------------- Callbacks
    def _task_msg_subscriber_callback(self, task_msg) -> None:
        """
        Callback for task messages: create new task, add to local tasks and update local states, and select new task

        :param task_msg: TaskMsgStamped message
        """

        # -> Check if the message is for the agent
        msg_target = task_msg.target

        # -> If the message is not for the agent...
        if msg_target != self.id and msg_target != "all":
            return

        # -> Unpack msg
        task_dict = loads(task_msg.memo)

        # -> Ensure id is a string
        task_dict["id"] = str(task_dict["id"])

        # -> Create task object
        task = Task.from_dict(task_dict, partial=True)

        if task_msg.meta_action == "pending":
            self.get_logger().info(f"{self.id} * Found new task: {task.id} (Type: {task.type}) - Pending task count: {len(self.tasklog.ids_pending) + 1}")

        elif task_msg.meta_action == "completed":
            self.get_logger().info(f"{self.id} v Task {task.id} completed - Pending task count: {len(self.tasklog.ids_pending) - 1}")

        elif task_msg.meta_action == "cancelled":
            self.get_logger().info(f"{self.id} x Task {task.id} cancelled - Pending task count: {len(self.tasklog.ids_pending) - 1}")

        # -> Create new task
        if task_msg.meta_action == "pending":
            # -> Pass if task is already in the task log
            if task.id in self.tasklog.ids:
                return

        #     # TODO: Remove this line once task creation handles stamp creation
        #     task.creation_timestamp = self.current_timestamp
        #
        # elif task_msg.meta_action == "completed":
        #     task.termination_timestamp = self.current_timestamp
        #     # TODO: Finish implementing task completion
        #
        # elif task_msg.meta_action == "cancelled":
        #     task.termination_timestamp = self.current_timestamp
        #     # TODO: Implement task cancel
        #     pass

        # -> Update situation awareness
        tasklog = TaskLog()
        tasklog.add_task(task=task)

        task_state_change, fleet_state_change = self.update_situation_awareness(tasklog=tasklog, fleet=None)

        # -> Select task
        self.update_allocation(reset_assignment=task_state_change and self.scenario.recompute_bids_on_state_change)   # TODO: Cleanup

        # -> Update previous state hash
        self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

        # -> If state has changed, update local states (only publish when necessary)
        self.publish_allocation_state_msg()

    def _team_msg_subscriber_callback(self, team_msg) -> None:
        """
        Callback for team messages: consensus phase of the CBAA algorithm

        :param team_msg: TeamComm message
        """
        # ---- Unpack msg
        # -> Ignore self messages
        if team_msg.source == self.id:
            return

        # -> Check if the message is for the agent
        msg_target = team_msg.target

        # # -> If the message is not for the agent...
        if msg_target != self.id and msg_target != "all":
            # -> Check if the agent should rebroadcast the message
            # msg, rebroadcast = self.rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub)
            return

        # -> Deserialise allocation state
        received_allocation_state = self.deserialise(state=team_msg.memo)

        # -> Update local situation awareness
        # > Convert received serialised tasklog to tasklog
        received_tasklog = TaskLog.from_dict(received_allocation_state["tasklog"])

        # > Convert received serialised fleet to fleet
        received_fleet = Fleet.from_dict(received_allocation_state["fleet"])

        task_state_change, fleet_state_change = self.update_situation_awareness(
            tasklog=received_tasklog,
            fleet=received_fleet
        )

        # -> Update shared states
        self.update_shared_states(**received_allocation_state)

        # -> Update the task in the task list x the agent is assigned to
        for task in received_tasklog:
            if task.id not in self.tasklog.ids_pending:
                continue

            task_id = task.id

            if task_id in self.agent.plan:
                self.update_task(
                    received_winning_bids_y=received_allocation_state["winning_bids_y"],
                    task_id=task_id
                )

        # -> Select new task
        self.update_allocation(reset_assignment=task_state_change and self.scenario.recompute_bids_on_state_change) # TODO: Cleanup

        # -> If state has changed, update local states (only publish when necessary)
        self.check_publish_state_change()

        # else:
        #     # -> Check if the agent should rebroadcast the message
        #     msg, rebroadcast = self.rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub)

    # ---------------- Processes
    def update_task(
            self,
            received_winning_bids_y: pd.DataFrame,
            task_id: str
    ) -> None:
        """
        Update current task based on received winning bids and updated allocation intercession from the fleet

        :param received_winning_bids_y: Winning bids list y received from the fleet
        :param task_id: task id
        """

        # -> Create set of agents with imposed allocations
        agents_with_imposed_allocations = set(
            self.shared_allocations_a.loc[task_id, self.shared_allocations_a.loc[task_id] == 1].index.to_list()
        )

        # -> If there are imposed allocations, find the imposed allocation with the highest priority
        if len(agents_with_imposed_allocations) > 0:
            # -> Find agent with the highest priority
            winning_agent = self.shared_allocations_priority_alpha.loc[
                task_id, agents_with_imposed_allocations].idxmax()

        else:
            # -> Compare received winning bids with local winning bids
            winning_agent = self.id if self.winning_bids_y.loc[task_id, "winning_bids_y"] >= \
                                       received_winning_bids_y.loc[task_id, "winning_bids_y"] else None

            # TODO: Fix self.winning_bids_y.loc[task_id] > received_winning_bids_y.loc[task_id] scenario. For now it is assumed that two agents cannot have the same bid for the same task

        # -> Update task list x
        # > If the winning agent is not the agent, remove the task from the task list
        if winning_agent is not self.id:
            # -> Cancel goal
            self.drop_task(
                task_id=task_id,
                reset=False,
                traceback="Task update",
                logger=True
            )

    @abstractmethod
    def update_allocation(self, reset_assignment: bool = False) -> None:
        """
        Select a task to bid for based on the current state
        1. Merge local states with shared states and update local states if necessary
        2. If no task is assigned to self, select a task

        :param reset_assignment: Flag to reset the current assignment
        """
        raise NotImplementedError
