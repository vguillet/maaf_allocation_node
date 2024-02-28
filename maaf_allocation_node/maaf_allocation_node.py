
##################################################################################################################

"""
This module contains the MAAF allocation node class, which is a ROS2 node that implements the CBAA algorithm for task allocation.
"""

# Built-in/Generic Imports
import os
from random import randint
from json import dumps, loads
from typing import List, Optional
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

# Local Imports
from .node_config import *
from .maaf_agent import MAAFAgent
from .dataclass_cores import maaf_list_dataclass
from .task_dataclasses import Task, Task_log
from .fleet_dataclasses import Agent, Fleet
from .state_dataclasses import Agent_state
from .tools import *
from maaf_msgs.msg import TeamCommStamped, Bid, Allocation

##################################################################################################################


class maaf_allocation_node(MAAFAgent):
    def __init__(self):

        # ---- Init parent class
        MAAFAgent.__init__(
            self,
            node_name="CBAAwI_allocation_node",
            skillset=["GOTO"]
        )

        # -----------------------------------  Agent allocation states
        # -> Setup additional CBAA-specific allocation states
        self.__setup_allocation_additional_states()

        # -> Initialise previous state hash
        self.__prev_state_hash_dict = deepcopy(self.state_hash_dict)

        # -----------------------------------  Confirm initialisation
        self.get_logger().info(f"MAAF agent {self.id}: Allocation node initialised ({ALLOCATION_METHOD})")

    # ============================================================== PROPERTIES
    def __setup_allocation_additional_states(self) -> None:
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
            index=self.task_log.ids_pending,
            columns=["task_list_x"]
        )

    # ============================================================== PROPERTIES
    # ---------------- Self state
    @property
    def local_allocation_state_dict(self) -> dict:
        """
        Local allocation state at current time step (serialised)

        :return: dict
        """
        return {
            "task_list_x": self.task_list_x.to_dict(),
            "local_bids_c": self.local_bids_c.to_dict(),
            "local_allocations_d": self.local_allocations_d.to_dict()
        }

    # >>>> Change tracking
    @property
    def allocation_state_change(self) -> bool:
        """
        Check if the shared allocation state has changed

        :return: bool
        """
        # -> Compare hash of current state with hash of previous state
        for key, value in self.state_hash_dict.items():
            # -> If any value has changed, return True
            if value != self.__prev_state_hash_dict[key]:
                return True

        # -> If no value has changed, return False
        return False

    @property
    def state_hash_dict(self) -> dict:
        """
        Hash of the agent state

        :return: dict
        """
        # -> Convert to series and dataframe to immutable hashable objects
        immutable_state = {}

        for key, value in self.state.items():
            if isinstance(value, pd.Series):
                immutable_state[key] = hash(str(value.to_string()))
            elif isinstance(value, pd.DataFrame):
                immutable_state[key] = hash(str(value.to_string()))
            else:
                immutable_state[key] = hash(str(value))

        return immutable_state

    # ============================================================== METHODS
    # ---------------- Callbacks
    # >>>> Base
    def bid_subscriber_callback(self, bid_msg: Bid) -> None:
        """
        Callback for the bid subscriber

        :param bid_msg: Bid message
        """

        self.get_logger().info(f"{self.id} < Received bid: \n    Task id: {bid_msg.task_id}\n    Agent id: {bid_msg.target_agent_id}\n    Value: {bid_msg.value}\n    Priority: {bid_msg.priority}")

        # -> Check if bid is for a task the agent is aware of
        if bid_msg.task_id not in self.task_log.ids:
            self.get_logger().info(f"!!! WARNING: Received bid for task {bid_msg.task_id} not in task log")
            return
        # -> Check if bid is for an agent the agent is aware of
        elif bid_msg.target_agent_id not in self.fleet.ids_active:
            self.get_logger().info(f"!!! WARNING: Received bid for agent {bid_msg.target_agent_id} not in fleet")
            return

        # -> Priority merge received bid into current bids b
        tasks_value_x_ij_updated, current_bids_b_ij_updated, current_bids_priority_beta_ij_updated = (
            self.priority_merge(
                # Logging
                task_id=bid_msg.task_id,
                agent_id=bid_msg.target_agent_id,

                # Merging
                matrix_updated_ij=self.current_bids_b.loc[bid_msg.task_id, bid_msg.target_agent_id],
                matrix_source_ij=bid_msg.value,
                priority_updated_ij=self.current_bids_priority_beta.loc[bid_msg.task_id, bid_msg.target_agent_id],
                priority_source_ij=bid_msg.priority,

                # Reset
                tasks_value_x_ij=self.task_list_x.loc[bid_msg.task_id, "task_list_x"],
                reset=True
            )
        )

        # > Update local states
        self.current_bids_b.loc[bid_msg.task_id, bid_msg.target_agent_id] = current_bids_b_ij_updated
        self.current_bids_priority_beta.loc[bid_msg.task_id, bid_msg.target_agent_id] = current_bids_priority_beta_ij_updated
        self.task_list_x.loc[bid_msg.task_id, "task_list_x"] = tasks_value_x_ij_updated

        # -> Select task
        self.select_task()

    def allocation_subscriber_callback(self, allocation_msg: Allocation) -> None:
        """
        Callback for the allocation subscriber

        :param allocation_msg: Allocation message
        """

        self.get_logger().info(f"{self.id} < Received allocation: \n    Task id: {allocation_msg.task_id}\n    Agent id: {allocation_msg.target_agent_id}\n    Action: {allocation_msg.action}\n    Priority: {allocation_msg.priority}")

        # -> Check if bid is for a task the agent is aware of
        if allocation_msg.task_id not in self.task_log.ids:
            self.get_logger().info(f"!!! WARNING: Received allocation for task {allocation_msg.task_id} not in task log")
            return
        # -> Check if bid is for an agent the agent is aware of
        elif allocation_msg.target_agent_id not in self.fleet.ids_active:
            self.get_logger().info(f"!!! WARNING: Received allocation for agent {allocation_msg.target_agent_id} not in fleet")
            return

        # -> Convert allocation action to
        allocation_state = self.action_to_allocation_state(action=allocation_msg.action)

        if allocation_state is not None:
            # -> Merge received allocation into current allocation
            tasks_value_x_ij_updated, current_allocations_a_ij_updated, current_allocations_priority_alpha_ij_updated = (
                self.priority_merge(
                    # Logging
                    task_id=allocation_msg.task_id,
                    agent_id=allocation_msg.target_agent_id,

                    # Merging
                    matrix_updated_ij=self.current_allocations_a.loc[allocation_msg.task_id, allocation_msg.target_agent_id],
                    matrix_source_ij=allocation_state,
                    priority_updated_ij=self.current_allocations_priority_alpha.loc[allocation_msg.task_id, allocation_msg.target_agent_id],
                    priority_source_ij=allocation_msg.priority,

                    # Reset
                    tasks_value_x_ij=self.task_list_x.loc[allocation_msg.task_id, "task_list_x"],
                    reset=True
                )
            )

            # > Update local states
            self.current_allocations_a.loc[allocation_msg.task_id, allocation_msg.target_agent_id] = current_allocations_a_ij_updated
            self.current_allocations_priority_alpha.loc[allocation_msg.task_id, allocation_msg.target_agent_id] = current_allocations_priority_alpha_ij_updated
            self.task_list_x.loc[allocation_msg.task_id, "task_list_x"] = tasks_value_x_ij_updated

            # -> Select task
            self.select_task()

    def task_msg_subscriber_callback(self, task_msg) -> None:
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
        task_dict = loads(task_msg.memo)  # Task from task factory

        # -> Ensure id is a string
        task_dict["id"] = str(task_dict["id"])

        # TODO: Remove this line once task creation handles stamp creation
        task_dict["creation_timestamp"] = self.current_timestamp

        # -> Create task object
        task = Task.from_dict(task_dict)

        # -> Create new task
        if task_msg.meta_action == "pending":
            # -> Pass if task is already in the task log
            if task.id in self.task_log.ids:
                return

            self.get_logger().info(f"{self.id}   Found new task: {task.id} (Type: {task.type}) - Pending task count: {len(self.task_log.ids_pending)}")

        elif task_msg.meta_action == "completed":
            task_dict["termination_timestamp"] = self.current_timestamp

            self.get_logger().info(f"{self.id}   Task {task.id} completed - Pending task count: {len(self.task_log.ids_pending)-1}")
            # TODO: Finish implementing task completion

        elif task_msg.meta_action == "cancelled":
            # TODO: Implement task cancel
            pass

        # -> Update situation awareness
        self.__update_situation_awareness(task_list=[task], fleet=None)

        # -> Select task
        self.select_task()

        # -> Update previous state hash
        self.__prev_state_hash_dict = deepcopy(self.state_hash_dict)

        # -> Publish allocation state to the fleet to share the new task
        self.publish_allocation_state_msg()

    def team_msg_subscriber_callback(self, team_msg) -> None:
        """
        Callback for team messages: consensus phase of the CBAA algorithm

        :param team_msg: TeamComm message
        """
        # ---- Unpack msg
        # -> Check if the message is for the agent
        msg_target = team_msg.target

        # -> If the message is not for the agent...
        if msg_target != self.id and msg_target != "all":
            # -> Check if the agent should rebroadcast the message
            msg, rebroadcast = self.rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub)
            return

        # -> Deserialise allocation state
        received_allocation_state = self.deserialise(state=team_msg.memo)

        # -> Update local situation awareness
        # > Convert task list to Task objects
        received_task_log = [Task.from_dict(task) for task in received_allocation_state["tasks"]]

        # > Convert fleet to Agent objects
        received_fleet = [Agent.from_dict(agent) for agent in received_allocation_state["fleet"]]

        self.__update_situation_awareness(
            task_list=received_task_log,
            fleet=received_fleet
        )

        # -> Update shared states
        self.__update_shared_states(
            received_current_bids_b=received_allocation_state["current_bids_b"],
            received_current_bids_priority_beta=received_allocation_state["current_bids_priority_beta"],
            received_current_allocations_a=received_allocation_state["current_allocations_a"],
            received_current_allocations_priority_alpha=received_allocation_state["current_allocations_priority_alpha"]
        )

        # -> Update the task in the task list x the agent is assigned to
        for task in received_task_log:
            if task.id not in self.task_log.ids_pending:
                continue

            task_id = task.id

            if self.task_list_x.loc[task_id, "task_list_x"] == 1:
                self.__update_task(
                    received_winning_bids_y=received_allocation_state["winning_bids_y"],
                    task_id=task_id
                )

        # -> Select new task
        self.select_task()

        # -> If state has changed, update local states (only publish when necessary)
        if self.allocation_state_change:
            # -> Update previous state hash
            self.__prev_state_hash_dict = deepcopy(self.state_hash_dict)

            # -> Publish allocation state to the fleet
            self.publish_allocation_state_msg()

        else:
            # -> Check if the agent should rebroadcast the message
            msg, rebroadcast = self.rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub)

    # >>>> CBAA
    def __update_situation_awareness(self, task_list: Optional[List[Task]], fleet: Optional[List[Agent]]) -> None:
        """
        Update local states with new tasks and fleet dicts. Add new tasks and agents to local states and extend
        local states with new rows and columns for new tasks and agents. Remove tasks and agents from local states if
        they are no longer in the task list or fleet.

        :param task_list: Tasks dict
        :param fleet: Fleet dict
        """
        
        def add_agent(agent: Agent) -> None:
            """
            Add new agent to local fleet and extend local states with new columns for new agent
            """
            # -> Add new agent to local fleet
            self.fleet.add_agent(agent=agent)

            # -> Add a column to all relevant allocation lists and matrices with new agent
            agent_id = agent.id

            # > Local
            self.local_bids_c[agent_id] = pd.Series(np.zeros(self.Task_count_N_t), index=self.task_log.ids_pending)
            self.local_allocations_d[agent_id] = pd.Series(np.zeros(self.Task_count_N_t), index=self.task_log.ids_pending)

            # > Shared
            self.current_bids_b[agent_id] = pd.Series(np.zeros(self.Task_count_N_t), index=self.task_log.ids_pending)
            self.current_bids_priority_beta[agent_id] = pd.Series(np.zeros(self.Task_count_N_t), index=self.task_log.ids_pending)

            self.current_allocations_a[agent_id] = pd.Series(np.zeros(self.Task_count_N_t), index=self.task_log.ids_pending)
            self.current_allocations_priority_alpha[agent_id] = pd.Series(np.zeros(self.Task_count_N_t), index=self.task_log.ids_pending)

        def remove_agent(agent: Agent) -> None:
            """
            Remove agent from local fleet and all relevant allocation lists and matrices
            """
            # -> Remove agent from local fleet
            # > Flag agent is inactive
            self.fleet.set_agent_state(agent=agent, state=agent.state)

            # TODO: Figure out how to deal with removing bids from the winning bid matrix (full reset vs bid owner tracking). For now, reset winning bids matrix (full reset)
            self.winning_bids_y = pd.Series(np.zeros(self.Task_count_N_t), index=self.task_log.ids_pending)

            # -> Remove agent from all relevant allocation lists and matrices
            agent_id = agent.id

            # > Local
            self.local_bids_c.drop(columns=agent_id, inplace=True)
            self.local_allocations_d.drop(columns=agent_id, inplace=True)

            # > Shared
            self.current_bids_b.drop(columns=agent_id, inplace=True)
            self.current_bids_priority_beta.drop(columns=agent_id, inplace=True)

            self.current_allocations_a.drop(columns=agent_id, inplace=True)
            self.current_allocations_priority_alpha.drop(columns=agent_id, inplace=True)

        def add_task(task: Task) -> None:
            """
            Add new task to local tasks and extend local states with new rows for new task
            """
            # -> Add new task to local tasks
            self.task_log.add_task(task=task)

            # -> Add a row to all allocation lists and matrices with new task
            task_id = task.id
            
            # > Local
            self.task_list_x.loc[task_id] = 0
            self.local_bids_c.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)
            self.local_allocations_d.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)

            # > Shared
            self.winning_bids_y.loc[task_id] = 0

            self.current_bids_b.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)
            self.current_bids_priority_beta.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)

            self.current_allocations_a.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)
            self.current_allocations_priority_alpha.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u),
                                                                             index=self.fleet.ids_active)

            # -> Estimate bid(s) for new task
            task_bids = self.bid(task=self.task_log[task_id], agent_lst=[self.agent])

            # -> Store bids to local bids matrix
            for bid in task_bids:
                self.local_bids_c.loc[task_id, bid["agent_id"]] = bid["bid"]

        def terminate_task(task: Task) -> None:
            # -> Update task log
            self.task_log.update_item_fields(
                item=task.id,
                field_value_pair={
                    "status": task.status,  # Termination reason: completed, cancelled, failed
                    "termination_timestamp": task.termination_timestamp
                }
            )
            
            # -> Remove task from all allocation lists and matrices
            # > Local
            self.task_list_x.drop(index=task.id, inplace=True)
            self.local_bids_c.drop(index=task.id, inplace=True)
            self.local_allocations_d.drop(index=task.id, inplace=True)

            # > Shared
            self.winning_bids_y.drop(index=task.id, inplace=True)

            self.current_bids_b.drop(index=task.id, inplace=True)
            self.current_bids_priority_beta.drop(index=task.id, inplace=True)

            self.current_allocations_a.drop(index=task.id, inplace=True)
            self.current_allocations_priority_alpha.drop(index=task.id, inplace=True)

        # self.print_state()

        # ---- Add new tasks and agents
        # -> Update local fleet
        if fleet is not None:
            for agent in fleet:
                # -> If the agent is not in the local fleet ...
                if agent.id not in self.fleet.ids:
                    # -> If the agent is active, add to fleet and extend local states with new columns
                    if agent.state.status == "active":
                        add_agent(agent=agent)

                    # -> If the agent is inactive, only add to the local fleet
                    else:
                        self.fleet.add_agent(agent=agent)

                # -> Else if the new agent state is more recent than the local agent state, update
                elif agent.state.timestamp > self.fleet[agent.id].state.timestamp:
                    # -> If the agent is active, update the agent state in the fleet to the latest state
                    if agent.state.status == "active":
                        self.fleet.set_agent_state(agent=agent, state=agent.state)

                    # -> If the agent is inactive, update state and remove agent from local states
                    else:
                        remove_agent(agent=agent)

        # -> Update task log and local states
        if task_list is not None:
            for task in task_list:
                # -> If the task is not in the task log ...
                if task.id not in self.task_log.ids:
                    # -> If the task is pending, add to task log and extend local states with new rows
                    if task.status == "pending":
                        add_task(task=task)

                    # -> If the task is completed, only add to the task log
                    else:
                        self.task_log.add_task(task=task)
                
                # -> Else if the task is in the task log and is not pending, flag as terminated in task log and remove rows from the local states
                elif task.status != "pending" and self.task_log[task.id].status == "pending":
                    terminate_task(task=task)

    def __update_shared_states(
            self,
            received_current_bids_b,
            received_current_bids_priority_beta,
            received_current_allocations_a,
            received_current_allocations_priority_alpha
    ):
        """
        Update local states with received states from the fleet

        :param received_current_bids_b: Task bids matrix b received from the fleet
        :param received_current_bids_priority_beta: Task bids priority matrix beta received from the fleet
        :param received_current_allocations_a: Task allocations matrix a received from the fleet
        :param received_current_allocations_priority_alpha: Task allocations priority matrix alpha received from the fleet
        """

        received_tasks_ids = list(received_current_bids_b.index)
        received_agent_ids = list(received_current_bids_b.columns)

        # -> For each task ...
        for task_id in received_tasks_ids:
            # -> If the task has been terminated, skip
            if task_id not in self.task_log.ids_pending:
                continue

            # -> for each agent ...
            for agent_id in received_agent_ids:
                #
                # if agent_id not in self.fleet.ids_active:
                #     continue

                # -> Priority merge with reset received current bids b into local current bids b
                # > Determine correct matrix values
                tasks_value_x_ij_updated, current_bids_b_ij_updated, current_bids_priority_beta_ij_updated = (
                    self.priority_merge(
                        # Logging
                        task_id=task_id,
                        agent_id=agent_id,

                        # Merging
                        matrix_updated_ij=self.current_bids_b.loc[task_id, agent_id],
                        matrix_source_ij=received_current_bids_b.loc[task_id, agent_id],
                        priority_updated_ij=self.current_bids_priority_beta.loc[task_id, agent_id],
                        priority_source_ij=received_current_bids_priority_beta.loc[task_id, agent_id],

                        # Reset
                        tasks_value_x_ij=self.task_list_x.loc[task_id, "task_list_x"],
                        reset=True
                    )
                )

                # > Update local states
                self.current_bids_b.loc[task_id, agent_id] = current_bids_b_ij_updated
                self.current_bids_priority_beta.loc[task_id, agent_id] = current_bids_priority_beta_ij_updated
                self.task_list_x.loc[task_id, "task_list_x"] = tasks_value_x_ij_updated

                # -> Priority merge received current allocations a into local current allocations a
                # > Determine correct matrix values
                tasks_value_x_ij_updated, current_allocations_a_ij_updated, current_allocations_priority_alpha_ij_updated = (
                    self.priority_merge(
                        # Logging
                        task_id=task_id,
                        agent_id=agent_id,

                        # Merging
                        matrix_updated_ij=self.current_allocations_a.loc[task_id, agent_id],
                        matrix_source_ij=received_current_allocations_a.loc[task_id, agent_id],
                        priority_updated_ij=self.current_allocations_priority_alpha.loc[task_id, agent_id],
                        priority_source_ij=received_current_allocations_priority_alpha.loc[task_id, agent_id],

                        # Reset
                        tasks_value_x_ij=self.task_list_x.loc[task_id, "task_list_x"],
                        reset=True
                    )
                )

                # > Update local states
                self.current_allocations_a.loc[task_id, agent_id] = current_allocations_a_ij_updated
                self.current_allocations_priority_alpha.loc[task_id, agent_id] = current_allocations_priority_alpha_ij_updated
                self.task_list_x.loc[task_id, "task_list_x"] = tasks_value_x_ij_updated

    def __update_task(
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
            self.current_allocations_a.loc[task_id, self.current_allocations_a.loc[task_id] == 1].index.to_list()
        )

        # -> If there are imposed allocations, find the imposed allocation with the highest priority
        if len(agents_with_imposed_allocations) > 0:
            # -> Find agent with the highest priority
            winning_agent = self.current_allocations_priority_alpha.loc[task_id, agents_with_imposed_allocations].idxmax()

        else:
            # -> Compare received winning bids with local winning bids
            winning_agent = self.id if self.winning_bids_y.loc[task_id, "winning_bids_y"] >= received_winning_bids_y.loc[task_id, "winning_bids_y"] else None

            # TODO: Fix self.winning_bids_y.loc[task_id] > received_winning_bids_y.loc[task_id] scenario. For now it is assumed that two agents cannot have the same bid for the same task

        # -> Update task list x
        if winning_agent is not self.id:
            # -> If the winning agent is not the agent, remove the task from the task list
            self.task_list_x.loc[task_id] = 0

            self.get_logger().info(f"{self.id} - Dropping task {task_id} from task list")

            # -> Cancel goal
            self.publish_goal_msg(task_id=task_id, meta_action="unassign")

    # ---------------- Processes
    # >>>> CBAA
    def select_task(self):
        """
        Select a task to bid for based on the current state
        1. Merge local states with shared states and update local states if necessary
        2. If no task is assigned to self, select a task
        """
        # ---- Merge local states with shared states
        # -> If own states have changed, update local states (optimisation to avoid unnecessary updates)
        if self.allocation_state_change:
            # > For each task ...
            for task_id in self.task_log.ids_pending:
                # > For each agent ...
                for agent_id in self.fleet.ids_active:
                    # -> Priority merge local bids c into current bids b
                    # > Determine correct matrix values
                    _, current_bids_b_ij_updated, current_bids_priority_beta_ij_updated = self.priority_merge(
                        # Logging
                        task_id=task_id,
                        agent_id=agent_id,

                        # Merging
                        matrix_updated_ij=self.current_bids_b.loc[task_id, agent_id],
                        matrix_source_ij=self.local_bids_c.loc[task_id, agent_id],
                        priority_updated_ij=self.current_bids_priority_beta.loc[task_id, agent_id],
                        priority_source_ij=self.hierarchy_level,

                        # Reset
                        tasks_value_x_ij=self.task_list_x.loc[task_id, "task_list_x"],
                        reset=False
                    )

                    # > Update local states
                    self.current_bids_b.loc[task_id, agent_id] = current_bids_b_ij_updated
                    self.current_bids_priority_beta.loc[task_id, agent_id] = current_bids_priority_beta_ij_updated

                    # -> Merge local intercessions into allocation intercession
                    if self.hierarchy_level > self.current_allocations_priority_alpha.loc[task_id, agent_id]:
                        allocation_state = self.action_to_allocation_state(
                            action=self.local_allocations_d.loc[task_id, agent_id]
                        )

                        if allocation_state is not None:
                            self.current_allocations_a.loc[task_id, agent_id] = allocation_state

        # ---- Select task
        # -> If no task is assigned to self, select a task
        if self.task_list_x["task_list_x"].sum() == 0:
            # -> Create list of valid tasks (pandas series of task ids initialised as 0)
            valid_tasks_list_h = pd.DataFrame(
                np.zeros(self.Task_count_N_t),
                index=self.task_log.ids_pending,
                columns=["valid_tasks_list_h"]
            )

            # > For each task ...
            for task_id in self.task_log.ids_pending:
                # -> Create set of agents with imposed allocations
                agents_with_imposed_allocations = list(set(self.current_allocations_a.loc[task_id, self.current_allocations_a.loc[task_id] == 1].index))

                # -> If there are imposed allocations, check if self is imposed allocation with the highest priority
                if len(agents_with_imposed_allocations) > 0:
                    # -> Find agent with the highest priority
                    winning_agent = self.current_allocations_priority_alpha.loc[task_id, agents_with_imposed_allocations].idxmax()

                    # TODO: Adjust to handle conflicting imposed allocations. For now assumed that allocation intercession is conflict-free

                    # -> If self is the winning agent, add the task to the task list
                    if winning_agent == self.id:
                        valid_tasks_list_h[task_id, "valid_tasks_list_h"] = 1
                else:
                    # -> If there are no imposed allocations, check if self has the highest bid and is not blacklisted
                    valid_task = int(self.current_allocations_a.loc[task_id, self.id] != -1 and self.current_bids_b.loc[task_id, self.id] > self.winning_bids_y.loc[task_id, "winning_bids_y"])

                    valid_tasks_list_h.loc[task_id, "valid_tasks_list_h"] = valid_task

            # -> If there are valid tasks, select the task with the highest bid
            if valid_tasks_list_h["valid_tasks_list_h"].sum() > 0:
                # -> Select task with the highest bid
                # > Get index of all the tasks for which the valid_tasks_list_h is 1
                valid_tasks = valid_tasks_list_h[valid_tasks_list_h["valid_tasks_list_h"] == 1].index.to_list()

                # > Get valid task with largest bid
                selected_task = self.current_bids_b.loc[valid_tasks, self.id].idxmax()

                # -> Add task to the task list
                self.task_list_x.loc[selected_task, "task_list_x"] = 1

                # -> Update winning bids
                self.winning_bids_y.loc[selected_task, "winning_bids_y"] = self.current_bids_b.loc[selected_task, self.id]

                self.get_logger().info(f"{self.id} + Assigning task {selected_task} to self (bid: {round(self.current_bids_b.loc[valid_tasks, self.id].max(), 3)} - Pending task count: {len(self.task_log.ids_pending)-1})")

                # -> Assign goal
                self.publish_goal_msg(task_id=selected_task, meta_action="assign")

    # ---------------- tools
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
            pprint(self.task_log.to_list())
            pprint(self.fleet.to_list())

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
            print(self.current_bids_b)

            # print("\nCurrent bids priority beta:")
            # print(self.current_bids_priority_beta)
            #
            # print("\n------------")
            # print("Current allocations a:")
            # print(self.current_allocations_a)
            #
            # print("\nCurrent allocations priority alpha:")
            # print(self.current_allocations_priority_alpha)

        print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")

    # ---------------- tools
    # >>>> CBAA
    def priority_merge(
            self,
            task_id: str,
            agent_id: str,
            matrix_updated_ij: float,
            matrix_source_ij: float,
            priority_updated_ij: float,
            priority_source_ij: float,
            tasks_value_x_ij: int or None = None,
            reset: bool = False
            ):
        """
        Merge two matrices values based on priority values. If source priority is higher, update the updated matrix value with the
        source matrix. If updated priority is higher, do nothing. If priorities are equal, apply other tie-breakers.

        Option to reset task value to zero if source priority is higher. If using reset, the tasks_value_x_ij must be
        provided.

        ### For logging purposes
        :param task_id: Task id
        :param agent_id: Agent id

        ### Merging variables
        :param matrix_updated_ij: Updated matrix value
        :param matrix_source_ij: Source matrix value to compare with updated matrix value
        :param priority_updated_ij: Updated priority value
        :param priority_source_ij: Source priority value used to compare source matrix value with updated priority value

        ### Reset variables
        :param tasks_value_x_ij: Task value to reset to zero if source priority is higher
        :param reset: Flag to reset task value to zero if source priority is higher

        :return: Updated task value, updated matrix value, updated priority value
        """

        # -> If source priority is higher
        if priority_source_ij > priority_updated_ij:

            # -> Update matrix value with source matrix value
            matrix_updated_ij = matrix_source_ij

            # -> Update priority value with source priority value
            priority_updated_ij = priority_source_ij

            # -> Reset task value to zero to remove allocation
            if reset:
                if tasks_value_x_ij == 1:
                    self.get_logger().info(f"{self.id} - Dropping task {task_id} from task list - Pending task count: {len(self.task_log.ids_pending)-1}")

                    tasks_value_x_ij = 0

                    # -> Cancel goal
                    self.publish_goal_msg(task_id=task_id, meta_action="unassign")

        # -> If updated priority is higher
        elif priority_source_ij < priority_updated_ij:
            # -> Do nothing as updated priority is higher, therefore keep updated matrix value and priority value
            pass

        # -> If priorities are equal
        else:
            # Apply other tie-breakers
            # TODO: Implement tie-breakers, for now larger value is kept
            matrix_updated_ij = max(matrix_updated_ij, matrix_source_ij)

        return tasks_value_x_ij, matrix_updated_ij, priority_updated_ij


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_sequence = maaf_allocation_node()

    rclpy.spin(path_sequence)

    path_sequence.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
