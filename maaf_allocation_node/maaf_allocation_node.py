##################################################################################################################

"""
This module contains the MAAF allocation node class, which is a ROS2 node that implements the CBAA algorithm for task allocation.
"""

# Built-in/Generic Imports
from random import randint
from json import dumps, loads
from typing import List, Optional
from copy import deepcopy
from datetime import datetime
from pprint import pprint

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
from .maaf_agent import maaf_agent
from .maaf_dataclass_cores import maaf_list_dataclass
from .maaf_task_dataclasses import Task, Task_log
from .maaf_fleet_dataclasses import Agent, Fleet
from .maaf_state_dataclasses import Agent_state
from .Tools import *
from maaf_msgs.msg import TeamCommStamped

##################################################################################################################


class maaf_allocation_node(maaf_agent):
    def __init__(self):

        # ---- Init parent class
        maaf_agent.__init__(self)

        # -----------------------------------  Agent state
        # ---- Agent state
        self.pose = None

        # ---- Allocation state
        """
        Task list x of size N_t: 
        - 0 if not assigned
        - 1 if assigned
        
        List is a pandas array of size N_t, initialized as zero array, with task ids as index
        """
        self.task_list_x = pd.Series(np.zeros(self.Task_count_N_t), index=self.task_log.ids_pending)

        """
        Local bids matrix c of size N_t x N_u: 
        > value c_ijr is bid agent i makes for task j, for for agent r
                
        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.local_bids_c = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active
        )

        """
        Local allocations matrix d of size N_t x N_u:
        - 0: do nothing
        - 1: reset (remove allocation or blacklisting)
        - 2: blacklist (ban allocation)
        - 3: allocate (impose allocation)
        
        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.local_allocations_d = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active
        )

        # ---- Shared states
        """
        Winning bids list y of size N_t: 
        > most up to date estimation of current highest bid for each task across all agents in fleet
        
        List is a pandas array of size N_t, initialized as zero array, with task ids as index
        """
        self.winning_bids_y = pd.Series(
            np.zeros(self.Task_count_N_t),
            index=self.task_log.ids_pending
        )

        """
        Current bids matrix b of size N_t x N_u: 
        > highest priority/value bids made across the fleet for each task and each agent
        
        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.current_bids_b = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active
        )

        """
        Current bids priority matrix beta of size N_t x N_u:
        > priority value corresponding to each bid in current_bids_bi
        
        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.current_bids_priority_beta = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active
        )

        """
        Current allocations matrix a of size N_t x N_u:
        - -1: blacklisted
        - 0: not allocated (free allocation)
        - 1: allocated (imposed allocation)
        
        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.current_allocations_a = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active
        )

        """
        Current allocations priority matrix alpha of size N_t x N_u:
        > priority value corresponding to each allocation in current_allocations_ai
        
        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.current_allocations_priority_alpha = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active
        )

        # -> Initialise previous state hash
        self.__prev_state_hash_dict = deepcopy(self.__state_hash_dict)

        # -----------------------------------  Confirm initialisation
        self.get_logger().info(f"MAAF agent {self.id}: Allocation node initialised ({ALLOCATION_METHOD})")

    # ============================================================== PROPERTIES
    # ---------------- Self state
    # >>>> Base
    @property
    def state_awareness_dict(self):
        """
        State awareness at current time step (serialised)

        :return: dict
        """
        return {
            "tasks": self.task_log.to_list(),
            "fleet": self.fleet.to_list()
        }

    @property
    def local_allocation_state_dict(self):
        """
        Local allocation state at current time step (serialised)

        :return: dict
        """
        return {
            "task_list_x": self.task_list_x.to_dict(),
            "local_bids_c": self.local_bids_c.to_dict(),
            "local_allocations_d": self.local_allocations_d.to_dict()
        }

    @property
    def shared_allocation_state_dict(self):
        """
        Shared allocation state at current time step (serialised)

        :return: dict
        """
        return {
            "winning_bids_y": self.winning_bids_y.to_dict(),
            "current_bids_b": self.current_bids_b.to_dict(),
            "current_bids_priority_beta": self.current_bids_priority_beta.to_dict(),
            "current_allocations_a": self.current_allocations_a.to_dict(),
            "current_allocations_priority_alpha": self.current_allocations_priority_alpha.to_dict()
        }

    @property
    def state(self) -> dict:
        """
        Full state of the agent at current time step (not serialised)
        Mainly used for debugging and logging

        :return: dict
        """
        return self.state_awareness_dict | self.local_allocation_state_dict | self.shared_allocation_state_dict

    @property
    def allocation_state(self) -> dict:
        """
        Allocation state at current time step (not serialised)
        Used in allocation process

        :return: dict
        """

        return self.state_awareness_dict | self.shared_allocation_state_dict

    @property
    def shared_allocation_state_change(self) -> bool:
        """
        Check if the shared allocation state has changed

        :return: bool
        """
        # -> Compare hash of current state with hash of previous state
        for key, value in self.__state_hash_dict.items():
            # -> If any value has changed, return True
            if value != self.__prev_state_hash_dict[key]:
                return True

        # -> If no value has changed, return False
        return False

    @property
    def __state_hash_dict(self) -> dict:
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
    def pose_subscriber_callback(self, pose_msg) -> None:
        """
        Callback for the pose subscriber

        :param pose_msg: PoseStamped message
        """
        # -> Convert quaternion to euler
        u, v, w = euler_from_quaternion(quat=pose_msg.pose.orientation)

        # -> Update state
        # > Pose
        self.fleet[self.id]["state"]["x"] = pose_msg.pose.position.x
        self.fleet[self.id]["state"]["y"] = pose_msg.pose.position.y
        self.fleet[self.id]["state"]["z"] = pose_msg.pose.position.z
        self.fleet[self.id]["state"]["u"] = u
        self.fleet[self.id]["state"]["v"] = v
        self.fleet[self.id]["state"]["w"] = w

        # > Timestamp
        self.fleet[self.id]["state"]["timestamp"] = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9

    def task_msg_subscriber_callback(self, task_msg) -> None:
        """
        Callback for task messages, create new task. add to local tasks and update local states, and select new task

        :param task_msg: TaskMsgStamped message
        """

        # -> Check if the message is for the agent
        msg_target = task_msg.target

        # -> If the message is not for the agent...
        if msg_target != self.id and msg_target != "all":
            pass
            ## -> Check if the agent should rebroadcast the message
            # msg, rebroadcast = self.check_rebroadcast(msg=task_msg, publisher=self.fleet_msgs_pub)

        # -> Create new task
        if task_msg.meta_action == "add":
            # -> Unpack msg
            task_dict = loads(task_msg.memo)  # Task from task factory

            # TODO: Remove this line once task creation handles stamp creation
            task_dict["creation_timestamp"] = self.current_timestamp

            # -> Create task object
            task = Task.from_dict(task_dict)

            # -> Update state awareness
            self.__update_situation_awareness(task_list=[task], fleet=None)

        # -> Select task
        self.select_task()

        # -> Update previous state hash
        self.__prev_state_hash_dict = deepcopy(self.__state_hash_dict)

        # -> Publish allocation state to the fleet to share the new task
        self.publish_allocation_state_msg()

        # # -> If state has changed, update local states (only publish when necessary)
        # if self.shared_allocation_state_change:
        #     # -> Update previous state hash
        #     self.__prev_state_hash_dict = deepcopy(self.__state_hash_dict)
        #
        #     # -> Publish allocation state to the fleet
        #     self.publish_allocation_state_msg()
        #
        # else:
        #     # -> Check if the agent should rebroadcast the message
        #     msg, rebroadcast = self.check_rebroadcast(msg=task_msg, publisher=self.fleet_msgs_pub)

    def fleet_msg_update_timer_callback(self) -> None:
        """
        Callback for the fleet message update timer
        """
        # -> Publish allocation state to the fleet
        self.publish_allocation_state_msg()

        self.get_logger().info(f">>>>>>>>>>>>>>>>>>>> Allocation state published")

    def team_msg_subscriber_callback(self, team_msg) -> None:
        """
        Callback for team messages, consensus phase of the CBAA algorithm

        :param team_msg: TeamComm message
        """
        # ---- Unpack msg
        # -> Check if the message is for the agent
        msg_target = team_msg.target

        # -> If the message is not for the agent...
        if msg_target == self.id and msg_target != "all":
            # -> Check if the agent should rebroadcast the message
            msg, rebroadcast = self.check_rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub)

        self.get_logger().info(f"<<<<<<<<<<<<<<<<<<<< Allocation state received from {team_msg.source}")

        # -> Deserialise allocation state
        allocation_state = self.deserialise(state=team_msg.memo)

        # -> Update local situation awareness
        # > Convert task list to Task objects
        task_list = [Task.from_dict(task) for task in allocation_state["tasks"]]

        # > Convert fleet to Agent objects
        fleet = [Agent.from_dict(agent) for agent in allocation_state["fleet"]]

        self.__update_situation_awareness(
            task_list=task_list,
            fleet=fleet
        )

        # -> Update shared states
        self.__update_shared_states(
            received_current_bids_b=allocation_state["current_bids_b"],
            received_current_bids_priority_beta=allocation_state["current_bids_priority_beta"],
            received_current_allocations_a=allocation_state["current_allocations_a"],
            received_current_allocations_priority_alpha=allocation_state["current_allocations_priority_alpha"]
        )

        # -> Update the task in the task list x the agent is assigned to
        for task_id in self.task_log.ids_pending:
            if self.task_list_x[task_id] == 1:
                self.__update_task(
                    received_winning_bids_y=allocation_state["winning_bids_y"],
                    task_id=task_id
                )

        # -> Select new task
        self.select_task()

        # -> If state has changed, update local states (only publish when necessary)
        if self.shared_allocation_state_change:
            # -> Update previous state hash
            self.__prev_state_hash_dict = deepcopy(self.__state_hash_dict)

            # -> Publish allocation state to the fleet
            self.publish_allocation_state_msg()

        else:
            # -> Check if the agent should rebroadcast the message
            msg, rebroadcast = self.check_rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub)

    # >>>> CBAA
    def __update_situation_awareness(self, task_list: Optional[List[Task]], fleet: Optional[List[Agent]]) -> None:
        """
        Update local states with new tasks and fleet dicts. Add new tasks and agents to local states and extend
        local states with new rows and columns for new tasks and agents

        :param task_list: Tasks dict
        :param fleet: Fleet dict
        """
        
        self.get_logger().info(f"> Updating situation awareness")
        
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

        def remove_agent(agent):
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
            self.task_list_x[task_id] = 0
            self.local_bids_c.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)
            self.local_allocations_d.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)

            # > Shared
            self.winning_bids_y[task_id] = 0

            self.current_bids_b.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)
            self.current_bids_priority_beta.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)

            self.current_allocations_a.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)
            self.current_allocations_priority_alpha.loc[task_id] = pd.Series(np.zeros(self.Agent_count_N_u),
                                                                             index=self.fleet.ids_active)

            # -> Estimate bid(s) for new task
            self.bid(task=self.task_log[task_id])

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
            task_id = task.id
            
            # > Local
            self.task_list_x.drop(index=task_id, inplace=True)
            self.local_bids_c.drop(index=task_id, inplace=True)
            self.local_allocations_d.drop(index=task_id, inplace=True)

            # > Shared
            self.winning_bids_y.drop(index=task_id, inplace=True)

            self.current_bids_b.drop(index=task_id, inplace=True)
            self.current_bids_priority_beta.drop(index=task_id, inplace=True)

            self.current_allocations_a.drop(index=task_id, inplace=True)
            self.current_allocations_priority_alpha.drop(index=task_id, inplace=True)

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
                        self.fleet.set_agent_state(agent=agent, state=agent["state"])

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
        
        self.get_logger().info(f"> Updating shared states")
        
        # -> For each task ...
        for task_id in self.task_log.ids_pending:
            # -> for each agent ...
            for agent_id in self.fleet.ids_active:
                # -> Priority merge with reset received current bids b into local current bids b
                # > Determine correct matrix values
                tasks_value_x_ij_updated, current_bids_b_ij_updated, current_bids_priority_beta_ij_updated = self.priority_merge(
                    matrix_updated_ij=self.current_bids_b.loc[task_id, agent_id],
                    matrix_source_ij=received_current_bids_b[task_id][agent_id],
                    priority_updated_ij=self.current_bids_priority_beta.loc[task_id, agent_id],
                    priority_source_ij=received_current_bids_priority_beta[task_id][agent_id],
                    tasks_value_x_ij=self.task_list_x[task_id],
                    reset=True
                )

                # > Update local states
                self.current_bids_b.loc[task_id, agent_id] = current_bids_b_ij_updated
                self.current_bids_priority_beta.loc[task_id, agent_id] = current_bids_priority_beta_ij_updated
                self.task_list_x[task_id] = tasks_value_x_ij_updated

                # -> Priority merge received current allocations a into local current allocations a
                # > Determine correct matrix values
                _, current_allocations_a_ij_updated, current_allocations_priority_alpha_ij_updated = self.priority_merge(
                    matrix_updated_ij=self.current_allocations_a.loc[task_id, agent_id],
                    matrix_source_ij=received_current_allocations_a[task_id][agent_id],
                    priority_updated_ij=self.current_allocations_priority_alpha.loc[task_id, agent_id],
                    priority_source_ij=received_current_allocations_priority_alpha[task_id][agent_id],
                )

                # > Update local states
                self.current_allocations_a.loc[task_id, agent_id] = current_allocations_a_ij_updated
                self.current_allocations_priority_alpha.loc[
                    task_id, agent_id] = current_allocations_priority_alpha_ij_updated

    def __update_task(
            self,
            received_winning_bids_y,
            task_id
    ):
        """
        Update current task based on received winning bids and updated allocation intercession from the fleet
        
        :param received_winning_bids_y: Winning bids list y received from the fleet
        :param task_id: task id
        """
        
        self.get_logger().info(f"> Updating task {task_id}")
        
        # -> Create set of agents with imposed allocations
        agents_with_imposed_allocations = set(
            self.current_allocations_a.loc[task_id][self.current_allocations_a.loc[task_id] == 1].index)

        # -> If there are imposed allocations, find the imposed allocation with the highest priority
        if len(agents_with_imposed_allocations) > 0:
            # -> Find agent with the highest priority
            winning_agent = self.current_allocations_priority_alpha.loc[task_id][
                agents_with_imposed_allocations].idxmax()

        else:
            # -> Compare received winning bids with local winning bids
            winning_agent = self.id if self.winning_bids_y.loc[task_id] >= received_winning_bids_y.loc[
                task_id] else None

            # TODO: Fix self.winning_bids_y.loc[task_id] > received_winning_bids_y.loc[task_id] scenario. For now it is assumed that two agents cannot have the same bid for the same task

        # -> Update task list x
        if winning_agent is not self.id:
            # -> If the winning agent is not the agent, remove the task from the task list
            self.task_list_x[task_id] = 0

    # ---------------- Processes
    # >>>> Base
    def publish_allocation_state_msg(self):
        """
        Publish allocation state to the fleet as TeamCommStamped.msg message
        """
        # -> Create message
        msg = TeamCommStamped()

        # > Metadata
        msg.stamp = timestamp_to_ros_time(timestamp=self.current_timestamp).to_msg()
        msg.trace = [self.id]

        # > Tracker
        msg.source = self.id
        msg.target = "all"

        msg.meta_action = "allocation update"
        msg.memo = dumps(self.allocation_state)

        # -> Publish message
        self.fleet_msgs_pub.publish(msg)

    def publish_goal(self, task_id):
        """
        Publish goal to the robot's goal topic for execution as TeamCommStamped message

        :param task_id: task id
        """

        # -> Create message
        msg = TeamCommStamped()

        # > Metadata
        msg.stamp = timestamp_to_ros_time(timestamp=self.current_timestamp).to_msg()
        msg.trace = [self.id]

        # > Tracker
        msg.source = self.id
        msg.target = self.id

        msg.meta_action = "goal assignment"
        msg.memo = dumps(self.task_log[task_id].to_dict())

        # -> Publish message
        self.goal_sequence_publisher.publish(msg)

    # >>>> CBAA
    def select_task(self):
        """
        Select a task to bid for based on the current state
        1. Merge local states with shared states and update local states if necessary
        2. If no task is assigned to self, select a task
        """
        # ---- Merge local states with shared states
        # -> If own states have changed, update local states (optimisation to avoid unnecessary updates)
        if self.shared_allocation_state_change:
            # > For each task ...
            for task_id in self.task_log.ids_pending:
                # > For each agent ...
                for agent_id in self.fleet.ids_active:
                    # -> Priority merge local bids c into current bids b
                    # > Determine correct matrix values
                    _, current_bids_b_ij_updated, current_bids_priority_beta_ij_updated = self.priority_merge(
                        matrix_updated_ij=self.current_bids_b.loc[task_id, agent_id],
                        matrix_source_ij=self.local_bids_c.loc[task_id, agent_id],
                        priority_updated_ij=self.current_bids_priority_beta.loc[task_id, agent_id],
                        priority_source_ij=self.hierarchy_level,
                    )

                    # > Update local states
                    self.current_bids_b.loc[task_id, agent_id] = current_bids_b_ij_updated
                    self.current_bids_priority_beta.loc[task_id, agent_id] = current_bids_priority_beta_ij_updated

                    # -> Merge local intercessions into allocation intercession
                    if self.hierarchy_level > self.current_allocations_priority_alpha.loc[task_id, agent_id]:
                        if self.local_allocations_d.loc[task_id, agent_id] == 1:
                            self.current_allocations_a.loc[task_id, agent_id] = 0  # Set to free allocation
                        elif self.local_allocations_d.loc[task_id, agent_id] == 2:
                            self.current_allocations_a.loc[task_id, agent_id] = -1  # Set to blacklisted
                        elif self.local_allocations_d.loc[task_id, agent_id] == 3:
                            self.current_allocations_a.loc[task_id, agent_id] = 1  # Set to imposed allocation
                        else:
                            pass  # Do nothing

        # ---- Select task
        # -> If no task is assigned to self, select a task
        if self.task_list_x.sum() == 0:
            # -> Create list of valid tasks (pandas series of task ids initialised as 0)
            valid_tasks_list_h = pd.Series(np.zeros(self.Task_count_N_t), index=self.task_log.ids_pending)

            # > For each task ...
            for task_id in self.task_log.ids_pending:
                # -> Create set of agents with imposed allocations
                agents_with_imposed_allocations = set(
                    self.current_allocations_a.loc[task_id][self.current_allocations_a.loc[task_id] == 1].index)

                # -> If there are imposed allocations, check if self is imposed allocation with the highest priority
                if len(agents_with_imposed_allocations) > 0:
                    # -> Find agent with the highest priority
                    winning_agent = self.current_allocations_priority_alpha.loc[task_id][
                        agents_with_imposed_allocations].idxmax()

                    # TODO: Adjust to handle conflicting imposed allocations. For now assumed that allocation intercession is conflict-free

                    # -> If self is the winning agent, add the task to the task list
                    if winning_agent == self.id:
                        valid_tasks_list_h[task_id] = 1

                else:
                    # -> If there are no imposed allocations, check if self has the highest bid and is not blacklisted
                    valid_tasks_list_h[task_id] = self.current_allocations_a.loc[task_id][self.id] != -1 and \
                                                  self.current_bids_b.loc[task_id][self.id] > self.winning_bids_y.loc[
                                                      task_id]

            # -> If there are valid tasks, select the task with the highest bid
            if valid_tasks_list_h.sum() > 0:
                # -> Select task with the highest bid
                selected_task = self.current_bids_b.loc[valid_tasks_list_h == 1][self.id].idxmax()

                # -> Add task to the task list
                self.task_list_x[selected_task] = 1

                # -> Update winning bids
                self.winning_bids_y[selected_task] = self.current_bids_b.loc[selected_task][self.id]

                # -> Publish goal
                self.publish_goal(task_id=selected_task)

    # ---------------- Tools
    # >>>> CBAA
    @staticmethod
    def priority_merge(matrix_updated_ij: float,
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
                tasks_value_x_ij = 0

        # -> If updated priority is higher
        elif priority_source_ij < priority_updated_ij:
            # -> Do nothing as updated priority is higher, therefore keep updated matrix value and priority value
            pass

        # -> If priorities are equal
        else:
            # Apply other tie-breakers
            # TODO: Implement tie-breakers
            pass

        return tasks_value_x_ij, matrix_updated_ij, priority_updated_ij

    def deserialise(self, state: str) -> dict:
        """
        Deserialise the state

        :param state: State to deserialise
        :return: dict
        """

        deserialised_state = loads(state)
        
        pandas_dicts = self.local_allocation_state_dict.keys() | self.shared_allocation_state_dict.keys()
        
        for key, value in deserialised_state.items():
            # -> If the value is in the local or shared allocation state ...
            if key in pandas_dicts:
                # -> Convert to pandas dataframe or series
                deserialised_state[key] = pd.DataFrame(value) if isinstance(value, dict) else pd.Series(value)

        return deserialised_state


def random_bid_evaluation(*args, **kwargs):
    return randint(0, 10000)


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_sequence = maaf_allocation_node()

    rclpy.spin(path_sequence)

    path_sequence.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
