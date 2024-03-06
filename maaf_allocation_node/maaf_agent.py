
##################################################################################################################

"""
Parent class for the CAF framework. To use, the following must be defined in the child class:
MAF:
    Optional:
    - on_set_state

CAF:
    - message_to_publish(self) (property)
    - process_msg(self, msg)
    - next_state(self) (property)
    # - action_to_take(self) (property)
    # - process_step(self, obs, reward, done, infos)

    Optional:
    - on_new_task(self, task_id, task_data)
"""

# Built-in/Generic Imports
import os
from abc import abstractmethod
from typing import List, Optional
from datetime import datetime, timedelta
from random import randint
from json import dumps, loads
from pprint import pprint, pformat
import warnings
from copy import deepcopy

# Libs
import numpy as np
import pandas as pd

# Suppress FutureWarning messages
warnings.simplefilter(action='ignore', category=FutureWarning)

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, Point

# TODO: Cleanup
# NetworkX
import networkx as nx

# Matplotlib
import matplotlib.pyplot as plt


# Local Imports
from orchestra_config.orchestra_config import *     # KEEP THIS LINE, DO NOT REMOVE
from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
from .node_config import *
from .task_dataclasses import Task, Task_log
from .fleet_dataclasses import Agent, Fleet
from .state_dataclasses import Agent_state
from .tools import *

from .Bidding_logics.random_bid import random_bid
from .Bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid
from .Bidding_logics.anticipated_action_task_interceding_agent import anticipated_action_task_interceding_agent

##################################################################################################################


class MAAFAgent(Node):
    def __init__(
            self,
            node_name: str,
            id: str = None,
            name: str = None,
            skillset: List[str] = None,
            bid_estimator = None
        ):
        """
        maaf agent class
        """

        # ----------------------------------- Node Configuration
        Node.__init__(
            self,
            node_name=node_name,
        )

        # ---- Agent properties
        # > declare all parameters at once
        self.declare_parameters(namespace="", parameters=[("id", ""), ("name", "")])

        if id is None:
            self.id = self.get_parameter("id").get_parameter_value().string_value
        else:
            self.id = id

        if name is None:
            self.name = self.get_parameter("name").get_parameter_value().string_value
        else:
            self.name = name

        # TODO: Implement local loading of configs
        self.agent_class = "Base"
        self.hierarchy_level = 0
        self.affiliations = []
        self.specs = {}

        if skillset is None:
            # self.skillset = self.get_parameter("skillset").get_parameter_value().string_array_value
            # TODO: Cleanup
            self.skillset = skillsets[self.id]
        else:
            self.skillset = skillset

        # ---- Bid evaluation function
        if bid_estimator is None:
            self.bid_evaluation_function = None
        elif bid_estimator is not None:
            self.bid_evaluation_function = bid_estimator

        # TODO: Cleanup
        if self.id in bid_function.keys():
            if bid_function[self.id] == "graph_weighted_manhattan_distance_bid":
                self.bid_evaluation_function = graph_weighted_manhattan_distance_bid
            elif bid_function[self.id] == "anticipated_action_task_interceding_agent":
                self.bid_evaluation_function = anticipated_action_task_interceding_agent

        # ---- Multi-hop behavior
        self.rebroadcast_received_msgs = False

        # ---- Environment
        self.env = None

        # ---- Fleet and task log
        self.__setup_fleet_and_task_log()

        # ---- Listeners
        self.__env_update_listeners = []
        self.__pose_update_listeners = []

        # -> Connect listeners
        # self.fleet.add_on_edit_list_listener()
        # self.task_log.add_on_edit_list_listener()

        # ---- Node connections
        self.__setup_node_pubs_subs()

        # ---- Allocation states
        self.__setup_allocation_base_states()

        # -> Initialise previous state hash
        self.prev_allocation_state_hash_dict = None
        # TODO: ALWAYS SET IN PARENT CLASS ONCE AGAIN

        self.get_logger().info(f"\n> Agent {self.id} initialised: " +
                               f"\n     Skillset: {self.skillset}" +
                               f"\n     Bid evaluation: {self.bid_evaluation_function}"
                               )

    # ============================================================== INIT
    def __setup_fleet_and_task_log(self) -> None:
        # ---- Fleet data
        """
        Fleet dict of size N_a
        Agents ids are obtained using the self.id_card property
        """
        # -> Create fleet object
        self.fleet = Fleet()

        # -> Fill with initial data
        # > Retrieve initial fleet data from parameters
        fleet_data = []

        # > Add initial fleet data to the fleet object
        self.fleet.from_list(item_dicts_list=fleet_data)

        # -> Check if self is in the fleet, if not, add self to the fleet
        if self.id not in self.fleet.ids:
            self.fleet.add_item(
                Agent(
                    id=self.id,
                    name=self.name,
                    agent_class=self.agent_class,
                    hierarchy_level=self.hierarchy_level,
                    affiliations=self.affiliations,
                    specs=self.specs,
                    skillset=self.skillset,
                    # TODO: Implement state init
                    state=Agent_state(
                        agent_id=self.id,
                        timestamp=self.current_timestamp,
                        battery_level=100,
                        stuck=False,
                        x=0,
                        y=0,
                        z=0,
                        u=0,
                        v=0,
                        w=0
                    )
                )
            )

        # ---- Task log
        """
        Tasks dict of size N_t
        Tasks are created using the self.create_task method
        """
        # -> Create task log object
        self.task_log = Task_log()

        # -> Fill with initial data
        # > Retrieve initial task data from parameters
        task_log_data = []

        # > Add initial task data to the task log object
        self.task_log.from_list(item_dicts_list=task_log_data)

    def __setup_node_pubs_subs(self) -> None:
        """
        Setup node publishers and subscribers
        Separated from __init__ for readability
        :return: None
        """

        # ----------------------------------- Subscribers
        if RUN_MODE == OPERATIONAL:
            # ---------- fleet_msgs
            self.fleet_msgs_sub = self.create_subscription(
                msg_type=TeamCommStamped,
                topic=topic_fleet_msgs,
                callback=self.team_msg_subscriber_callback,
                qos_profile=qos_fleet_msgs
            )

        elif RUN_MODE == SIM:
            # ---------- fleet_msgs_filtered
            self.fleet_msgs_sub = self.create_subscription(
                msg_type=TeamCommStamped,
                topic=topic_fleet_msgs_filtered,
                callback=self.team_msg_subscriber_callback,
                qos_profile=qos_fleet_msgs
            )

        # ---------- environment
        # TODO: Cleanup
        self.env_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_environment,
            callback=self.env_callback,
            qos_profile=qos_env
        )

        # ---------- task
        self.task_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_tasks,
            callback=self.task_msg_subscriber_callback,
            qos_profile=qos_tasks
        )

        # ---------- pose
        self.pose_sub = self.create_subscription(
            msg_type=PoseStamped,
            topic=f"/{self.id}{topic_pose}",
            callback=self.pose_subscriber_callback,
            qos_profile=qos_pose
        )

        # ---------- bids
        self.bid_sub = self.create_subscription(
            msg_type=Bid,
            topic=topic_bids,
            callback=self.bid_subscriber_callback,
            qos_profile=qos_intercession
        )

        # ---------- allocation
        self.allocation_sub = self.create_subscription(
            msg_type=Allocation,
            topic=topic_allocations,
            callback=self.allocation_subscriber_callback,
            qos_profile=qos_intercession
        )

        # ----------------------------------- Publishers
        # ---------- fleet_msgs
        self.fleet_msgs_pub = self.create_publisher(
            msg_type=TeamCommStamped,
            topic=topic_fleet_msgs,
            qos_profile=qos_fleet_msgs
        )

        # ---------- goals
        # Goals publisher
        self.goals_pub = self.create_publisher(
            msg_type=TeamCommStamped,
            topic=topic_goals,
            qos_profile=qos_goal
        )

        # ----------------------------------- Timers
        # # ---- Fleet msg update timer
        # self.fleet_msg_update_timer = self.create_timer(
        #     timer_period_sec=FLEET_MSG_UPDATE_TIMER,
        #     callback=self.fleet_msg_update_timer_callback
        # )

    def __setup_allocation_base_states(self) -> None:
        """
        Setup allocation states for the agents
        """
        # ---- Local states
        """
        Local bids matrix c of size N_t x N_u: 
        > value c_ijr is bid agent i makes for task j, for for agent r

        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.local_bids_c = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active,
        )

        self.local_bids_c = self.local_bids_c.astype(float)

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
            columns=self.fleet.ids_active,
        )

        # ---- Shared states
        """
        Winning bids list y of size N_t: 
        > most up to date estimation of current highest bid for each task across all agents in fleet

        List is a pandas dataframe of size N_t, initialized as zero dataframe, with task ids as index
        """

        self.winning_bids_y = pd.DataFrame(
            np.zeros((self.Task_count_N_t, 1)),
            index=self.task_log.ids_pending,
            columns=["winning_bids_y"]
        )

        self.winning_bids_y = self.winning_bids_y.astype(float)

        """
        Shared bids matrix b of size N_t x N_u: 
        > highest priority/value bids made across the fleet for each task and each agent

        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.shared_bids_b = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active,
        )

        self.shared_bids_b = self.shared_bids_b.astype(float)

        """
        Shared bids priority matrix beta of size N_t x N_u:
        > priority value corresponding to each bid in shared_bids_bi

        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.shared_bids_priority_beta = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active
        )

        """
        Shared allocations matrix a of size N_t x N_u:
        - -1: blacklisted
        - 0: not allocated (free allocation)
        - 1: allocated (imposed allocation)

        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.shared_allocations_a = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active
        )

        """
        Shared allocations priority matrix alpha of size N_t x N_u:
        > priority value corresponding to each allocation in shared_allocations_ai

        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.shared_allocations_priority_alpha = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active
        )

    @abstractmethod
    def __setup_allocation_additional_states(self) -> None:
        """
        Setup additional method-dependent allocation states for the agents
        """
        pass

    def reset_allocation_states(self) -> None:
        """
        Reset allocation states for the agents
        """
        # -> Reset fleet and task log
        self.__setup_fleet_and_task_log()

        # -> Reset allocation states
        self.__setup_allocation_base_states()
        self.__setup_allocation_additional_states()

    # ============================================================== Listeners
    # ---------------- Env update
    def add_on_env_update_listener(self, listener) -> None:
        """
        Add listener for environment updates

        :param listener: callable
        """
        self.__env_update_listeners.append(listener)

    def call_on_env_update_listeners(self) -> None:
        """
        Call all environment update listeners
        """
        for listener in self.__env_update_listeners:
            listener()

    # ---------------- Pose update
    def add_on_pose_update_listener(self, listener) -> None:
        """
        Add listener for pose updates

        :param listener: callable
        """
        self.__pose_update_listeners.append(listener)

    def call_on_pose_update_listeners(self) -> None:
        """
        Call all pose update listeners
        """
        for listener in self.__pose_update_listeners:
            listener()

    # ============================================================== PROPERTIES
    # ---------------- Generic
    @property
    def current_timestamp(self) -> float:
        """
        Get current timestamp as float value in seconds

        :return: timestamp float in seconds
        """
        # -> Get current ROS time as timestamp
        time_obj = self.get_clock().now().to_msg()

        return timestamp_from_ros_time(time_obj)

    @property
    def agent(self):
        return self.fleet[self.id]

    # ---------------- Situation state
    # >>>> Shapes
    @property
    def Task_count_N_t(self) -> int:
        """
        Pending tasks count
        """
        return len(self.task_log.tasks_pending)

    @property
    def Agent_count_N_u(self) -> int:
        """
        Active agents count
        """
        return len(self.fleet.ids_active)

    # ============================================================== PROPERTIES
    # ---------------- Self state
    # >>>> State change tracking
    @property
    def allocation_state_hash_dict(self) -> dict:
        """
        Hash of the agent allocation state

        :return: dict
        """
        # -> Convert to series and dataframe to immutable hashable objects
        immutable_state = {}

        state = self.get_state(
            state_awareness=False,
            local_allocation_state=True,
            shared_allocation_state=True,
            serialised=True
            )

        for key, value in state.items():
            if isinstance(value, pd.Series):
                immutable_state[key] = hash(str(value.to_string()))
            elif isinstance(value, pd.DataFrame):
                immutable_state[key] = hash(str(value.to_string()))
            else:
                immutable_state[key] = hash(str(value))

        return immutable_state

    @property
    def allocation_state_change(self) -> bool:
        """
        Check if the shared allocation state has changed

        :return: bool
        """
        # -> Compare hash of current state with hash of previous state
        for key, value in self.allocation_state_hash_dict.items():
            # -> If any value has changed, return True
            if value != self.prev_allocation_state_hash_dict[key]:
                return True

        # -> If no value has changed, return False
        return False

    def check_publish_state_change(self):
        # -> If state has changed, update local states (only publish when necessary)
        if self.allocation_state_change:
            # -> Update previous state hash
            self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

            # -> Publish allocation state to the fleet
            self.publish_allocation_state_msg()

    # >>>> Base states grouped getter
    def get_state(self,
                  state_awareness: bool = False,
                  local_allocation_state: bool = False,
                  shared_allocation_state: bool = False,
                  serialised: bool = False
                  ):
        """
        Main getter for node states. Returns a dict of all requested states, serialised or not

        :param state_awareness: bool, whether to include state awareness
        :param local_allocation_state: bool, whether to include local allocation state
        :param shared_allocation_state: bool, whether to include shared allocation state
        :param serialised: bool, whether to return serialised state or not

        :return: dict
        """

        state = {}

        # !!!!! All states in state awareness must be
        if state_awareness:
            if not serialised:
                state = {**state, **self.state_awareness}
            else:
                serialised_state = {}

                for key, value in self.state_awareness.items():
                    serialised_state[key] = value.to_list()

                state = {**state, **serialised_state}

        if local_allocation_state:
            if not serialised:
                state = {**state, **self.local_allocation_state}
            else:
                serialised_state = {}

                for key, value in self.local_allocation_state.items():
                    serialised_state[key] = value.to_dict()

                state = {**state, **serialised_state}

        if shared_allocation_state:
            if not serialised:
                state = {**state, **self.shared_allocation_state}
            else:
                serialised_state = {}

                for key, value in self.shared_allocation_state.items():
                    serialised_state[key] = value.to_dict()

                state = {**state, **serialised_state}

        return state

    # >>>> Base states grouped
    @property
    def state_awareness(self) -> dict:
        """
        State awareness at current time step (not serialised)
        !!!! All entries must be maaf_list_dataclasses !!!!

        :return: dict
        """
        return {
            "tasks": self.task_log,
            "fleet": self.fleet
        }
    
    @property
    def shared_allocation_state(self):
        """
        Shared allocation state at current time step (not serialised)
        !!!! All entries must be dataframes !!!!

        :return: dict
        """
        return {
            "winning_bids_y": self.winning_bids_y,
            "shared_bids_b": self.shared_bids_b,
            "shared_bids_priority_beta": self.shared_bids_priority_beta,
            "shared_allocations_a": self.shared_allocations_a,
            "shared_allocations_priority_alpha": self.shared_allocations_priority_alpha
        }

    @property
    @abstractmethod
    def local_allocation_state(self) -> dict:
        """
        Local allocation state at current time step not (serialised)
        !!!! All entries must be dataframes !!!!

        :return: dict
        """
        pass

    # ============================================================== METHODS
    # ---------------- Callbacks
    # >>>> Base
    def env_callback(self, msg: TeamCommStamped) -> None:   # TODO: Cleanup
        if self.env is None:    # TODO: Review env management logic
            self.get_logger().info(f"         < Received environment update")
            data = loads(msg.memo)
            self.env = {
                "type": data["env_type"],
                "graph": nx.node_link_graph(data["graph"]),
                "pos": {eval(k): v for k, v in data["pos"].items()}
            }

            # -> Display the graph
            # nx.draw(self.env["graph"], pos=self.env["pos"])
            # plt.show()

            # -> Recompute local bids for all tasks
            for task in self.task_log.tasks_pending:
                task_bids = self.bid(task, [self.agent])

                # -> Store bids to local bids matrix
                for bid in task_bids:
                    # > Bid
                    self.local_bids_c.loc[task.id, bid["agent_id"]] = bid["bid"]

                    # > Allocation
                    self.local_allocations_d.loc[task.id, bid["agent_id"]] = bid["allocation"]

        # -> Call env update listeners
        self.call_on_env_update_listeners()

    def pose_subscriber_callback(self, pose_msg) -> None:
        """
        Callback for the pose subscriber

        :param pose_msg: PoseStamped message
        """
        # -> Convert quaternion to euler
        u, v, w = euler_from_quaternion(quat=pose_msg.pose.orientation)

        # -> Update state
        # > Pose
        self.agent.state.x = pose_msg.pose.position.x
        self.agent.state.y = pose_msg.pose.position.y
        self.agent.state.z = pose_msg.pose.position.z
        self.agent.state.u = u
        self.agent.state.v = v
        self.agent.state.w = w

        # > Timestamp
        self.agent.state.timestamp = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9

        # -> Call pose update listeners
        self.call_on_pose_update_listeners()
        
        self.publish_allocation_state_msg()     # TODO: Cleanup

        # self.get_logger().info(f"         < Received state update ({self.agent.state.x},{self.agent.state.y},{self.agent.state.z})")

    def fleet_msg_update_timer_callback(self) -> None:
        """
        Callback for the fleet message update timer
        """
        # -> Publish allocation state to the fleet
        self.publish_allocation_state_msg()

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
        self.shared_bids_b, self.shared_bids_priority_beta = self.priority_merge(
                                task_id=bid_msg.task_id,
                                agent_id=bid_msg.target_agent_id,

                                # -> Updated matrix
                                # > Updated matrix value
                                matrix_updated=self.shared_bids_b,
                                matrix_updated_ij=self.shared_bids_b.loc[bid_msg.task_id, bid_msg.target_agent_id],

                                # > Updated priority value
                                matrix_priority_updated=self.shared_bids_priority_beta,
                                priority_updated_ij=self.shared_bids_priority_beta.loc[bid_msg.task_id, bid_msg.target_agent_id],

                                # -> Source matrix
                                # > Source matrix value
                                matrix_source=None,
                                matrix_source_ij=bid_msg.value,

                                # > Source priority value
                                matrix_priority_source=None,
                                priority_source_ij=bid_msg.priority,

                                reset=True
                                )

        # -> Update allocation
        self.update_allocation()

        # -> If state has changed, update local states (only publish when necessary)
        self.check_publish_state_change()

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

        # -> Action is not None
        if allocation_state is not None:
            # -> Merge received allocation into current allocation
            self.shared_allocations_a, self.shared_allocations_priority_alpha = self.priority_merge(
                task_id=allocation_msg.task_id,
                agent_id=allocation_msg.target_agent_id,

                # -> Updated matrix
                # > Updated matrix value
                matrix_updated=self.shared_allocations_a,
                matrix_updated_ij=self.shared_allocations_a.loc[allocation_msg.task_id, allocation_msg.target_agent_id],

                # > Updated priority value
                matrix_priority_updated=self.shared_allocations_priority_alpha,
                priority_updated_ij=self.shared_allocations_priority_alpha.loc[allocation_msg.task_id, allocation_msg.target_agent_id],

                # -> Source matrix
                # > Source matrix value
                matrix_source=None,
                matrix_source_ij=allocation_state,

                # > Source priority value
                matrix_priority_source=None,
                priority_source_ij=allocation_msg.priority,

                reset=True
            )

            # -> Update allocation
            self.update_allocation()

            # -> If state has changed, update local states (only publish when necessary)
            self.check_publish_state_change()

    @abstractmethod
    def task_msg_subscriber_callback(self, task_msg):
        """
        Callback for task messages, create new task. add to local tasks and update local states, and select new task

        :param task_msg: TeamCommStamped.msg message
        """
        pass

    @abstractmethod
    def team_msg_subscriber_callback(self, team_msg):
        """
        Callback for team messages: consensus phase of the CBAA algorithm

        :param team_msg: TeamCommStamped.msg message
        """
        pass

    # >>>> Node specific
    @abstractmethod
    def __update_situation_awareness(self, task_list: Optional[List[Task]], fleet: Optional[List[Agent]]) -> None:
        """
        Update local states with new tasks and fleet dicts. Add new tasks and agents to local states and extend
        local states with new rows and columns for new tasks and agents. Remove tasks and agents from local states if
        they are no longer in the task list or fleet.

        :param task_list: Tasks dict
        :param fleet: Fleet dict
        """
        pass

    @abstractmethod
    def __update_shared_states(
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
        pass

    # ---------------- Processes
    # >>>> Base
    def bid(self, task: Task, agent_lst: list[Agent]) -> list[dict]:
        """
        Bid for a task

        :param task: Task object
        :param agent_lst: List of agents to compute bids for

        :return: Bid(s) list, with target agent id and corresponding bid and allocation action values
        """

        if self.env is None:
            # -> Return 0 bids for all agents as the environment is not available
            self.get_logger().warning("!!!!!! WARNING: Environment not available")
            return []

        # -> If no bid evaluation function, return empty list
        if self.bid_evaluation_function is None:
            return []

        # -> Compute bids
        task_bids = self.bid_evaluation_function(
            task=task,
            agent_lst=agent_lst,
            shared_bids_b=self.shared_bids_b,
            environment=self.env,
            logger=self.get_logger()
        )

        # -> Store bids to local bids matrix
        for bid in task_bids:
            # > Bid
            self.local_bids_c.loc[task.id, bid["agent_id"]] = bid["bid"]

            # > Allocation
            self.local_allocations_d.loc[task.id, bid["agent_id"]] = bid["allocation"]

        # -> Set task bid reference state
        task.local["bid(s)_reference_state"] = self.agent.state

        return task_bids

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
        msg.memo = dumps(
            self.get_state(
                state_awareness=True,
                local_allocation_state=False,
                shared_allocation_state=True,
                serialised=True
            )
        )

        # -> Publish message
        self.fleet_msgs_pub.publish(msg)

    def publish_goal_msg(self, task_id: str, meta_action: str):
        """
        Publish goal to the robot's goal topic for execution as TeamCommStamped message

        :param task_id: task id
        :param meta_action: action to take, can be "assign" or "cancel"
        """

        # -> Create message
        msg = TeamCommStamped()

        # > Metadata
        msg.stamp = timestamp_to_ros_time(timestamp=self.current_timestamp).to_msg()
        msg.trace = [self.id]

        # > Tracker
        msg.source = self.id
        msg.target = task_id

        msg.meta_action = meta_action

        # msg.memo = dumps(self.task_log[task_id].to_dict())    # TODO: Cleanup
        memo = {
            "agent": self.agent.to_dict(),
            "task": self.task_log[task_id].to_dict()
        }

        msg.memo = dumps(memo)

        # -> Publish message
        self.goals_pub.publish(msg)

        # self.get_logger().info(f"         > Published goal msg: {meta_action} task {task_id}")

    # >>>> Node specific
    @abstractmethod
    def update_allocation(self):
        """
        Update allocation state for the agent
        """
        pass

    # ---------------- tools
    @staticmethod
    def action_to_allocation_state(action: int) -> Optional[int]:
        """
        Convert action to allocation state

        :param action: Action to convert
        :return: int
        """

        if action == 1:
            return 0            # Free allocation
        elif action == 2:
            return -1           # Blacklisted
        elif action == 3:
            return 1            # Imposed allocation
        else:
            return None         # No action

    def rebroadcast(self, msg, publisher) -> tuple:
        """
        Conditional rebroadcast of a message based on the trace
        The trace ensures every msg is only broadcast once per robot
        The second output (trace_flag), returns whether a msg was re-broadcasted based on the trace

        :param msg: Message to rebroadcast
        :param publisher: Publisher to use

        :return: tuple (msg, trace_flag)
        """

        if not self.rebroadcast_received_msgs:
            return msg, False

        # -> If self not in trace, add self to trace and re-broadcast
        if self.id not in msg.trace:
            msg.trace.append(self.id)

            publisher.publish(msg)
            return msg, True

        # -> If self already in trace, do not re-broadcast
        else:
            return msg, False

    def deserialise(self, state: str) -> dict:
        """
        Deserialise the state

        :param state: State to deserialise
        :return: dict
        """

        deserialised_state = loads(state)

        states = self.get_state(
            state_awareness=False,
            local_allocation_state=True,
            shared_allocation_state=True,
            serialised=False
        )

        pandas_dicts = states.keys()

        for key, value in deserialised_state.items():
            # -> If the value is in the local or shared allocation state ...
            if key in pandas_dicts:
                # -> Convert to pandas dataframe or series
                deserialised_state[key] = pd.DataFrame(value) if isinstance(value, dict) else pd.Series(value)

        return deserialised_state

    @abstractmethod
    def priority_merge(
            self,
            task_id: str,
            agent_id: str,

            # -> Updated matrix
            # > Updated matrix value
            matrix_updated: pd.DataFrame,
            matrix_updated_ij: float,

            # > Updated priority value
            matrix_priority_updated: pd.DataFrame,
            priority_updated_ij: float,

            # -> Source matrix
            # > Source matrix value
            matrix_source: Optional[pd.DataFrame],
            matrix_source_ij: float,

            # > Source priority value
            matrix_priority_source: Optional[pd.DataFrame],
            priority_source_ij: float,

            reset: bool = False
        ):
        """
        Priority merge function for the agent
        """
        pass