
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
import numpy as np
import pandas as pd
from json import dumps, loads
from pprint import pprint, pformat

# Libs
# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# TODO: Cleanup
# NetworkX
import networkx as nx

# Matplotlib
import matplotlib.pyplot as plt


# Local Imports
from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
from .node_config import *
from .task_dataclasses import Task, Task_log
from .fleet_dataclasses import Agent, Fleet
from .state_dataclasses import Agent_state
from .tools import *

from .Bidding_logics.random_bid import random_bid
from .Bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid


##################################################################################################################


class MAAFAgent(Node):
    def __init__(
            self,
            node_name: str,
            id: str = None,
            name: str = None,
            skillset: List[str] = None
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
            self.skillset = self.get_parameter("skillset").get_parameter_value().string_array_value
        else:
            self.skillset = skillset

        # TODO: Implement bid evaluation function selection logic
        # self.bid_evaluation_function = random_bid
        self.bid_evaluation_function = graph_weighted_manhattan_distance_bid

        self.env = None

        # ---- Fleet and task log
        self.__setup_fleet_and_task_log()

        # ---- Node connections
        self.__setup_node_pubs_subs()

        # ---- Allocation states
        self.__setup_allocation_base_states()

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
            # ---------- /fleet/fleet_msgs
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            )

            self.fleet_msgs_sub = self.create_subscription(
                msg_type=TeamCommStamped,
                topic=f"/fleet/fleet_msgs",
                callback=self.team_msg_subscriber_callback,
                qos_profile=qos
            )

        elif RUN_MODE == SIM:
            # ---------- /sim/fleet/fleet_msgs_filtered
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            )

            self.fleet_msgs_sub = self.create_subscription(
                msg_type=TeamCommStamped,
                topic=f"/sim/fleet/fleet_msgs_filtered",
                callback=self.team_msg_subscriber_callback,
                qos_profile=qos
            )

        # ---------- /sim/environment
        # TODO: Cleanup
        self.env_subscriber = self.create_subscription(
            msg_type=TeamCommStamped,
            topic="/sim/environment",
            callback=self.env_callback,
            qos_profile=10
        )

        # ---------- /fleet/task
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
        )
        self.robot_task_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=f"/fleet/task",
            callback=self.task_msg_subscriber_callback,
            qos_profile=qos
        )

        # ---------- /robot_.../pose
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.robot_pose_sub = self.create_subscription(
            msg_type=PoseStamped,
            topic=f"/{self.id}/data/pose",
            callback=self.pose_subscriber_callback,
            qos_profile=qos
        )

        # ---------- /fleet/bids
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
        )

        self.fleet_bid_sub = self.create_subscription(
            msg_type=Bid,
            topic=f"/fleet/bids",
            callback=self.bid_subscriber_callback,
            qos_profile=qos
        )

        # ---------- /fleet/allocation
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
        )

        self.fleet_allocation_sub = self.create_subscription(
            msg_type=Allocation,
            topic=f"/fleet/allocations",
            callback=self.allocation_subscriber_callback,
            qos_profile=qos
        )

        # ----------------------------------- Publishers
        # ---------- /fleet/fleet_msgs
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
        )

        self.fleet_msgs_pub = self.create_publisher(
            msg_type=TeamCommStamped,
            topic=f"/fleet/fleet_msgs",
            qos_profile=qos
        )

        # ---------- /robot_.../goal
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
        )

        # Goals publisher
        self.goal_sequence_publisher = self.create_publisher(
            msg_type=TeamCommStamped,
            # topic=f"/{self.id}/control/goal", # TODO: Fix/clean up
            topic=f"/goal",
            qos_profile=qos
        )

        # ----------------------------------- Timers
        # ---- Fleet msg update timer
        self.fleet_msg_update_timer = self.create_timer(
            timer_period_sec=FLEET_MSG_UPDATE_TIMER,
            callback=self.fleet_msg_update_timer_callback
        )

        # ---- Auction timer
        # self.auction_timer = self.create_timer(
        #     timer_period=timedelta(seconds=0.1),
        #     callback=self.auction_timer_callback
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
        Current bids matrix b of size N_t x N_u: 
        > highest priority/value bids made across the fleet for each task and each agent

        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.current_bids_b = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.task_log.ids_pending,
            columns=self.fleet.ids_active,
        )

        self.current_bids_b = self.current_bids_b.astype(float)

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
    # >>>> Base

    # >>>> Situation states grouped
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
    def state_awareness_dict(self) -> dict:
        """
        State awareness at current time step (serialised)

        :return: dict
        """
        return {
            "tasks": self.task_log.to_list(),
            "fleet": self.fleet.to_list()
        }

    @property
    def shared_allocation_state_dict(self) -> dict:
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
    @abstractmethod
    def local_allocation_state_dict(self) -> dict:
        """
        Local allocation state at current time step (serialised)

        :return: dict
        """
        pass

    # ============================================================== METHODS
    # ---------------- Callbacks
    # >>>> Base
    # TODO: Cleanup
    def env_callback(self, msg: TeamCommStamped):
        if self.env is None:
            # self.get_logger().info(f"         > Received environment update")
            data = loads(msg.memo)
            self.env = {
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
                    self.local_bids_c.loc[task.id, bid["agent_id"]] = bid["bid"]

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

        # self.get_logger().info(f"         < Received state update")
        # self.get_logger().info(f"{pformat(self.agent.state.to_dict())}")

    def fleet_msg_update_timer_callback(self) -> None:
        """
        Callback for the fleet message update timer
        """
        # -> Publish allocation state to the fleet
        self.publish_allocation_state_msg()

    @abstractmethod
    def bid_subscriber_callback(self, bid_msg: Bid) -> None:
        """
        Callback for the bid subscriber

        :param bid_msg: Bid message
        """
        pass

    @abstractmethod
    def allocation_subscriber_callback(self, allocation_msg: Allocation) -> None:
        """
        Callback for the allocation subscriber

        :param allocation_msg: Allocation message
        """
        pass

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
    pass

    # ---------------- Processes
    # >>>> Base
    def bid(self, task: Task, agent_lst: list[Agent]) -> list[dict]:
        """
        Bid for a task

        :param task: Task object
        :param agent_lst: List of agents to compute bids for

        :return: Bid(s) list, with target agent id and corresponding bid value
        """

        return self.bid_evaluation_function(
            task=task,
            agent_lst=agent_lst,
            env=self.env,
            logger=self.get_logger()
        )

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
        memo = self.task_log[task_id].to_dict()
        memo["cost"] = self.local_bids_c.loc[task_id, self.id]
        msg.memo = dumps(memo)

        # -> Publish message
        self.goal_sequence_publisher.publish(msg)

        # self.get_logger().info(f"         > Published goal msg: {meta_action} task {task_id}")

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

    def rebroadcast(self, msg, publisher):
        """
        Conditional rebroadcast of a message based on the trace
        The trace ensures every msg is only broadcast once per robot
        The second output (trace_flag), returns whether a msg was re-broadcasted based on the trace
        """
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

        pandas_dicts = self.local_allocation_state_dict.keys() | self.shared_allocation_state_dict.keys()

        for key, value in deserialised_state.items():
            # -> If the value is in the local or shared allocation state ...
            if key in pandas_dicts:
                # -> Convert to pandas dataframe or series
                deserialised_state[key] = pd.DataFrame(value) if isinstance(value, dict) else pd.Series(value)

        return deserialised_state
