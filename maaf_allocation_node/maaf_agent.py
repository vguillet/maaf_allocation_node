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
from abc import abstractmethod
from typing import List, Optional
from datetime import datetime, timedelta
from random import randint

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Local Imports
from .node_config import *
from .maaf_task_dataclasses import Task, Task_log
from .maaf_fleet_dataclasses import Agent, Fleet
from .maaf_state_dataclasses import Agent_state
from .Tools import *
from maaf_msgs.msg import TeamCommStamped, Bid, Allocation

##################################################################################################################


def random_bid_evaluation(task: Task, agent_lst: list[Agent]) -> list[dict]:
    bids = []

    for agent in agent_lst:
        bids.append({
            "agent_id": agent.id,
            "bid": randint(0, 10000)
        })

    return bids


class maaf_agent(Node):
    def __init__(self):
        """
        maaf agent class
        """

        # ----------------------------------- Node Configuration
        Node.__init__(
            self,
            # node_name=f"{self.id}_CBAAwI_allocation_node",
            node_name=f"CBAAwI_allocation_node",
        )

        # ---- Agent properties
        # > declare all parameters at once
        self.declare_parameters(namespace="", parameters=[("id", ""), ("name", "")])

        self.id = self.get_parameter("id").get_parameter_value().string_value
        self.name = self.get_parameter("name").get_parameter_value().string_value

        # TODO: Implement local loading of configs
        self.agent_class = "Base"
        self.hierarchy_level = 0
        self.affiliations = []
        self.specs = {}
        self.skillset = []

        # TODO: Implement bid evaluation function selection logic
        self.bid_evaluation_function = random_bid_evaluation

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

        # ---- Tasks dict
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

        # ---- Node connections
        self.setup_node_pubs_subs()

    def setup_node_pubs_subs(self) -> None:
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
            topic=f"/fleet/allocation",
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
            topic=f"/{self.id}/control/goal",
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
    @property
    @abstractmethod
    def state(self):
        pass

    @property
    @abstractmethod
    def allocation_state(self):
        """
        Allocation state to publish to the fleet
        """
        pass

    # ============================================================== METHODS
    # ---------------- Callbacks
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
    def pose_subscriber_callback(self, pose_msg):
        """
        Callback for the pose subscriber

        :param pose_msg: PoseStamped message
        """
        pass

    @abstractmethod
    def fleet_msg_update_timer_callback(self):
        """
        Callback for the fleet message update timer
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
        Callback for team messages, consensus phase of the CBAA algorithm

        :param team_msg: TeamCommStamped.msg message
        """
        pass

    # ---------------- Processes
    def bid(self, task: Task, agent_lst: list[Agent]) -> list[dict]:
        """
        Bid for a task

        :param task: Task object

        :return: Bid(s) list, with target agent id and corresponding bid value
        """
        return self.bid_evaluation_function(task=task, agent_lst=agent_lst)

    @abstractmethod
    def publish_allocation_state_msg(self):
        """
        Publish allocation state to the fleet as TeamComm message
        """
        pass

    @abstractmethod
    def publish_goal(self, task_id):
        """
        Publish goal to the robot's goal topic for execution as Goal messaged

        :param task_id: task id
        """
        pass

    # ---------------- Tools
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
