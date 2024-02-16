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

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Local Imports
from .node_config import *
from .Tasks_dataclasses import Task, Task_log
from .Fleet_dataclasses import Agent, Fleet
from .Tools import *
from maaf_msgs.msg import TeamCommStamped

##################################################################################################################


class maaf_agent(Node):
    def __init__(
            self,
            agent_id: str,
            name: str,
            agent_class,
            hierarchy_level: int = 0,
            affiliations: list = [],
            specs: dict = None,
            skillset: list = None,
            bid_evaluation_function=None,
            fleet: Fleet = Fleet(),
            task_log: Task_log = Task_log()
    ):
        """
        maaf agent class

        :param agent_id: agent id (str)
        :param name: agent name (str)
        :param agent_class: agent class (str)
        :param hierarchy_level: agent hierarchy level (int, larger is higher)
        :param affiliations: agent affiliations (list of str)
        :param specs: agent specs (dict)
        :param skillset: agent skillset (list of str)
        :param bid_evaluation_function: function to evaluate bids (function)
        """

        # ---- Agent properties
        self.id = agent_id
        self.name = name
        self.agent_class = agent_class
        self.hierarchy_level = hierarchy_level
        self.affiliations = affiliations

        self.specs = specs
        self.skillset = skillset

        self.bid_evaluation_function = bid_evaluation_function

        # ---- Fleet data
        """
        Fleet dict of size N_a
        Agents ids are obtained using the self.id_card property
        """
        self.fleet = fleet

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
                    status="active"
                )
            )

        # ---- Tasks dict
        """
        Tasks dict of size N_t
        Tasks are created using the self.create_task method
        """
        self.task_log = task_log

        # ----------------------------------- Node Configuration
        Node.__init__(
            self,
            node_name=f"maaf_agent_{self.id}"
        )

        # ---- Node behavior
        self.verbose = 0

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
            # ---------- fleet_msgs
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            )

            self.fleet_msgs_sub = self.create_subscription(
                msg_type=TeamCommStamped,
                topic=f"/fleet_msgs",
                callback=self.team_msg_subscriber_callback,
                qos_profile=qos
            )

        elif RUN_MODE == SIM:
            # ---------- fleet_msgs_filtered
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            )

            self.fleet_msgs_sub = self.create_subscription(
                msg_type=TeamCommStamped,
                topic=f"/{self.id}/sim/fleet_msgs_filtered",
                callback=self.team_msg_subscriber_callback,
                qos_profile=qos
            )

        # ---------- robot_.../task
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
        )
        self.robot_task_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=f"/task",
            callback=self.task_msg_subscriber_callback,
            qos_profile=qos
        )

        # ---------- robot_.../pose
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.robot_pose_sub = self.create_subscription(
            msg_type=PoseStamped,
            topic=f"/{self.id}/pose",
            callback=self.pose_subscriber_callback,
            qos_profile=qos
        )

        # ----------------------------------- Publishers
        # ---------- fleet_msgs
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
        )

        self.fleet_msgs_pub = self.create_publisher(
            msg_type=TeamCommStamped,
            topic=f"/fleet_msgs",
            qos_profile=qos
        )

        # ---------- robot_.../goal
        qos = QoSProfile(depth=1)

        # Goals publisher
        self.goal_sequence_publisher = self.create_publisher(
            msg_type=TeamCommStamped,
            topic=f"/{self.id}/goal",
            qos_profile=qos
        )

        # ----------------------------------- Timers
        # ---- Fleet msg update timer
        self.fleet_msg_update_timer = self.create_timer(
            timer_period_sec=0.1,
            callback=self.fleet_msg_update_timer_callback
        )

        # ---- Auction timer
        # self.auction_timer = self.create_timer(
        #     timer_period=timedelta(seconds=0.1),
        #     callback=self.auction_timer_callback
        # )

        self.get_logger().info(f"MAAF agent {self.id}: Pubsubs initialized")

    # ============================================================== PROPERTIES
    # ---------------- Generic
    @property
    def current_timestamp(self):
        # -> Get current ROS time as timestamp
        time_obj = self.get_clock().now().to_msg()

        return timestamp_from_ros_time(time_obj)

    @property
    def id_card(self) -> dict:
        return self.fleet[self.id].to_dict()

    def __task_factory(
            self,
            task_id: str or int,
            task_type: str,
            task_affiliations: List[str],
            priority: int,
            task_instructions: List[str]) -> Task:
        """
        Construct a task object from the Task dataclass
        """

        return Task(
            # > Metadata
            id=task_id,
            type=task_type,
            creator=self.id,
            affiliations=task_affiliations,

            # > Task data
            priority=priority,
            instructions=task_instructions,

            # > Task status
            creation_timestamp=self.current_timestamp,
            termination_timestamp=None,
            status="pending"
        )

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
    def bid(self, task_data):
        """
        Bid for a task

        :param task_data: task data
        """
        return self.bid_evaluation_function(task_data)

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
    def check_rebroadcast(self, msg):
        """
        Conditional rebroadcast of a message based on the trace
        The trace ensures every msg is only broadcast once per robot
        The second output (trace_flag), returns whether a msg needs to be re-broadcast based on the trace
        """
        # -> If self not in trace, add self to trace and re-broadcast
        if self.id not in msg.trace:
            msg.trace.append(self.id)
            return msg, True

        # -> If self already in trace, do not re-broadcast
        else:
            return msg, False
