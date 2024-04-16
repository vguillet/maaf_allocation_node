
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
import sys
from abc import abstractmethod
from typing import List, Optional, Tuple
from json import dumps, loads
import warnings
from copy import deepcopy
from pprint import pprint, pformat

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

# Local Imports
try:
    from rlb_simple_sim.Scenario import Scenario

    from orchestra_config.orchestra_config import *  # KEEP THIS LINE, DO NOT REMOVE
    from orchestra_config.sim_config import *

    from maaf_msgs.msg import TeamCommStamped, Bid, Allocation

    from maaf_tools.datastructures.task.TaskLog import TaskLog
    from maaf_tools.datastructures.task.Task import Task

    from maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.datastructures.agent.AgentState import AgentState
    from maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.tools import *
    from maaf_tools.Singleton import SLogger

    # > CBAA
    from maaf_allocation_node.allocation_logics.CBAA.bidding_logics.anticipated_action_task_interceding_agent import anticipated_action_task_interceding_agent
    from maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid

    # > CBBA
    from maaf_allocation_node.allocation_logics.CBBA.bidding_logics.interceding_skill_based_bid_amplifier import interceding_skill_based_bid_amplifier
    from maaf_allocation_node.allocation_logics.CBBA.bidding_logics.graph_weigthed_manhattan_distance_bundle_bid import graph_weighted_manhattan_distance_bundle_bid

except ModuleNotFoundError:
    from rlb_simple_sim.rlb_simple_sim.Scenario import Scenario

    from orchestra_config.orchestra_config import *  # KEEP THIS LINE, DO NOT REMOVE
    from orchestra_config.orchestra_config.sim_config import *

    from maaf_msgs.msg import TeamCommStamped, Bid, Allocation

    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog
    from maaf_tools.maaf_tools.datastructures.task.Task import Task

    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.AgentState import AgentState
    from maaf_tools.maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.maaf_tools.tools import *
    from maaf_tools.maaf_tools.Singleton import SLogger

    # > CBAA
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBAA.bidding_logics.anticipated_action_task_interceding_agent import anticipated_action_task_interceding_agent
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid

    # > CBBA
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBBA.bidding_logics.interceding_skill_based_bid_amplifier import interceding_skill_based_bid_amplifier
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBBA.bidding_logics.graph_weigthed_manhattan_distance_bundle_bid import graph_weighted_manhattan_distance_bundle_bid

##################################################################################################################

FREE_ALLOCATION = 0
BLACKLISTED = -1
IMPOSED_ALLOCATION = 1
NO_ACTION = None


class MAAFAgent(Node):
    def __init__(self,
                 node_name: str,
                 id: str = None,
                 name: str = None,
                 skillset: List[str] = None,
                 bid_estimator=None
                 ):
        """
        maaf agent class
        """

        # ----------------------------------- Node Configuration
        Node.__init__(
            self,
            node_name=node_name,
        )

        logger = SLogger()
        logger.logger = self.get_logger()

        # TODO: Clean up
        # -> Get launch parameters configuration
        self.declare_parameters(
            namespace="",
            parameters=[
                ("scenario_id", "simple_sim"),
            ]
        )

        scenario_id = self.get_parameter("scenario_id").get_parameter_value().string_value

        self.scenario = Scenario(scenario_id=scenario_id, load_only=True, logger=self.get_logger())

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
            self.skillset = self.scenario.fleet_skillsets[self.id]
        else:
            self.skillset = skillset

        # ---- Bid evaluation function
        if bid_estimator is None:
            self.bid_evaluation_function = None
        elif bid_estimator is not None:
            self.bid_evaluation_function = bid_estimator

        # TODO: Cleanup
        if self.id in self.scenario.fleet_bids_mechanisms.keys():
            if self.scenario.fleet_bids_mechanisms[self.id] == "graph_weighted_manhattan_distance_bid":
                # self.bid_evaluation_function = graph_weighted_manhattan_distance_bid           # CBAA
                self.bid_evaluation_function = graph_weighted_manhattan_distance_bundle_bid    # CBBA
                self.hierarchy_level = 0

            elif self.scenario.fleet_bids_mechanisms[self.id] == "anticipated_action_task_interceding_agent":
                # self.bid_evaluation_function = anticipated_action_task_interceding_agent       # CBAA
                self.bid_evaluation_function = interceding_skill_based_bid_amplifier           # CBBA
                self.hierarchy_level = 1

        # ---- Multi-hop behavior
        self.rebroadcast_received_msgs = False

        # ---- Environment
        self.environment = None

        # ---- Fleet and task log
        self.__setup_fleet_and_tasklog()

        # ---- Listeners
        self.__env_update_listeners = []
        self.__pose_update_listeners = []

        # -> Connect listeners
        # self.fleet.add_on_edit_list_listener()
        # self.tasklog.add_on_edit_list_listener()

        # ---- Node connections
        self.__setup_node_pubs_subs()

        # ---- Allocation states
        self.__setup_allocation_base_states()
        # self._setup_allocation_additional_states()

        # -> Initialise previous state hash
        self.prev_allocation_state_hash_dict = None
        # TODO: ALWAYS SET IN PARENT CLASS ONCE AGAIN

        self.get_logger().info(f"\n> Agent {self.id} initialised: " +
                               f"\n     Hierarchy level: {self.hierarchy_level}" +
                               f"\n     Skillset: {self.skillset}" +
                               f"\n     Bid evaluation: {self.bid_evaluation_function}\n"
                               )

    # ============================================================== INIT
    def __setup_fleet_and_tasklog(self) -> None:
        # ---- Fleet data
        """
        Fleet dict of size N_a
        Agents ids are obtained using the self.id_card property
        """
        # -> Create fleet object
        self.fleet = Fleet()

        # # -> Fill with initial data
        # # > Retrieve initial fleet data from parameters
        # fleet_data = []
        #
        # > Add initial fleet data to the fleet object
        # self.fleet.from_dict(maaflist_dict=fleet_data)

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
                    state=AgentState(
                        agent_id=self.id,
                        _timestamp=self.current_timestamp,
                        battery_level=100,
                        stuck=False,
                        x=0,
                        y=0,
                        z=0,
                        u=0,
                        v=0,
                        w=0
                    ),
                    plan=Plan()
                )
            )

        # -> Set get_timestamp method
        def get_timestamp():
            return self.current_timestamp

        self.agent.state.set_get_timestamp(get_timestamp)

        # -> Add timestamp sync to the self agent
        def timestamp_sync(agent: Agent):
            # > Call timestamp to force refresh it
            agent.state.timestamp

        self.agent.add_pre_asdict_subscriber(timestamp_sync)

        # ---- Task log
        """
        Tasks dict of size N_t
        Tasks are created using the self.create_task method
        """
        # -> Create task log object
        self.tasklog = TaskLog()
        self.tasklog.init_tasklog(agent_id=self.agent.id)

        # # -> Fill with initial data
        # # > Retrieve initial task data from parameters
        # tasklog_data = []
        #
        # # > Add initial task data to the task log object
        # self.tasklog.from_dict(maaflist_dict=tasklog_data)

    def __setup_node_pubs_subs(self) -> None:
        """
        Setup node publishers and subscribers
        Separated from __init__ for readability
        :return: None
        """

        # ----------------------------------- Subscribers
        # ---------- simulator_signals
        self.simulator_signals_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_simulator_signals,
            callback=self.__simulator_signals_callback,
            qos_profile=qos_simulator_signals
        )

        if RUN_MODE == OPERATIONAL:
            # ---------- fleet_msgs
            self.fleet_msgs_sub = self.create_subscription(
                msg_type=TeamCommStamped,
                topic=topic_fleet_msgs,
                callback=self._team_msg_subscriber_callback,
                qos_profile=qos_fleet_msgs
            )

        elif RUN_MODE == SIM:
            # ---------- fleet_msgs_filtered
            self.fleet_msgs_sub = self.create_subscription(
                msg_type=TeamCommStamped,
                topic=topic_fleet_msgs_filtered,
                callback=self._team_msg_subscriber_callback,
                qos_profile=qos_fleet_msgs
            )

        # ---------- environment
        # TODO: Cleanup
        self.env_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_environment,
            callback=self.__env_callback,
            qos_profile=qos_env
        )

        # ---------- task
        self.task_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_tasks,
            callback=self._task_msg_subscriber_callback,
            qos_profile=qos_tasks
        )

        # ---------- pose
        self.pose_sub = self.create_subscription(
            msg_type=PoseStamped,
            topic=f"/{self.id}{topic_pose}",
            callback=self.__pose_subscriber_callback,
            qos_profile=qos_pose
        )

        # ---------- bids
        self.bid_sub = self.create_subscription(
            msg_type=Bid,
            topic=topic_bids,
            callback=self.__bid_subscriber_callback,
            qos_profile=qos_intercession
        )

        # ---------- allocation
        self.allocation_sub = self.create_subscription(
            msg_type=Allocation,
            topic=topic_allocations,
            callback=self.__allocation_subscriber_callback,
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
        # ---- Fleet msg update timer
        # self.fleet_msg_update_timer = self.create_timer(
        #     timer_period_sec=0.1,
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
            index=self.tasklog.ids_pending,
            columns=self.fleet.ids_active,
        )

        self.local_bids_c = self.local_bids_c.astype(float)

        # > Include in self agent shared state
        self.agent.shared["local_bids_c"] = self.local_bids_c

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
            index=self.tasklog.ids_pending,
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
            index=self.tasklog.ids_pending,
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
            index=self.tasklog.ids_pending,
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
            index=self.tasklog.ids_pending,
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
            index=self.tasklog.ids_pending,
            columns=self.fleet.ids_active
        )

        """
        Shared allocations priority matrix alpha of size N_t x N_u:
        > priority value corresponding to each allocation in shared_allocations_ai

        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.shared_allocations_priority_alpha = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.tasklog.ids_pending,
            columns=self.fleet.ids_active
        )

    def _setup_allocation_additional_states(self) -> None:
        """
        Setup additional method-dependent allocation states for the agents
        Optional method to be implemented in child classes
        """
        pass

    def reset_allocation_states(self) -> None:
        """
        Reset allocation states for the agents
        """
        # -> Reset fleet and task log
        self.__setup_fleet_and_tasklog()

        # -> Reset allocation states
        self.__setup_allocation_base_states()
        self._setup_allocation_additional_states()

    # ============================================================== Listeners
    # ---------------- Env update
    def add_on_env_update_listener(self, listener) -> None:
        """
        Add listener for environment updates

        :param listener: callable
        """
        self.__env_update_listeners.append(listener)

    def call_on_env_update_listeners(self, environment) -> None:
        """
        Call all environment update listeners
        """
        for listener in self.__env_update_listeners:
            listener(environment)

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
        return len(self.tasklog.tasks_pending)

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
    def allocation_state_hash_exclusion(self) -> List[str]:
        """
        Optional
        List of allocation states to exclude from the state hash
        """
        return []

    @property
    def allocation_state_hash_dict(self) -> str:
        """
        Hash of the agent allocation state

        :return: dict
        """
        # -> Convert to series and dataframe to immutable hashable objects
        immutable_state = {}

        state = self.get_state(
            environment=False,
            state_awareness=False,
            local_allocation_state=True,
            shared_allocation_state=True,
            serialised=True
            )

        # -> Remove all keys in exclusion list
        for key in self.allocation_state_hash_exclusion:
            if key in state.keys():
                del state[key]

        # -> Hash all values
        for key, value in state.items():
            if isinstance(value, pd.Series):
                immutable_state[key] = hash(str(value.to_string()))
            elif isinstance(value, pd.DataFrame):
                immutable_state[key] = hash(str(value.to_string()))
            else:
                immutable_state[key] = hash(str(value))

        return dumps(immutable_state)

    @property
    def allocation_state_change(self) -> bool:
        """
        Check if the shared allocation state has changed

        :return: bool
        """
        # # -> Compare hash of current state with hash of previous state
        # for key, value in self.allocation_state_hash_dict.items():
        #     # -> If any value has changed, return True
        #     if value != self.prev_allocation_state_hash_dict[key]:
        #         return True
        #
        # # -> If no value has changed, return False
        # return False

        if self.allocation_state_hash_dict != self.prev_allocation_state_hash_dict:
            return True
        else:
            return False

    def check_publish_state_change(self):
        # -> If state has changed, update local states (only publish when necessary)
        if self.allocation_state_change:
            # -> Update previous state hash
            self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

            # -> Publish allocation state to the fleet
            self.publish_allocation_state_msg()

            # self.get_logger().info(f"         < {self.id} Published allocation state update")
        else:
            return
            # self.get_logger().info(f"         < {self.id} No allocation state change")

    # >>>> Base states grouped getter
    def get_state(self,
                  environment: bool = False,
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

        if environment:
            if not serialised:
                state["environment"] = self.environment
            else:
                try:
                    state["environment"] = graph_to_json(graph=self.environment["graph"], pos=self.environment["pos"])
                except:
                    state["environment"] = None

        # !!!!! All states in state awareness must be maaf_list_dataclasses !!!!!
        if state_awareness:
            if not serialised:
                state = {**state, **self.state_awareness}
            else:
                serialised_state = {}

                for key, value in self.state_awareness.items():
                    serialised_state[key] = value.asdict()

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
            "tasklog": self.tasklog,
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
            "shared_allocations_priority_alpha": self.shared_allocations_priority_alpha,
            **self.additional_shared_states
        }

    @property
    def additional_shared_states(self) -> dict:
        """
        OPTIONAL
        Additional (algo specific) shared states at current time step (not serialised)
        !!!! All entries must be dataframes !!!!

        :return: dict
        """
        return {}

    @property
    @abstractmethod
    def local_allocation_state(self) -> dict:
        """
        Local allocation state at current time step (not serialised)
        !!!! All entries must be dataframes !!!!

        :return: dict
        """
        pass

    # ============================================================== METHODS
    # ---------------- Callbacks
    # >>>> Base
    def __simulator_signals_callback(self, msg: TeamCommStamped):
        if msg.meta_action == "order 66":
            self.get_logger().info("Received order 66: Terminating simulation")

            # -> Terminate node
            self.destroy_node()

            # -> Terminate script
            sys.exit()

    def __env_callback(self, msg: TeamCommStamped) -> None:   # TODO: Cleanup
        if self.environment is None:    # TODO: Review environment management logic

            self.environment = json_to_graph(graph_json=msg.memo)

            self.environment["all_pairs_shortest_paths"] = dict(nx.all_pairs_shortest_path(self.environment["graph"]))

            self.get_logger().info(f"         < Received environment update")

            # # -> Display the graph
            # nx.draw(self.environment["graph"], pos=self.environment["pos"])
            # plt.show()

            # -> Recompute local bids for all tasks
            for task in self.tasklog.tasks_pending:
                task_bids = self._bid(task, [self.agent])

                # -> Store bids to local bids matrix
                for bid in task_bids:
                    # > Bid
                    self.local_bids_c.loc[task.id, bid["agent_id"]] = bid["bid"]

                    # > Allocation
                    self.local_allocations_d.loc[task.id, bid["agent_id"]] = bid["allocation"]

        # -> Call environment update listeners
        self.call_on_env_update_listeners(environment=self.environment)

    def __pose_subscriber_callback(self, pose_msg) -> None:
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

        # # > Timestamp
        # self.agent.state.timestamp = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9

        # self.publish_allocation_state_msg()     # TODO: Cleanup

        # self.get_logger().info(f"         < Received state update ({self.agent.state.x},{self.agent.state.y},{self.agent.state.z})")

        # self.compute_bids()

        # -> Publish goal (necessary to publish updated paths if computed before
        self.publish_goal_msg(meta_action="update", traceback="Pose update")

        # -> Call pose update listeners
        self.call_on_pose_update_listeners()

    def __fleet_msg_update_timer_callback(self) -> None:
        """
        Callback for the fleet message update timer
        """
        # -> Publish allocation state to the fleet
        self.publish_allocation_state_msg()

    def __bid_subscriber_callback(self, bid_msg: Bid) -> None:
        """
        Callback for the bid subscriber

        :param bid_msg: Bid message
        """

        self.get_logger().info(f"{self.id} < Received bid: \n    Task id: {bid_msg.task_id}\n    Agent id: {bid_msg.target_agent_id}\n    Value: {bid_msg.value}\n    Priority: {bid_msg.priority}")

        # -> Check if bid is for a task the agent is aware of
        if bid_msg.task_id not in self.tasklog.ids:
            self.get_logger().info(f"!!! WARNING: Received bid for task {bid_msg.task_id} not in task log")
            return
        # -> Check if bid is for an agent the agent is aware of
        elif bid_msg.target_agent_id not in self.fleet.ids_active:
            self.get_logger().info(f"!!! WARNING: Received bid for agent {bid_msg.target_agent_id} not in fleet")
            return

        # -> Priority merge received bid into current bids b
        self.shared_bids_b, self.shared_bids_priority_beta = self.priority_merge(
                                # Logging
                                task_id=bid_msg.task_id,
                                agent_id=bid_msg.target_agent_id,

                                # Merging
                                matrix_updated_ij=self.shared_bids_b.loc[bid_msg.task_id, bid_msg.target_agent_id],
                                matrix_source_ij=bid_msg.value,
                                priority_updated_ij=self.shared_bids_priority_beta.loc[bid_msg.task_id, bid_msg.target_agent_id],
                                priority_source_ij=bid_msg.priority,

                                greater_than_zero_condition=True,

                                # Reset
                                currently_assigned=None,
                                reset=False     # TODO: REVIEW (was True before refactor..?)
                                )

        # -> Update allocation
        self.update_allocation()

        # -> If state has changed, update local states (only publish when necessary)
        self.check_publish_state_change()

    def __allocation_subscriber_callback(self, allocation_msg: Allocation) -> None:
        """
        Callback for the allocation subscriber

        :param allocation_msg: Allocation message
        """

        self.get_logger().info(f"{self.id} < Received allocation: \n    Task id: {allocation_msg.task_id}\n    Agent id: {allocation_msg.target_agent_id}\n    Action: {allocation_msg.action}\n    Priority: {allocation_msg.priority}")

        # -> Check if bid is for a task the agent is aware of
        if allocation_msg.task_id not in self.tasklog.ids:
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
                # Logging
                task_id=allocation_msg.task_id,
                agent_id=allocation_msg.target_agent_id,

                # Merging
                matrix_updated_ij=self.shared_allocations_a.loc[allocation_msg.task_id, allocation_msg.target_agent_id],
                matrix_source_ij=allocation_state,
                priority_updated_ij=self.shared_allocations_priority_alpha.loc[allocation_msg.task_id, allocation_msg.target_agent_id],
                priority_source_ij=allocation_msg.priority,

                greater_than_zero_condition=False,

                # Reset
                currently_assigned=None,
                reset=False     # TODO: REVIEW (was True before refactor..?)
            )

            # -> Update allocation
            self.update_allocation()

            # -> If state has changed, update local states (only publish when necessary)
            self.check_publish_state_change()

    @abstractmethod
    def _task_msg_subscriber_callback(self, task_msg):
        """
        Callback for task messages, create new task. add to local tasks and update local states, and select new task

        :param task_msg: TeamCommStamped.msg message
        """
        pass

    @abstractmethod
    def _team_msg_subscriber_callback(self, team_msg):
        """
        Callback for team messages: consensus phase of the CBAA algorithm

        :param team_msg: TeamCommStamped.msg message
        """
        pass

    # >>>> Node specific
    @abstractmethod
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
        raise NotImplementedError

    @abstractmethod
    def update_shared_states(self,
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

    # ---------------- Processes
    # >>>> Base
    def publish_allocation_state_msg(self):
        """
        Publish allocation state to the fleet as TeamCommStamped.msg message
        """

        # TODO: Remove once pathfinding handled by controller -----
        # -> Update path to start task to ensure agent path starts from current position
        if self.agent.plan:
            start_task = self.tasklog[self.agent.plan[0]]
            self.update_path(
                source=self.agent.id,
                source_pos=(self.agent.state.x, self.agent.state.y),
                target=start_task.id,
                target_pos=(start_task.instructions["x"], start_task.instructions["y"])
            )
        # TODO: ---------------------------------------------------

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
                environment=False,
                state_awareness=True,
                local_allocation_state=False,
                shared_allocation_state=True,
                serialised=True
            )
        )

        # -> Publish message
        self.fleet_msgs_pub.publish(msg)

    def publish_goal_msg(self,
                         meta_action: str = "empty",     # empty/update,
                         *args, **kwargs
                         ):
        """
        Publish goal to the robot's goal topic for execution as TeamCommStamped message

        :param meta_action: action to take, can be "empty" or "update"
        """

        # -> Create message
        msg = TeamCommStamped()

        # > Metadata
        msg.stamp = timestamp_to_ros_time(timestamp=self.current_timestamp).to_msg()
        msg.trace = [self.id]

        # > Tracker
        msg.source = self.id
        msg.meta_action = meta_action

        if meta_action == "update":
            # TODO: Remove once pathfinding handled by controller -----
            # -> Update path to start task to ensure agent path starts from current position
            if self.agent.plan:
                start_task = self.tasklog[self.agent.plan[0]]
                self.update_path(
                    source=self.agent.id,
                    source_pos=(self.agent.state.x, self.agent.state.y),
                    target=start_task.id,
                    target_pos=(start_task.instructions["x"], start_task.instructions["y"])
                )

            # update_success = self.agent.update_plan(tasklog=self.tasklog)
            #
            # if not update_success:
            #     self.get_logger().info(f"!!! WARNING: Plan update failed for agent {self.id}")

            # -> Embed paths in agent
            agent_dict = self.agent.asdict()
            agent_dict["plan"]["paths"] = self.agent.plan.get_paths(
                agent_id=self.agent.id,
                tasklog=self.tasklog,
                requirement=None,  # TODO: Correct once path requirements are implemented
                selection="shortest"  # TODO: Correct once path requirements are implemented
            )
            # TODO: ---------------------------------------------------

            # -> Gather tasks in plan
            tasks = {}

            for task_id in self.agent.plan:
                tasks[task_id] = self.tasklog[task_id].asdict()

            msg.target = self.id
            memo = {
                "agent": agent_dict,
                "tasks": tasks,
                "args": args,
                "kwargs": kwargs
            }

        else:
            # TODO: Cleanup (used to push msg to all channel to init all trackers across nodes
            msg.target = "all"
            memo = {"agent": self.agent.asdict()}

        # > Dump memo
        msg.memo = dumps(memo)

        # -> Publish message
        self.goals_pub.publish(msg)

        # self.get_logger().info(f"         > Published goal msg: {meta_action} task {task_id}")

    # >>>> Node specific
    # ---------------- tools
    @staticmethod
    def action_to_allocation_state(action: int) -> Optional[int]:
        """
        Convert action to allocation state

        :param action: Action to convert
        :return: int
        """

        if action == 1:
            return FREE_ALLOCATION
        elif action == 2:
            return BLACKLISTED
        elif action == 3:
            return IMPOSED_ALLOCATION
        else:
            return NO_ACTION

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
            environment=False,
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

    def update_path(self,
                    source: str,
                    source_pos: tuple,
                    target: str,
                    target_pos: tuple,
                    ) -> None:
        """
        Update path to a task for an agent

        :param source: Source node
        :param source_pos: Source node position
        :param target: Target node
        :param target_pos: Target node position
        """

        current_path = self.tasklog.get_path(
            source=source,
            target=target,
            requirement=None,
            selection="shortest"
        )

        if current_path:
            current_path = current_path["path"]

            if source_pos in current_path:
                path = current_path[current_path.index(source_pos):]
                compute_path = False

            else:
                compute_path = True
        else:
            compute_path = True

        if compute_path:
            # -> Find the Manhattan distance between the agent and the task
            path = self.environment["all_pairs_shortest_paths"][source_pos][target_pos]
            # path = nx.shortest_path(self.environment["graph"], source_pos, target_pos)
            # path = nx.astar_path(environment["graph"], source_pos, target_pos, weight="weight")

        # > Store path to agent task log
        self.tasklog.add_path(
            source_node=source,
            target_node=target,
            path={
                "id": f"{self.id}:{source}_{target}",
                "path": path,
                "requirements": ["ground"]
            },
            two_way=False,
            selection="latest"
        )

    def priority_merge(self,
                       # Logging
                       task_id: str,
                       agent_id: str,

                       # Merging
                       matrix_updated_ij: float,
                       priority_updated_ij: float,

                       matrix_source_ij: float,
                       priority_source_ij: float,

                       # Reset
                       currently_assigned: Optional[bool] = None,
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

        :param greater_than_zero_condition: Condition to check if the source matrix value is greater than zero

        ### Reset variables
        :param currently_assigned: Flag to check if the task is currently assigned
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
            if reset and currently_assigned:
                # -> Cancel goal
                self._drop_task(
                    task_id=task_id,
                    reset=True,    # This reset is not for task drop, but for y (hence True)
                    traceback="Priority merge reset",
                    logger=True
                )

            # elif reset and agent_id == self.agent.id:   # TODO: REVIEW THEORY OF THIS CHANGE
            #     # -> Cancel goal
            #     self._drop_task(
            #         task_id=self.agent.plan.current_task_id,
            #         reset=False,    # This reset is not for task drop, but for y (hence False)
            #         traceback="Priority merge reset",
            #         logger=True
            #     )

        # -> If updated priority is higher
        elif priority_source_ij < priority_updated_ij:
            # -> Do nothing as updated priority is higher, therefore keep updated matrix value and priority value
            pass

        # -> If priorities are equal
        else:
            # Apply other tie-breakers
            # TODO: Implement tie-breakers, for now larger value is kept
            matrix_updated_ij = max(matrix_updated_ij, matrix_source_ij)
            # matrix_updated_ij = matrix_source_ij

        return matrix_updated_ij, priority_updated_ij

    # >>>> Node specific
    @abstractmethod
    def _bid(self, task: Task, agent_lst: list[Agent]) -> list[dict]:
        """
        Bid for a task

        :param task: Task object
        :param agent_lst: List of agents to compute bids for

        :return: Bid(s) list, with target agent id and corresponding bid and allocation action values
        """
        raise NotImplementedError

    @abstractmethod
    def _drop_task(self, task_id: str, reset: bool, traceback: str, logger: bool):
        """
        Drop a task from the task list x or y. If reset is True, the task is removed from the task list x, otherwise it is removed from the task list y.

        :param task_id: Task id
        :param reset: Flag to reset task value to zero if source priority is higher
        :param traceback: Reason for dropping the task
        :param logger: Flag to log the task drop
        """
        raise NotImplementedError
