
##################################################################################################################

"""
Core class of the MAAF framework. Contains all the basic components, interfaces, and methods for an agent to operate
in a MAAF coordination network. Must be inherited by all agents in the network.
"""

# Built-in/Generic Imports
import sys
import os
from abc import abstractmethod
from typing import List, Optional, Tuple
import warnings
from copy import deepcopy
import time
import random

# Libs
from pprint import pformat
import matplotlib.pyplot as plt

# Suppress FutureWarning messages
warnings.simplefilter(action='ignore', category=FutureWarning)

# ROS2 Imports
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point

# NetworkX

# Local Imports
try:
    from maaf_config.maaf_config import *

    from maaf_msgs.msg import TeamCommStamped, Bid, Allocation

    from maaf_tools.datastructures.task.TaskLog import TaskLog
    from maaf_tools.datastructures.task.Task import Task

    from maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.datastructures.agent.AgentState import AgentState
    from maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.datastructures.organisation.Organisation import Organisation
    from maaf_tools.datastructures.environment.Environment import Environment

    from maaf_tools.tools import *
    from maaf_tools.Singleton import SLogger

    # > CBAA
    from maaf_allocation_node.allocation_logics.CBAA.bidding_logics.anticipated_action_task_interceding_agent import anticipated_action_task_interceding_agent
    from maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid

    # > CBBA
    from maaf_allocation_node.allocation_logics.CBBA.bidding_logics.interceding_skill_based_bid_amplifier import interceding_skill_based_bid_amplifier
    from maaf_allocation_node.allocation_logics.CBBA.bidding_logics.GraphWeightedManhattanDistanceBundleBid import GraphWeightedManhattanDistanceBundleBid

except ModuleNotFoundError as e:
    warnings.warn(f"Module not found: {e}. Please check your imports.")
    from maaf_config.maaf_config.maaf_config import *

    from maaf_msgs.msg import TeamCommStamped, Bid, Allocation

    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog
    from maaf_tools.maaf_tools.datastructures.task.Task import Task

    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.AgentState import AgentState
    from maaf_tools.maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.maaf_tools.datastructures.organisation.Organisation import Organisation
    from maaf_tools.maaf_tools.datastructures.environment.Environment import Environment

    from maaf_tools.maaf_tools.tools import *
    from maaf_tools.maaf_tools.Singleton import SLogger

    # > CBAA
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBAA.bidding_logics.anticipated_action_task_interceding_agent import anticipated_action_task_interceding_agent
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBAA.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid

    # > CBBA
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBBA.bidding_logics.interceding_skill_based_bid_amplifier import interceding_skill_based_bid_amplifier
    from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBBA.bidding_logics.GraphWeightedManhattanDistanceBundleBid import graph_weighted_manhattan_distance_bundle_bid

##################################################################################################################


class MAAFAgent(Node):
    def __init__(self, node_class: str, id_override: str = None):
        """
        maaf agent class
        """
        # ----------------------------------- Node Configuration
        super().__init__(node_name=node_class)

        logger = SLogger()
        logger.logger = self.get_logger()

        # ---- Agent properties
        # > declare all parameters at once
        if id_override is None:
            self.declare_parameters(namespace="", parameters=[("id", "")])
            self.id = self.get_parameter("id").get_parameter_value().string_value

            # -> Check that id is not None
            if self.id == "":
                raise ValueError("Agent id parameter not set (must be set in a launch file or in the constructor)")

        else:
            self.id = id_override

        # ---- Multi-hop behavior
        self.rebroadcast_received_msgs = False

        # ---- Environment
        self.environment = None

        # ---- Fleet and task log
        self.fleet = None
        self.organisation = None
        self.__setup_fleet_and_tasklog()

        # ---- Listeners
        self.__env_update_listeners = []
        self.__pose_update_listeners = []

        # -> Connect listeners
        # self.fleet.add_on_edit_list_listener()
        # self.tasklog.add_on_edit_list_listener()

        # ---- Node connections
        self.__setup_node_base_pubs_subs()
        self._setup_node_additional_pubs_subs()

        # -> Initialise previous state hash
        self.prev_allocation_state_hash_dict = None

    # ============================================================== INIT
    def __setup_fleet_and_tasklog(self) -> None:
        # ---- Fleet data
        """
        Fleet dict of size N_a
        Agents ids are obtained using the self.id_card property
        """

        # TODO: Remove once organisation is loaded from file, failsafe for CoHoMa ----------------
        try:
            from maaf_tools.datastructures.organisation.MOISEPlus.MoiseModel import MoiseModel
            from maaf_tools.datastructures.organisation.RoleAllocation import RoleAllocation

        except:
            from maaf_tools.maaf_tools.datastructures.organisation.MOISEPlus.MoiseModel import MoiseModel
            from maaf_tools.maaf_tools.datastructures.organisation.RoleAllocation import RoleAllocation

        # -> Load Moise+ model components
        with open(organisation_folder_path + "/moise_deontic_specification.json", "r") as file:
            deontic_specification = json.load(file)

        with open(organisation_folder_path + "/moise_functional_specification.json", "r") as file:
            functional_specification = json.load(file)

        with open(organisation_folder_path + "/moise_structural_specification.json", "r") as file:
            structural_specification = json.load(file)

        model = MoiseModel(
            deontic_specification=deontic_specification,
            functional_specification=functional_specification,
            structural_specification=structural_specification
        )

        # -> Load team compo json
        with open(organisation_folder_path + "/fleet_role_allocation.json", "r") as file:
            role_allocation = json.load(file)

        role_allocation = RoleAllocation(role_allocation=role_allocation)

        # -> Construct fleet
        # Load agent classes json and fleet agents json
        with open(organisation_folder_path + "/fleet_agent_classes.json", "r") as file:
            agent_classes = json.load(file)

        with open(organisation_folder_path + "/fleet_agents.json", "r") as file:
            fleet_agents = json.load(file)

        # Construct fleet instance
        fleet = Fleet.from_config_files(
            fleet_agents=fleet_agents,
            agent_classes=agent_classes
        )

        with open(organisation_folder_path + "/fleet_allocation_specification.json", "r") as file:
            allocation_specification = json.load(file)

        self.organisation = Organisation(
            fleet=fleet,
            moise_model=model,
            role_allocation=role_allocation,
            allocation_specification=allocation_specification
        )

        # TODO: Uncomment once organisation is loaded from file, failsafe for CoHoMa ----------------
        # -> Load organisation
        # self.organisation = Organisation.load_from_file(
        #     filename=organisation_file_path, # TODO: Monte Carlo - Change to ROS parameters
        #     partial=False
        # )

        # -> Extract fleet object
        self.fleet = self.organisation.fleet.clone()

        # -> Remove fleet from organisation to avoid data duplication
        self.organisation.fleet = None  # TODO: Rethink fleet/organisation management

        # -> Check if self is in the fleet
        if self.id not in self.fleet.ids:
            raise ValueError(f"Agent {self.id} not in fleet. Please check the organisation file."
                             f"\n> Fleet contains: {self.fleet.ids}")

        # -> Set get_timestamp method
        def get_timestamp():
            return self.current_timestamp

        self.agent.state.set_get_timestamp(get_timestamp)

        # -> Add timestamp sync to the self agent
        def timestamp_sync(agent: Agent):
            # > Call timestamp to force refresh it
            agent.state.timestamp

        self.agent.add_pre_asdict_subscriber(timestamp_sync)

        # -> Set self state as online
        self.agent.set_online_state(online=True)

        # ---- Task log
        """
        Tasks dict of size N_t
        Tasks are created using the self.create_task method
        """
        # -> Create task log object
        self.tasklog = TaskLog()

    def __setup_node_base_pubs_subs(self) -> None:
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

    def _setup_node_additional_pubs_subs(self) -> None:
        """
        Setup additional node publishers and subscribers
        Optional method to be implemented in child classes
        !!! Called automatically by the parent class constructor, does not need to be called manually !!!
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

    def final_init(self) -> None:
        """
        Final initialisation method to be called at the end of the initialisation process.
        Used to synchronise agents.
        IMPORTANT: AUTOMATICALLY CALLED AT THE END OF THE INITIALISATION PROCESS, does not need to be called manually.
        """

        # -> Initial publish to announce the agent to the fleet and share initial state
        time.sleep(2)
        self.publish_allocation_state_msg(if_state_change=False)

        self.get_logger().info(f"\n>>>>>>>>>>>>>>>>>>>>>>>>>>> Agent {self.id} initialised: " +
                               #f"\n     Hierarchy level: {self.hierarchy_level}" +
                               f"\n     Skillset: {self.agent.skillset}" +
                               f"\n     Environment: {self.environment}"
                               )

    def __init_subclass__(cls, **kwargs):
        """
        Logic for calling final_init at the end of the initialisation process of all the child classes

        :param kwargs:
        :return:
        """
        super().__init_subclass__(**kwargs)
        original_init = cls.__init__

        def wrapped_init(self, *args, **kwargs):
            original_init(self, *args, **kwargs)
            # Call final_init only if this is the instance's exact class
            if type(self) is cls:
                self.final_init()

        cls.__init__ = wrapped_init

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

        :param environment: bool, whether to include environment state
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
                    state["environment"] = self.environment.to_json()
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

        if local_allocation_state and self.local_allocation_state is not None:
            if not serialised:
                state = {**state, **self.local_allocation_state}
            else:
                serialised_state = {}

                for key, value in self.local_allocation_state.items():
                    if isinstance(value, pd.DataFrame):
                        serialised_state[key] = value.to_dict(orient="split")
                    else:
                        serialised_state[key] = value.asdict()

                state = {**state, **serialised_state}

        if shared_allocation_state and self.shared_allocation_state is not None:
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
            "fleet": self.fleet,
            "organisation": self.organisation
        }

    @property
    def local_allocation_state(self) -> dict or None:
        """
        Local allocation state at current time step (not serialised)
        !!!! All entries must be dataframes !!!!

        :return: dict
        """
        return None

    @property
    def shared_allocation_state(self):
        """
        Shared allocation state at current time step (not serialised)
        !!!! All entries must be dataframes !!!!

        :return: dict
        """
        return None

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

    def __env_callback(self, msg: TeamCommStamped) -> None:
        if self.environment is None:

            self.environment = Environment.from_json(json_str=msg.memo)

            self.get_logger().info(f"         > {self}: Received environment update: Computing all shortest paths missing...")
            self.environment.compute_all_shortest_paths(recompute_all_shortest_paths=False)
            self.get_logger().info(f"         > {self}: Received environment update: Done computing all shortest paths")

            self.get_logger().info(f"         < Received environment update")

            # TODO: Remove, only for testing
            # -> Select a random node in the environment and set as agent position
            random_node = random.choice(list(self.environment.graph.nodes()))
            random_node_pos = self.environment.graph.nodes[random_node]["pos"]
            self.agent.state.x = random_node_pos[0]
            self.agent.state.y = random_node_pos[1]
            self.agent.state.z = random_node_pos[2]
            self.get_logger().info(f">>>>>>>>>>>>>>>>>>>>> {self.agent.state}")

            # -> Display the graph
            #self.environment.plot_env(fleet=self.fleet)

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

        # -> Call pose update listeners
        self.call_on_pose_update_listeners()

    def __fleet_msg_update_timer_callback(self) -> None:
        """
        Callback for the fleet message update timer
        """
        # -> Publish allocation state to the fleet
        self.publish_allocation_state_msg()

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

    # ---------------- Processes
    # >>>> Base
    def check_if_handle_message(self, msg) -> bool:
        """
        Check if the message should be handled by the agent or ignored
        Ignore if:
        - the message is from the agent itself
        - the message is not for the agent
        - the message is not for all agents

        :param msg: msg to evaluate
        :return: bool
        """
        # -> Ignore self messages
        if msg.source == self.id:
            return False

        # -> Check if the message is for the agent
        msg_target = msg.target

        # -> If the message is not for the agent...
        if msg_target != self.id and msg_target != "all":
            # -> Check if the agent should rebroadcast the message
            # msg, rebroadcast = self.rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub) # TODO: Review for message rebroadcasting
            return False

        # -> If the message is for all agents or self, handle it
        return True

    def publish_allocation_state_msg(self, if_state_change: bool = True) -> None:
        """
        Publish allocation state to the fleet as TeamCommStamped.msg message

        :param if_state_change: bool, whether to publish the message only if the state has changed
        """

        # -> If publish is dependent on state change
        if if_state_change:
            # -> If state has not changed, do not publish
            if not self.allocation_state_change:
                return

        # -> Update previous state hash
        self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

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
        Publish goal to the robot's goal topic for execution as TeamCommStamped message.

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
            # # TODO: Remove once pathfinding handled by controller -----
            # # -> Update path to start task to ensure agent path starts from current position
            # if self.agent.plan:
            #     for source_node, target_node in self.agent.plan.get_node_pairs(agent_id=self.agent.id):
            #         self.update_path(
            #             source_node=source_node,
            #             target_node=target_node,
            #         )
            #
            # # -> Embed paths in agent
            # agent_dict = self.agent.asdict()
            # agent_dict["plan"]["paths"] = self.agent.plan.get_paths(
            #     agent_id=self.agent.id,
            #     tasklog=self.tasklog,
            #     requirement=None,  # TODO: Correct once path requirements are implemented
            #     selection="shortest"  # TODO: Correct once path requirements are implemented
            # )
            #
            # if len(self.agent.plan.task_sequence) == 0:
            #     plan_path = []
            #
            # else:
            #     plan_tasks_locs_sequence = [self.agent.state.pos]
            #     for task_id in self.agent.plan.task_sequence:
            #         task_ = self.tasklog[task_id]
            #
            #         plan_tasks_locs_sequence.append(
            #             [task_.instructions["x"], task_.instructions["y"]]
            #         )
            #
            #     plan_path = self.environment.get_loc_sequence_shortest_path(
            #         loc_sequence=plan_tasks_locs_sequence,
            #         x_lim=None,
            #         y_lim=None,
            #         create_new_node=False,
            #         compute_missing_paths=True
            #     )
            #
            # # -> Remove the current agent position from the path
            # plan_path.pop(0)
            # # TODO: ---------------------------------------------------

            # -> Gather tasks in plan
            tasks = {}

            for task_id in self.agent.plan:
                tasks[task_id] = self.tasklog[task_id].asdict()

            msg.target = self.id
            memo = {
                "agent": self.agent.asdict(),
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

        # ---- Merge received fleet into local one
        fleet_state_change = self.fleet.merge(
            fleet=fleet,
            add_agent_callback=self.add_agent,
            remove_agent_callback=self.remove_agent,
            fleet_state_change_callback=None,
            prioritise_local=False,
            logger=self.get_logger(),
        )

        # ---- Merge received task list into local one
        task_state_change = self.tasklog.merge(
            tasklog=tasklog,
            add_task_callback=self.add_task,
            terminate_task_callback=self.terminate_task,
            tasklog_state_change_callback=None,
            prioritise_local=False,
            logger=self.get_logger()
        )

        return task_state_change, fleet_state_change

    def add_agent(self, agent: Agent) -> None:
        """
        Method called when a new agent is added to the fleet. To use, simply override this method in the child class.

        :param agent: Agent to add
        """
        pass

    def remove_agent(self, agent: Agent) -> None:
        """
        Method called when an agent is removed from the fleet. To use, simply override this method in the child class.

        :param agent: Agent to remove
        """
        pass

    def terminate_task(self, task: Task) -> None:
        """
        Method called when a task is terminated. To use, simply override this method in the child class.

        :param task: Task to terminate
        """
        pass

    def add_task(self, task: Task) -> None:
        """
        Method called when a new task is added to the task log. To use, simply override this method in the child class.

        :param task: Task to add
        """
        pass

    # >>>> Node specific
    # ---------------- tools
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
