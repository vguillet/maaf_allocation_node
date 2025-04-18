
##################################################################################################################

"""
This module contains the ICBAgent allocation node class. It is a child class of the MAAFAgent class and implements the
basic components of consensus-based allocation algorithms. It must be extended by the child classes to implement
the specific allocation algorithm.
"""

# Built-in/Generic Imports
from typing import List, Optional, Tuple
from copy import deepcopy
from abc import ABC, abstractmethod

# ROS2 Imports

# Local Imports
try:
    from maaf_config.maaf_config import *

    from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
    from maaf_allocation_node.maaf_agent import MAAFAgent

    from maaf_tools.datastructures.task.TaskLog import TaskLog
    from maaf_tools.datastructures.task.Task import Task

    from maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.tools import *

except ModuleNotFoundError:
    from maaf_config.maaf_config.maaf_config import *

    from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
    from maaf_allocation_node.maaf_allocation_node.maaf_agent import MAAFAgent

    from maaf_tools.maaf_tools.datastructures.task.Task import Task
    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.maaf_tools.tools import *

##################################################################################################################

FREE_ALLOCATION = 0
BLACKLISTED = -1
IMPOSED_ALLOCATION = 1
NO_ACTION = None


class ICBAgent(MAAFAgent):
    def __init__(self, node_class: str):
        # ---- Init parent class
        super().__init__(node_class=node_class)

        # ---- Allocation states
        self.__setup_allocation_base_states()
        self._setup_allocation_additional_states()

        # ---------- Recompute
        reset_assignment = True

        # -> Recompute bids on environment change
        if True:    # TODO: Add parameter to enable/disable this in config
            self.add_on_env_update_listener(lambda *args, **kwargs: self.update_allocation(reset_assignment=reset_assignment))

        # -> Recompute bids on state change
        if self.organisation.allocation_specification.get_property(agent_id=self.id, property_name="recompute_bids_on_state_change"):
            self.add_on_pose_update_listener(lambda *args, **kwargs: self.update_allocation(reset_assignment=reset_assignment))

    # ============================================================== INIT
    def _setup_node_additional_pubs_subs(self) -> None:
        """
        Setup additional pubs-subs for the node.
        !!! Called automatically by the parent class constructor
        :return:
        """

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

    @abstractmethod
    def _setup_allocation_additional_states(self) -> None:
        """
        Setup additional method-dependent allocation states for the agents
        Optional method to be implemented in child classes
        !!! Called automatically by the parent class constructor, does not need to be called manually !!!
        """
        raise NotImplementedError

    # ============================================================== PROPERTIES
    @property
    @abstractmethod
    def local_allocation_state(self) -> dict:
        """
        Local allocation state at current time step (not serialised)
        !!!! All entries must be dataframes !!!!

        :return: dict
        """
        pass

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

    # ============================================================== METHODS
    # ---------------- Callbacks
    def _task_msg_subscriber_callback(self, task_msg) -> None:
        """
        Callback for task messages: create new task, add to local tasks and update local states, and select new task

        :param task_msg: TaskMsgStamped message
        """

        # ----- Check if the message should be processed
        if not self.check_if_handle_message(msg=task_msg):
            return

        # ----- Unpack msg
        # -> Unpack msg
        task_dict = loads(task_msg.memo)

        # -> Ensure id is a string
        task_dict["id"] = str(task_dict["id"])

        # -> Create task object
        task = Task.from_dict(task_dict, partial=True)

        if task_msg.meta_action == "pending":
            self.get_logger().info(f"{self.id} * Task {task.id} found (Type: {task.type}) - Pending task count: {len(self.tasklog.ids_pending) + 1}")

        elif task_msg.meta_action == "completed":
            # if task.id in self.tasklog.ids_pending: # Only log if task was pending (avoid double logging)
            self.get_logger().info(f"{self.id} v Task {task.id} completed - Pending task count: {len(self.tasklog.ids_pending) - 1}")

        elif task_msg.meta_action == "cancelled":
            if task.id in self.tasklog.ids_pending: # Only log if task was pending (avoid double logging)
                self.get_logger().info(f"{self.id} x Task {task.id} cancelled - Pending task count: {len(self.tasklog.ids_pending) - 1}")

        # -> Create new task
        if task_msg.meta_action == "pending":
            # -> Pass if task is already in the task log
            if task.id in self.tasklog.ids:
                return

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
        success = tasklog.add_task(task=task)

        # self.get_logger().info(f">>>>>>>>>>>>>>>>>>>>>>>>>> {self.id} - Task {task.id} added to tasklog: {success}")

        # if self.id == "Turtle_4":
        #     self.get_logger().info(f"\n{self.tasklog.pretty_table}")

        task_state_change, fleet_state_change = self.update_situation_awareness(tasklog=tasklog, fleet=None)

        # -> Select task
        self.update_allocation(
            reset_assignment=task_state_change and
            self.organisation.allocation_specification.get_property(agent_id=self.id, property_name="recompute_bids_on_state_change")
        )

        # -> If state has changed, update local states (only publish when necessary)
        self.publish_allocation_state_msg(if_state_change=True)

        #self.environment.plot_env(fleet=self.fleet, tasklog=self.tasklog)

    def _team_msg_subscriber_callback(self, team_msg) -> None:
        """
        Callback for team messages: consensus phase of the CBAA algorithm

        :param team_msg: TeamComm message
        """
        # ----- Check if the message should be processed
        if not self.check_if_handle_message(msg=team_msg):
            return

        # ----- Unpack msg
        # -> Deserialize allocation state
        received_allocation_state = self.deserialise(state=team_msg.memo)

        # ----- Update local situation awareness
        # > Convert received serialised tasklog to tasklog
        received_tasklog = TaskLog.from_dict(received_allocation_state["tasklog"])

        # > Convert received serialised fleet to fleet
        received_fleet = Fleet.from_dict(received_allocation_state["fleet"])

        # > Update situation awareness
        task_state_change, fleet_state_change = self.update_situation_awareness(
            tasklog=received_tasklog,
            fleet=received_fleet
        )

        # ----- Update shared states
        # > Get received agent
        received_agent = self.fleet[team_msg.source]

        #self.get_logger().info(f"Received states: {received_allocation_state.keys()}")

        # > Update the agent in the fleet
        self.update_shared_states(
            agent=received_agent,
            **received_allocation_state
        )

        # ----- Update task
        # -> Update the task in the task list x the agent is assigned to
        for task in received_tasklog:
            # > If task is terminated, skip
            if task.id not in self.tasklog.ids_pending:
                continue

            # > Update task
            self.update_task(
                task=task,
                agent=received_agent,
                **received_allocation_state,
            )

        # ----- Update allocation
        self.update_allocation(
            reset_assignment=task_state_change and
                             self.organisation.allocation_specification.get_property(agent_id=self.id, property_name="recompute_bids_on_state_change"),
            agent=received_agent,
            **received_allocation_state
        )

        # ----- Publish allocation state if state has changed
        # -> If state has changed, update local states (only publish when necessary)
        self.publish_allocation_state_msg(if_state_change=True)

        # else:
        #     # -> Check if the agent should rebroadcast the message
        #     msg, rebroadcast = self.rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub)

    def __bid_subscriber_callback(self, bid_msg: Bid) -> None:
        """
        # TODO: Review and test this callback
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

                                #greater_than_zero_condition=True,

                                # Reset
                                currently_assigned=None,
                                reset=False     # TODO: REVIEW (was True before refactor..?)
                                )

        # -> Update allocation
        self.update_allocation()

        # -> If state has changed, update local states (only publish when necessary)
        self.publish_allocation_state_msg(if_state_change=True)

    def __allocation_subscriber_callback(self, allocation_msg: Allocation) -> None:
        """
        # TODO: Review and test this callback
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

                #greater_than_zero_condition=False,

                # Reset
                currently_assigned=None,
                reset=False     # TODO: REVIEW (was True before refactor..?)
            )

            # -> Update allocation
            self.update_allocation()

            # -> If state has changed, update local states (only publish when necessary)
            self.publish_allocation_state_msg(if_state_change=True)

    # ---------------- Processes

    @abstractmethod
    def add_agent(self, agent: Agent) -> None:
        """
        Add new agent to local fleet and extend local states with new columns for new agent

        :param agent: Agent to add
        """
        raise NotImplementedError

    @abstractmethod
    def remove_agent(self, agent: Agent) -> None:
        """
        Remove agent from local fleet and all relevant allocation lists and matrices

        :param agent: Agent to remove
        """
        raise NotImplementedError

    @abstractmethod
    def terminate_task(self, task: Task) -> None:
        """
        Flag task as terminated in task log and remove all relevant allocation lists and matrices rows

        :param task: Task to terminate
        """
        raise NotImplementedError

    @abstractmethod
    def add_task(self, task: Task) -> None:
        """
        Add new task to local tasks and extend local states with new rows for new task

        :param task: Task to add
        """
        raise NotImplementedError

    @abstractmethod
    def update_shared_states(self,
                             agent: Agent,
                             *args,
                             **kwargs
                             ):
        """
        Update local states with received states from the fleet

        Parameters are algorithm-specific
        """
        raise NotImplementedError

    @abstractmethod
    def update_task(self,
                    task: Task,
                    agent: Agent,
                    *args,
                    **kwargs
                    ) -> None:
        """
        Update current task based on received winning bids and updated allocation intercession from the fleet

        Parameters are algorithm-specific
        """
        raise NotImplementedError

    @abstractmethod
    def update_allocation(self,
                          reset_assignment: bool = False,
                          agent: Agent = None,
                          *args, **kwargs
                          ) -> None:
        """
        Select a task to bid for based on the current state
        1. Merge local states with shared states and update local states if necessary
        2. If no task is assigned to self, select a task

        :param reset_assignment: Flag to reset the current assignment
        :param agent: Agent object
        """
        raise NotImplementedError

    def reset_allocation(self, reset_assignment: bool = False) -> None:
        """
        Reset the allocation state of the agent. This is a placeholder method that can be overridden by subclasses.

        :param reset_assignment: Flag to reset the current assignment
        """

        if self.agent.plan.current_task_id is not None:
            # -> Drop current task
            self.get_logger().info(f"Agent {self.id}: Resetting current task assignment")
            self._drop_task(task_id=self.agent.plan.current_task_id, reset=True, forward=True)

        # -> Update allocation
        self.update_allocation(reset_assignment=reset_assignment)  # TODO: Review reset_assignment for efficiency

        # -> Publish allocation state if state has changed
        self.publish_allocation_state_msg(if_state_change=True)

    # ---------------- tools
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
    def _drop_task(self, task_id: str, reset: bool, traceback: str, logger: bool, *args, **kwargs):
        """
        Drop a task from the task list x or y. If reset is True, the task is removed from the task list x, otherwise it is removed from the task list y.

        :param task_id: Task id
        :param reset: Flag to reset task value to zero if source priority is higher
        :param traceback: Reason for dropping the task
        :param logger: Flag to log the task drop
        """
        raise NotImplementedError
