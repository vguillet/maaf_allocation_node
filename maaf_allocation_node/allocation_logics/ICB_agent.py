
##################################################################################################################

"""
This module contains the MAAF allocation node class, which is a ROS2 node that implements the CBAA algorithm for task allocation.
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


class ICBAgent(MAAFAgent):
    def __init__(self,
                 node_name: str,
                 id: str = None,
                 name: str = None,
                 skillset: List[str] = None,
                 bid_estimator=None
                 ):
        # ---- Init parent class
        super().__init__(
            node_name=node_name,
            id=id,
            name=name,
            skillset=skillset,
            bid_estimator=bid_estimator
        )

        # ---------- Recompute
        if self.organisation.allocation_specification.get_property(agent_id=self.id, property_name="recompute_bids_on_state_change"):
            def recompute_on_pose_update():
                if self.agent.plan.current_task_id is not None:
                    # -> Drop current task
                    self.get_logger().info(f"Agent {self.id}: Resetting current task assignment")
                    self._drop_task(task_id=self.agent.plan.current_task_id, reset=True, forward=True)

                # -> Update allocation
                self.update_allocation(reset_assignment=True)

                # -> Publish allocation state if state has changed
                self.publish_allocation_state_msg(if_state_change=True)

            self.add_on_pose_update_listener(recompute_on_pose_update)

    # ============================================================== PROPERTIES

    # ============================================================== METHODS
    # ---------------- Callbacks
    def _task_msg_subscriber_callback(self, task_msg) -> None:
        """
        Callback for task messages: create new task, add to local tasks and update local states, and select new task

        :param task_msg: TaskMsgStamped message
        """

        # ----- Check if the message should be processed
        if not self.handle_message(msg=task_msg):
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

    def _team_msg_subscriber_callback(self, team_msg) -> None:
        """
        Callback for team messages: consensus phase of the CBAA algorithm

        :param team_msg: TeamComm message
        """
        # ----- Check if the message should be processed
        if not self.handle_message(msg=team_msg):
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

    # ---------------- Processes
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
