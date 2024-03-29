
##################################################################################################################

"""
This module contains the MAAF allocation node class, which is a ROS2 node that implements the CBAA algorithm for task allocation.
"""

# Built-in/Generic Imports
import os
import time
from random import randint
from json import dumps, loads
from typing import List, Optional, Tuple
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
from networkx import astar_path, shortest_path

# Local Imports
from orchestra_config.sim_config import *
from maaf_allocation_node.maaf_agent import MAAFAgent

from maaf_tools.datastructures.task.Task import Task
from maaf_tools.datastructures.task.TaskLog import TaskLog

from maaf_tools.datastructures.agent.Agent import Agent
from maaf_tools.datastructures.agent.Fleet import Fleet
from maaf_tools.datastructures.agent.Plan import Plan

from maaf_tools.tools import *

from maaf_allocation_node.bidding_logics.graph_weighted_manhattan_distance_bid import graph_weighted_manhattan_distance_bid
from maaf_allocation_node.bidding_logics.anticipated_action_task_interceding_agent import anticipated_action_task_interceding_agent

##################################################################################################################


class ICBAANode(MAAFAgent):
    def __init__(self):

        # ---- Init parent class
        MAAFAgent.__init__(
            self,
            node_name="CBAAwI_allocation_node",
            skillset=None
        )

        # -----------------------------------  Agent allocation states
        # -> Setup additional CBAA-specific allocation states
        self.__setup_allocation_additional_states()

        # -> Initialise previous state hash
        self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

        # -----------------------------------  Confirm initialisation
        # -> Initial publish to announce the agent to the fleet and share initial state
        time.sleep(2)
        self.publish_allocation_state_msg()

        self.publish_goal_msg(meta_action="empty")

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
            index=self.tasklog.ids_pending,
            columns=["task_list_x"]
        )

    # ============================================================== PROPERTIES
    # ---------------- Self state
    @property
    def local_allocation_state(self) -> dict:
        """
        Local allocation state at current time step (serialised)

        :return: dict
        """
        return {
            "task_list_x": self.task_list_x,
            "local_bids_c": self.local_bids_c,
            "local_allocations_d": self.local_allocations_d
        }

    # ============================================================== METHODS
    # ---------------- Callbacks
    # >>>> Base
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
        task_dict = loads(task_msg.memo)

        # -> Ensure id is a string
        task_dict["id"] = str(task_dict["id"])

        # -> Create task object
        task = Task.from_dict(task_dict, partial=True)

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
        tasklog.add_task(task=task)

        task_state_change, fleet_state_change = self.__update_situation_awareness(tasklog=tasklog, fleet=None)

        # -> Select task
        self.update_allocation(reset_assignment=task_state_change and self.scenario.recompute_bids_on_state_change)   # TODO: Cleanup

        # -> Update previous state hash
        self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

        # > Publish goal msg
        self.publish_goal_msg(meta_action="update")

        # -> If state has changed, update local states (only publish when necessary)
        self.publish_allocation_state_msg()

        if task_msg.meta_action == "pending":
            self.get_logger().info(f"{self.id} * Found new task: {task.id} (Type: {task.type}) - Pending task count: {len(self.tasklog.ids_pending)}")

        elif task_msg.meta_action == "completed":
            self.get_logger().info(f"{self.id} v Task {task.id} completed - Pending task count: {len(self.tasklog.ids_pending)}")

        elif task_msg.meta_action == "cancelled":
            self.get_logger().info(f"{self.id} x Task {task.id} cancelled - Pending task count: {len(self.tasklog.ids_pending)}")

    def team_msg_subscriber_callback(self, team_msg) -> None:
        """
        Callback for team messages: consensus phase of the CBAA algorithm

        :param team_msg: TeamComm message
        """
        # ---- Unpack msg
        # -> Ignore self messages
        if team_msg.source == self.id:
            return

        # -> Check if the message is for the agent
        msg_target = team_msg.target

        # # -> If the message is not for the agent...
        if msg_target != self.id and msg_target != "all":
            # -> Check if the agent should rebroadcast the message
            # msg, rebroadcast = self.rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub)
            return

        # -> Deserialise allocation state
        received_allocation_state = self.deserialise(state=team_msg.memo)

        # -> Update local situation awareness
        # > Convert received serialised tasklog to tasklog
        received_task_log = TaskLog.from_dict(received_allocation_state["tasklog"])

        # > Convert received serialised fleet to fleet
        received_fleet = Fleet.from_dict(received_allocation_state["fleet"])

        task_state_change, fleet_state_change = self.__update_situation_awareness(
            tasklog=received_task_log,
            fleet=received_fleet
        )

        # -> Update shared states
        self.__update_shared_states(
            received_shared_bids_b=received_allocation_state["shared_bids_b"],
            received_shared_bids_priority_beta=received_allocation_state["shared_bids_priority_beta"],
            received_shared_allocations_a=received_allocation_state["shared_allocations_a"],
            received_shared_allocations_priority_alpha=received_allocation_state["shared_allocations_priority_alpha"]
        )

        # -> Update the task in the task list x the agent is assigned to
        for task in received_task_log:
            if task.id not in self.tasklog.ids_pending:
                continue

            task_id = task.id

            if self.task_list_x.loc[task_id, "task_list_x"] == 1:
                self.__update_task(
                    received_winning_bids_y=received_allocation_state["winning_bids_y"],
                    task_id=task_id
                )

        # -> Select new task
        self.update_allocation(reset_assignment=task_state_change and self.scenario.recompute_bids_on_state_change) # TODO: Cleanup

        # > Publish goal msg
        self.publish_goal_msg(meta_action="update")

        # -> If state has changed, update local states (only publish when necessary)
        self.check_publish_state_change()

        # else:
        #     # -> Check if the agent should rebroadcast the message
        #     msg, rebroadcast = self.rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub)

    # >>>> CBAA
    def __update_situation_awareness(self,
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

        def add_agent(agent: Agent) -> None:
            """
            Add new agent to local fleet and extend local states with new columns for new agent
            """

            # -> Add a column to all relevant allocation lists and matrices with new agent
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            # > Remove task_list_x from the state (not needed for this operation)
            state.pop("task_list_x")

            # > Remove winning_bids_y from the state (not needed for this operation)
            state.pop("winning_bids_y")

            for matrix in state.values():
                matrix[agent.id] = pd.Series(np.zeros(self.Task_count_N_t), index=self.tasklog.ids_pending)

        def remove_agent(agent: Agent) -> None:
            """
            Remove agent from local fleet and all relevant allocation lists and matrices
            """

            # TODO: Figure out how to deal with removing bids from the winning bid matrix (full reset vs bid owner tracking). For now, reset winning bids matrix (full reset)
            self.winning_bids_y = pd.Series(np.zeros(self.Task_count_N_t), index=self.tasklog.ids_pending)

            # -> Remove agent from all allocation lists and matrices
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            for matrix in state.values():
                try:
                    matrix.drop(columns=agent.id, inplace=True)
                except KeyError:
                    pass

        def add_task(task: Task) -> None:
            """
            Add new task to local tasks and extend local states with new rows for new task
            """
            # -> Add a row to all allocation lists and matrices with new task
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            # > Remove task_list_x from the state (operation performed separately)
            state.pop("task_list_x")
            self.task_list_x.loc[task.id] = 0

            # > Remove winning_bids_y from the state (operation performed separately)
            state.pop("winning_bids_y")
            self.winning_bids_y.loc[task.id] = 0

            for matrix in state.values():
                matrix.loc[task.id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)

        def terminate_task(task: Task) -> None:
            """
            Flag task as terminated in task log and remove all relevant allocation lists and matrices rows
            """

            # -> Cancel goal if task is assigned to self
            if self.task_list_x.loc[task.id, "task_list_x"] == 1:
                # -> Remove task from plan
                self.agent.remove_task_from_plan(
                    tasklog=self.tasklog,
                    task=task,
                    # logger=self.get_logger()
                )

            # -> Remove task from all allocation lists and matrices
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            for matrix in state.values():
                try:
                    matrix.drop(index=task.id, inplace=True)
                except KeyError:
                    pass

        # ---- Merge received fleet into local one
        fleet_state_change = self.fleet.merge(
            fleet=fleet,
            add_agent_callback=add_agent,
            remove_agent_callback=remove_agent,
            fleet_state_change_callback=None,
            prioritise_local=False,
            logger=self.get_logger()
        )

        # ---- Merge received task list into local one
        task_state_change = self.tasklog.merge(
            tasklog=tasklog,
            add_task_callback=add_task,
            terminate_task_callback=terminate_task,
            tasklog_state_change_callback=self.compute_bids,
            prioritise_local=False,
            logger=self.get_logger()
        )

        return task_state_change, fleet_state_change

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

        received_tasks_ids = list(received_shared_bids_b.index)
        received_agent_ids = list(received_shared_bids_b.columns)

        # -> For each task ...
        for task_id in received_tasks_ids:
            # -> If the task has been terminated, skip
            if task_id not in self.tasklog.ids_pending:
                continue

            # -> for each agent ...
            for agent_id in received_agent_ids:
                #
                # if agent_id not in self.fleet.ids_active:
                #     continue

                # -> Priority merge with reset received current bids b into local current bids b
                # > Determine correct matrix values
                tasks_value_x_ij_updated, shared_bids_b_ij_updated, shared_bids_priority_beta_ij_updated = (
                    self.CBAA_priority_merge(
                        # Logging
                        task_id=task_id,
                        agent_id=agent_id,

                        # Merging
                        matrix_updated_ij=self.shared_bids_b.loc[task_id, agent_id],
                        matrix_source_ij=received_shared_bids_b.loc[task_id, agent_id],
                        priority_updated_ij=self.shared_bids_priority_beta.loc[task_id, agent_id],
                        priority_source_ij=received_shared_bids_priority_beta.loc[task_id, agent_id],

                        # Reset
                        tasks_value_x_ij=self.task_list_x.loc[task_id, "task_list_x"],
                        reset=True
                    )
                )

                # > Update local states
                self.shared_bids_b.loc[task_id, agent_id] = shared_bids_b_ij_updated
                self.shared_bids_priority_beta.loc[task_id, agent_id] = shared_bids_priority_beta_ij_updated
                self.task_list_x.loc[task_id, "task_list_x"] = tasks_value_x_ij_updated

                # -> Priority merge received current allocations a into local current allocations a
                # > Determine correct matrix values
                tasks_value_x_ij_updated, shared_allocations_a_ij_updated, shared_allocations_priority_alpha_ij_updated = (
                    self.CBAA_priority_merge(
                        # Logging
                        task_id=task_id,
                        agent_id=agent_id,

                        # Merging
                        matrix_updated_ij=self.shared_allocations_a.loc[task_id, agent_id],
                        matrix_source_ij=received_shared_allocations_a.loc[task_id, agent_id],
                        priority_updated_ij=self.shared_allocations_priority_alpha.loc[task_id, agent_id],
                        priority_source_ij=received_shared_allocations_priority_alpha.loc[task_id, agent_id],

                        # Reset
                        tasks_value_x_ij=self.task_list_x.loc[task_id, "task_list_x"],
                        reset=True
                    )
                )

                # > Update local states
                self.shared_allocations_a.loc[task_id, agent_id] = shared_allocations_a_ij_updated
                self.shared_allocations_priority_alpha.loc[task_id, agent_id] = shared_allocations_priority_alpha_ij_updated
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
            self.shared_allocations_a.loc[task_id, self.shared_allocations_a.loc[task_id] == 1].index.to_list()
        )

        # -> If there are imposed allocations, find the imposed allocation with the highest priority
        if len(agents_with_imposed_allocations) > 0:
            # -> Find agent with the highest priority
            winning_agent = self.shared_allocations_priority_alpha.loc[task_id, agents_with_imposed_allocations].idxmax()

        else:
            # -> Compare received winning bids with local winning bids
            winning_agent = self.id if self.winning_bids_y.loc[task_id, "winning_bids_y"] >= received_winning_bids_y.loc[task_id, "winning_bids_y"] else None

            # TODO: Fix self.winning_bids_y.loc[task_id] > received_winning_bids_y.loc[task_id] scenario. For now it is assumed that two agents cannot have the same bid for the same task

        # -> Update task list x
        if winning_agent is not self.id:
            # -> If the winning agent is not the agent, remove the task from the task list
            self.task_list_x.loc[task_id] = 0

            # -> Remove task from plan
            self.agent.remove_task_from_plan(
                tasklog=self.tasklog,
                task=task_id,
                logger=self.get_logger()
            )

    # ---------------- Processes
    # >>>> CBAA
    def compute_bids(self) -> None:
        """
        Conditionally compute bids for all tasks based on the current state and the state the current
        bids were computed in. If the current state is different from the state the current bids were
        computed in, recompute the bids.
        """
        # ... for each task
        for task in self.tasklog.tasks_pending:
            # -> Check bid reference state
            compute_bids = False

            # > If bids were never computed for the task, compute bids
            if "bid(s)_reference_state" not in task.local.keys():
                compute_bids = True

            # > If bids are outdated and recompute on state change is enabled, compute bids # TODO: Review
            elif task.local["bid(s)_reference_state"] != self.agent.state and self.scenario.recompute_bids_on_state_change:
                compute_bids = True

            # -> Re-compute bids if necessary
            if compute_bids:
                # -> List agents to compute bids for
                # TODO: Clean up
                if self.bid_evaluation_function is anticipated_action_task_interceding_agent:
                    agent_lst = self.fleet.agents_active
                else:
                    agent_lst = [self.agent]

                # -> Compute bids
                self.bid(task=task, agent_lst=agent_lst)

            # TODO: Cleanup
            # elif RUN_MODE == SIM: # TODO: Enable once comm sim setup
            # -> Recompute path to task for self if necessary
            elif task.local["bid(s)_reference_state"] != self.agent.state:
                # -> Agent node
                agent_node = (self.agent.state.x, self.agent.state.y)

                # -> Task node
                task_node = (task.instructions["x"], task.instructions["y"])

                # -> Find the Manhattan distance between the agent and the task
                path = shortest_path(self.env["graph"], agent_node, task_node)
                # path = astar_path(environment["graph"], agent_node, task_node, weight="weight")

                # > Store path to agent task log
                self.tasklog.add_path(
                    source_node="agent",
                    target_node=task.id,
                    path={
                        "id": f"{self.id}_{task.id}",
                        "path": path,
                        "requirements": ["ground"]
                    },
                    two_way=False,
                    selection="latest"
                )

        # -> Update plan (retrieve new path)
        self.agent.update_plan(
            tasklog=self.tasklog
        )

    def update_allocation(self, reset_assignment: bool = False) -> None:
        """
        Select a task to bid for based on the current state
        1. Merge local states with shared states and update local states if necessary
        2. If no task is assigned to self, select a task

        :param reset_assignment: Flag to reset the current assignment
        """
        # ---- Merge local states with shared states
        # -> If own states have changed, update local states (optimisation to avoid unnecessary updates)
        if self.allocation_state_change:
            # > For each task ...
            for task_id in self.tasklog.ids_pending:
                # > For each agent ...
                for agent_id in self.fleet.ids_active:
                    # -> Priority merge local bids c into current bids b
                    # > Determine correct matrix values
                    _, shared_bids_b_ij_updated, shared_bids_priority_beta_ij_updated = self.CBAA_priority_merge(
                        # Logging
                        task_id=task_id,
                        agent_id=agent_id,

                        # Merging
                        matrix_updated_ij=self.shared_bids_b.loc[task_id, agent_id],
                        matrix_source_ij=self.local_bids_c.loc[task_id, agent_id],
                        priority_updated_ij=self.shared_bids_priority_beta.loc[task_id, agent_id],
                        priority_source_ij=self.hierarchy_level,

                        # Reset
                        tasks_value_x_ij=self.task_list_x.loc[task_id, "task_list_x"],
                        reset=False
                    )

                    # > Update local states
                    self.shared_bids_b.loc[task_id, agent_id] = shared_bids_b_ij_updated
                    self.shared_bids_priority_beta.loc[task_id, agent_id] = shared_bids_priority_beta_ij_updated

                    # -> Merge local intercessions into allocation intercession
                    if self.hierarchy_level > self.shared_allocations_priority_alpha.loc[task_id, agent_id]:
                        allocation_state = self.action_to_allocation_state(
                            action=self.local_allocations_d.loc[task_id, agent_id]
                        )

                        if allocation_state is not None:
                            self.shared_allocations_a.loc[task_id, agent_id] = allocation_state

        # ---- Select task
        # -> If reset assignment, reset the task list
        if reset_assignment:
            # -> Unassign task if currently assigned
            if self.task_list_x["task_list_x"].sum() != 0:
                # -> Find currently assigned task
                task_id = self.task_list_x[self.task_list_x["task_list_x"] == 1].index[0]

                # -> Reset allocation states
                self.task_list_x.loc[task_id, "task_list_x"] = 0

                # -> Reset winning bids
                self.winning_bids_y.loc[task_id, "winning_bids_y"] = 0

                # -> Remove task from plan
                self.agent.remove_task_from_plan(
                    tasklog=self.tasklog,
                    task=task_id,
                    logger=self.get_logger()
                )

        # -> If no task is assigned to self, select a task
        if self.task_list_x["task_list_x"].sum() == 0:
            # -> Create list of valid tasks (pandas series of task ids initialised as 0)
            valid_tasks_list_h = pd.DataFrame(
                np.zeros(self.Task_count_N_t),
                index=self.tasklog.ids_pending,
                columns=["valid_tasks_list_h"]
            )

            # > For each task ...
            for task_id in self.tasklog.ids_pending:
                # -> Create set of agents with imposed allocations
                agents_with_imposed_allocations = list(set(self.shared_allocations_a.loc[task_id, self.shared_allocations_a.loc[task_id] == 1].index))

                # -> If there are imposed allocations, check if self is imposed allocation with the highest priority
                if len(agents_with_imposed_allocations) > 0:
                    # -> Find agent with the highest priority
                    winning_agent = self.shared_allocations_priority_alpha.loc[task_id, agents_with_imposed_allocations].idxmax()

                    # TODO: Adjust to handle conflicting imposed allocations. For now assumed that allocation intercession is conflict-free

                    # -> If self is the winning agent, add the task to the task list
                    if winning_agent == self.id:
                        valid_tasks_list_h[task_id, "valid_tasks_list_h"] = 1
                else:
                    # -> If there are no imposed allocations, check if self has the highest bid and is not blacklisted
                    valid_task = int(self.shared_allocations_a.loc[task_id, self.id] != -1 and self.shared_bids_b.loc[task_id, self.id] > self.winning_bids_y.loc[task_id, "winning_bids_y"])

                    valid_tasks_list_h.loc[task_id, "valid_tasks_list_h"] = valid_task

            # -> If there are valid tasks, select the task with the highest bid
            if valid_tasks_list_h["valid_tasks_list_h"].sum() > 0:
                # -> Select task with the highest bid
                # > Get index of all the tasks for which the valid_tasks_list_h is 1
                valid_tasks = valid_tasks_list_h[valid_tasks_list_h["valid_tasks_list_h"] == 1].index.to_list()

                # > Get valid task with largest bid
                selected_task_id = self.shared_bids_b.loc[valid_tasks, self.id].idxmax()

                # -> Add task to the task list
                self.task_list_x.loc[selected_task_id, "task_list_x"] = 1

                # -> Update winning bids
                self.winning_bids_y.loc[selected_task_id, "winning_bids_y"] = self.shared_bids_b.loc[selected_task_id, self.id]

                # if RUN_MODE == SIM: # TODO: Enable once comm sim setup

                # -> Assign goal
                self.agent.add_task_to_plan(
                    tasklog=self.tasklog,
                    task=selected_task_id,
                    logger=self.get_logger()
                )

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
            pprint(self.tasklog.asdict())
            pprint(self.fleet.asdict())

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
            print(self.shared_bids_b)

            # print("\nCurrent bids priority beta:")
            # print(self.shared_bids_priority_beta)
            #
            # print("\n------------")
            # print("Current allocations a:")
            # print(self.shared_allocations_a)
            #
            # print("\nCurrent allocations priority alpha:")
            # print(self.shared_allocations_priority_alpha)

        print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")

    # >>>> CBAA
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
        ) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """ 
        Wrapper for CBAA_priority_merge to be used by the agent class
        
        :param task_id: Task id
        :param agent_id: Agent id
        :param matrix_updated: Updated matrix
        :param matrix_updated_ij: Updated matrix value
        :param matrix_priority_updated: Updated priority value
        :param priority_updated_ij: Updated priority value
        :param matrix_source: Source matrix
        :param matrix_source_ij: Source matrix value
        :param matrix_priority_source: Source priority value
        :param priority_source_ij: Source priority value
        :param reset: Flag to reset task value to zero if source priority is higher
        
        :return: Updated matrix, updated priority
        """
        
        # -> Priority merge received bid into current bids b
        tasks_value_x_ij, matrix_updated_ij, priority_updated_ij = self.CBAA_priority_merge(
                # Logging
                task_id=task_id,
                agent_id=agent_id,

                # Merging
                matrix_updated_ij=matrix_updated_ij,
                priority_updated_ij=priority_updated_ij,
                
                matrix_source_ij=matrix_source_ij,
                priority_source_ij=priority_source_ij,

                # Reset
                tasks_value_x_ij=self.task_list_x.loc[task_id, "task_list_x"],
                reset=True
            )
        
        # -> Update local x state
        self.task_list_x.loc[task_id, "task_list_x"] = tasks_value_x_ij
        
        # -> Update local states
        matrix_updated.loc[task_id, agent_id] = matrix_updated_ij
        matrix_priority_updated.loc[task_id, agent_id] = priority_updated_ij
        
        return matrix_updated, matrix_priority_updated
        
    def CBAA_priority_merge(
            self,
            task_id: str,
            agent_id: str,
            
            matrix_updated_ij: float,
            priority_updated_ij: float,

            matrix_source_ij: float,            
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
        if priority_source_ij > priority_updated_ij and matrix_source_ij > 0:

            # -> Update matrix value with source matrix value
            matrix_updated_ij = matrix_source_ij

            # -> Update priority value with source priority value
            priority_updated_ij = priority_source_ij

            # -> Reset task value to zero to remove allocation
            if reset:
                if tasks_value_x_ij == 1:
                    tasks_value_x_ij = 0

                    # -> Cancel goal
                    self.agent.remove_task_from_plan(
                        tasklog=self.tasklog,
                        task=task_id,
                        logger=self.get_logger()
                    )

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
