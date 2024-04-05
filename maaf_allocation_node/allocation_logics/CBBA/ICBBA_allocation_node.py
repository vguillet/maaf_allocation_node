
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
import networkx as nx

# Local Imports
from orchestra_config.orchestra_config import *     # KEEP THIS LINE, DO NOT REMOVE
from orchestra_config.sim_config import *

from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
from maaf_allocation_node.maaf_agent import MAAFAgent
from maaf_allocation_node.allocation_logics.ICB_agent import ICBAgent

from maaf_tools.datastructures.task.Task import Task
from maaf_tools.datastructures.task.TaskLog import TaskLog

from maaf_tools.datastructures.agent.Agent import Agent
from maaf_tools.datastructures.agent.Fleet import Fleet
from maaf_tools.datastructures.agent.Plan import Plan

from maaf_tools.tools import *

from maaf_allocation_node.allocation_logics.CBBA.bidding_logics.graph_weigthed_manhattan_distance_bundle_bid import graph_weighted_manhattan_distance_bundle_bid

##################################################################################################################

SHALLOW = 0
DEEP = 1


class ICBBANode(ICBAgent):
    def __init__(self):
        # ---- Init parent class
        ICBAgent.__init__(
            self,
            node_name="CBBAwI_allocation_node",
            skillset=None
        )

        # ----------------------------------- Agent allocation states
        # -> Setup additional CBAA-specific allocation states
        self._setup_allocation_additional_states()

        # -> Initialise previous state hash
        self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

        # ----------------------------------- Confirm initialisation
        # -> Initial publish to announce the agent to the fleet and share initial state
        time.sleep(2)
        self.publish_allocation_state_msg()

        self.publish_goal_msg(meta_action="empty", traceback="Initialisation")

    # ============================================================== PROPERTIES
    def _setup_allocation_additional_states(self) -> None:
        """
        Setup additional method-dependent allocation states for the agents
        """
        # ---- Local state
        """
        Bids depth e of the agent. Used to track the depth of the bids for each agent/task combination
        > SHALLOW (0)
        > DEEP    (1)  

        Matrix is a pandas dataframe of size N_t x N_u, initialized as zero matrix, with task ids as index and agent ids as columns
        """
        self.bids_depth_e = pd.DataFrame(
            np.zeros((self.Task_count_N_t, self.Agent_count_N_u)),
            index=self.tasklog.ids_pending,
            columns=self.fleet.ids_active,
        )

        """
        Winning agent list k. Used to track the winning agent for each task in winning bids y
        
        Matrix is a pandas dataframe of size N_t x 1, initialized as a matrix of empty strings, with task ids as index
        """
        self.winning_agents_k = pd.DataFrame(
            np.array(["" for _ in range(self.Task_count_N_t)]),
            index=self.tasklog.ids_pending,
            columns=["winning_agents_k"]
        )

        """
        Last update s. Used to track the last update time for each agent in the fleet
        
        Matrix is a pandas dataframe of size N_u x 1, initialized as zero matrix, with agent ids as index
        """
        self.last_update_s = pd.DataFrame(
            np.zeros(self.Agent_count_N_u),
            index=self.fleet.ids_active,
            columns=["last_update_s"]
        )

    # ============================================================== PROPERTIES
    # ---------------- Self state
    @property
    def local_allocation_state(self) -> dict:
        """
        Local allocation state at current time step (not serialised)
        !!!! All entries must be dataframes !!!!

        :return: dict
        """
        return {
            "path_p": pd.DataFrame(self.agent.plan.task_sequence, columns=["path_p"]),
            "bundle_b": pd.DataFrame(self.agent.plan.task_bundle, columns=["bundle_b"]),
            "bids_depth_e": self.bids_depth_e,
            "winning_agents_k": self.winning_agents_k,
            "last_update_s": self.last_update_s,

            # > Base local allocation states
            "local_bids_c": self.local_bids_c,
            "local_allocations_d": self.local_allocations_d
        }

    # ============================================================== METHODS
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

            # > Remove task_path_p from the state (not needed for this operation)
            state.pop("path_p")

            # > Remove bundle_b from the state (not needed for this operation)
            state.pop("bundle_b")

            # > Remove winning_bids_y from the state (not needed for this operation)
            state.pop("winning_bids_y")

            # > Remove winning_agents_k from the state (not needed for this operation)
            state.pop("winning_agents_k")

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

            # > Remove task_path_p from the state (not needed for this operation)
            state.pop("path_p")

            # > Remove bundle_b from the state (not needed for this operation)
            state.pop("bundle_b")

            # > Remove winning_bids_y from the state (not needed for this operation)
            state.pop("winning_bids_y")

            # > Remove winning_agents_k from the state (not needed for this operation)
            state.pop("winning_agents_k")

            for matrix in state.values():
                if agent.id in matrix.columns:
                    matrix.drop(columns=agent.id, inplace=True)

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

            # > Remove task_path_p from the state (not needed for this operation)
            state.pop("path_p")

            # > Remove bundle_b from the state (not needed for this operation)
            state.pop("bundle_b")

            # > Remove winning_bids_y from the state (operation performed separately)
            state.pop("winning_bids_y")
            self.winning_bids_y.loc[task.id] = 0

            # > Remove winning_agents_k from the state (operation performed separately)
            state.pop("winning_agents_k")
            self.winning_agents_k.loc[task.id] = ""

            # > Remove last_update_s from the state (not needed for this operation)
            state.pop("last_update_s")

            for matrix in state.values():
                matrix.loc[task.id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)

        def terminate_task(task: Task) -> None:
            """
            Flag task as terminated in task log and remove all relevant allocation lists and matrices rows
            """

            # -> If self agent terminated task and task is current task id, only remove task
            if task.termination_source_id == self.id and task.id == self.agent.plan.current_task_id:
                self._drop_task(task_id=task.id, forward=False)

            # -> Cancel goal if task is assigned to self
            elif task.id in self.agent.plan:
                self._drop_task(task_id=task.id, forward=True)

            # -> Remove task from all allocation lists and matrices
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            # > Remove task_path_p from the state (operation performed separately)
            state.pop("path_p")

            # > Remove bundle_b from the state (operation performed separately)
            state.pop("bundle_b")

            # > Remove last_update_s from the state (not needed for this operation)
            state.pop("last_update_s")

            for matrix in state.values():
                if task.id in matrix.index:
                    matrix.drop(index=task.id, inplace=True)

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

    def update_task(self):
        pass

    @staticmethod
    def _update_decision(
                         k_agent_id: str,
                         k_winning_agent_id: str,
                         k_winning_bid_y_kj,
                         k_timestamp_matrix,

                         i_agent_id: str,
                         i_winning_agent_id: str,
                         i_winning_bid_y_ij,
                         i_timestamp_matrix
                         ) -> str:
        """
        Generate an update decision
        
        :param k_agent_id: The agent k that sent the message
        :param k_winning_agent_id: The agent that won the task in k's allocation
        :param k_winning_bid_y_kj: The winning bid in k's allocation
        :param k_timestamp_matrix: The timestamp matrix of agent k

        :param i_agent_id: The agent i that received the message
        :param i_winning_agent_id: The agent that won the task in i's allocation
        :param i_winning_bid_y_ij: The winning bid in i's allocation
        :param i_timestamp_matrix: The timestamp matrix of agent i

        :return: The decision to update, reset, or leave the task
        """

        # -> k agent beliefs
        k_thinks_agent_k_won = k_winning_agent_id == k_agent_id
        k_thinks_agent_i_won = k_winning_agent_id == i_agent_id
        k_thinks_agent_m_won = k_winning_agent_id not in [k_agent_id, i_agent_id] and k_winning_agent_id != ""
        k_thinks_task_unassigned = k_winning_agent_id == ""

        # > Verify that only one is true
        assert sum([k_thinks_agent_k_won, k_thinks_agent_i_won, k_thinks_agent_m_won, k_thinks_task_unassigned]) == 1

        # -> i agent beliefs
        i_thinks_agent_i_won = i_winning_agent_id == i_agent_id
        i_thinks_agent_k_won = i_winning_agent_id == k_agent_id
        i_thinks_agent_m_won = i_winning_agent_id not in [i_agent_id, k_agent_id] and i_winning_agent_id != "" and i_winning_agent_id == k_winning_agent_id
        i_thinks_agent_n_won = i_winning_agent_id not in [i_agent_id, k_agent_id, k_winning_agent_id] and i_winning_agent_id != ""
        i_thinks_task_unassigned = i_winning_agent_id == ""

        # > Verify that only one is true
        assert sum([i_thinks_agent_i_won, i_thinks_agent_k_won, i_thinks_agent_m_won, i_thinks_agent_n_won, i_thinks_task_unassigned]) == 1

        # --------------------------------------------------------- 1
        # > Sender thinks they won the task
        if k_thinks_agent_k_won:

            # > Local agent thinks they won the task
            if i_thinks_agent_i_won:
                # > If the k has a higher bid: update
                if k_winning_bid_y_kj > i_winning_bid_y_ij:
                    return 'update'

                # > Default to leave
                else:
                    return 'leave'

            # > Local agent thinks k won the task: update
            elif i_thinks_agent_k_won:
                return 'update'

            # > Local agent thinks someone else won the task
            elif i_thinks_agent_m_won:

                # > If the k bid is more recent or higher: update
                if (k_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"]
                        or k_winning_bid_y_kj > i_winning_bid_y_ij):
                    return 'update'

                # > Default to leave
                else:
                    return 'leave'

            # > Local agent thinks the task is unassigned: update
            elif i_thinks_task_unassigned:
                return 'update'

            else:
                raise ValueError("Invalid state")

        # --------------------------------------------------------- 2
        # > Sender thinks i agent won the task
        elif k_thinks_agent_i_won:

            # > Local agent thinks they won the task: leave
            if i_thinks_agent_i_won:
                return 'leave'

            # > Local agent thinks k won the task: reset
            elif i_thinks_agent_k_won:
                return 'reset'

            # > Local agent thinks someone else won the task
            elif i_thinks_agent_m_won:
                # > If the k bid is more recent: reset
                if k_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"]:
                    return 'reset'

                # > Default to leave
                else:
                    return 'leave'

            # > Local agent thinks the task is unassigned: leave
            elif i_thinks_task_unassigned:
                return 'leave'

            else:
                raise ValueError("Invalid state")

        # --------------------------------------------------------- 3
        # > Sender thinks someone else won the task
        elif k_thinks_agent_m_won:

            # > Local agent thinks they won the task
            if i_thinks_agent_i_won:

                # > If the k bid is more recent and higher: update
                if (k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"]
                        and k_winning_bid_y_kj > i_winning_bid_y_ij):
                    return 'update'

                # > Default to leave
                else:
                    return 'leave'

            # > Local agent thinks k won the task
            elif i_thinks_agent_k_won:

                # > If the k bid is more recent: update
                if k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"]:
                    return 'update'

                # > Else: reset
                else:
                    return 'reset'

            # > Local agent also thinks the same agent won the task
            elif i_thinks_agent_m_won:

                # > If the k bid is more recent and higher: update
                if k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"]:
                    return 'update'

                # > Default to leave
                else:
                    return 'leave'

            # > Local agent thinks someone else won the task, and it is not the same as the k's winning agent
            elif i_thinks_agent_n_won:

                # > If the k bid is more recent both ways: update
                if (k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"]
                        and k_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"]):
                    return 'update'

                # > If the k bid is more recent and higher: update
                elif (k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"]
                      and k_winning_bid_y_kj > i_winning_bid_y_ij):
                    return 'update'

                # > If the bids both are more recent crosswise: reset
                elif (k_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"]
                      and i_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"]):
                    return 'reset'

                # > Default to leave
                else:
                    return 'leave'

            # > Local agent thinks the task is unassigned
            elif i_thinks_task_unassigned:
                return 'leave'

            else:
                raise ValueError("Invalid state")

        # > Sender thinks the task is unassigned
        elif k_thinks_task_unassigned:

            # > Local agent thinks they won the task: leave
            if i_thinks_agent_i_won:
                return 'leave'

            # > Local agent thinks k won the task: update
            elif i_thinks_agent_k_won:
                return 'update'

            # > Local agent thinks someone else won the task
            elif i_thinks_agent_m_won:

                # > If the k bid is more recent: update
                if k_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"]:
                    return 'update'

                # > Default to leave
                else:
                    return 'leave'

            # > Local agent thinks the task is unassigned
            elif i_thinks_task_unassigned:
                return 'leave'

            else:
                raise ValueError("Invalid state")

        else:
            raise ValueError("Invalid state")

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
            while len(self.agent.plan) < len(self.tasklog.pending_tasks):
                # -> Calculate bids
                # > List tasks not in plan
                remaining_tasks = [task for task in self.tasklog.pending_tasks if task.id not in self.agent.plan]

                # > For each task not in the plan ...
                for task in remaining_tasks:
                    # -> Compute bids
                    self._bid(task=task, agent=[self.agent])   # TODO: Review to better integrate intercession

                # -> Merge local bids c into shared bids b
                # > For each task ...
                for task_id in self.tasklog.ids_pending:
                    # > For each agent ...
                    for agent_id in self.fleet.ids_active:
                        # -> Priority merge local bids c into current bids b
                        # > Determine correct matrix values
                        shared_bids_b_ij_updated, shared_bids_priority_beta_ij_updated = self.priority_merge(
                            # Logging
                            task_id=task_id,
                            agent_id=agent_id,

                            # Merging
                            matrix_updated_ij=self.shared_bids_b.loc[task_id, agent_id],
                            matrix_source_ij=self.local_bids_c.loc[task_id, agent_id],
                            priority_updated_ij=self.shared_bids_priority_beta.loc[task_id, agent_id],
                            priority_source_ij=self.hierarchy_level,

                            # Reset
                            currently_assigned=self.agent.plan.has_task(task_id),
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

                        # -> Merge current bids b into local bids c according to intercession depth e
                        if self.bids_depth_e.loc[task_id, agent_id] == DEEP:
                            self.local_bids_c.loc[task_id, agent_id] = self.shared_bids_b.loc[task_id, agent_id]

                # ---- Select task
                # -> Create list of valid tasks (pandas series of task ids initialised as 0)
                valid_tasks_list_h = pd.DataFrame(
                    np.zeros(len(remaining_tasks)),
                    index=[task.id for task in remaining_tasks],
                    columns=["valid_tasks_list_h"]
                )

                # > For each task ...
                for task in remaining_tasks:
                    # -> Create set of agents with imposed allocations
                    agents_with_imposed_allocations = list(
                        set(self.shared_allocations_a.loc[task.id, self.shared_allocations_a.loc[task.id] == 1].index))

                    # > If there are imposed allocations, check if self is imposed allocation with the highest priority
                    if len(agents_with_imposed_allocations) > 0:
                        # -> Find agent with the highest priority
                        winning_agent = self.shared_allocations_priority_alpha.loc[
                            task.id, agents_with_imposed_allocations].idxmax()

                        # TODO: Adjust to handle conflicting imposed allocations. For now assumed that allocation intercession is conflict-free

                        # > If self is the winning agent, add the task to the task list
                        if winning_agent == self.id:
                            valid_tasks_list_h[task.id, "valid_tasks_list_h"] = 1
                    else:
                        # > If there are no imposed allocations, check if self has the highest bid and is not blacklisted
                        valid_task = int(
                            self.shared_allocations_a.loc[task.id, self.id] != -1 and self.shared_bids_b.loc[task.id, self.id] >
                            self.winning_bids_y.loc[task.id, "winning_bids_y"])

                        valid_tasks_list_h.loc[task.id, "valid_tasks_list_h"] = valid_task

                # -> If there are valid tasks, select the task with the highest bid
                if valid_tasks_list_h["valid_tasks_list_h"].sum() > 0:
                    # -> Select task with the highest bid
                    # > Get index of all the tasks for which the valid_tasks_list_h is 1
                    valid_tasks = valid_tasks_list_h[valid_tasks_list_h["valid_tasks_list_h"] == 1].index.to_list()

                    # > Get valid task with largest bid (using local bids !!!)
                    selected_task_id = self.local_bids_c.loc[valid_tasks, self.id].idxmax()

                    # -> Update winning bids
                    self.winning_bids_y.loc[selected_task_id, "winning_bids_y"] = self.shared_bids_b.loc[selected_task_id, self.id]

                    # -> Add task to plan
                    self.agent.add_task_to_plan(
                        tasklog=self.tasklog,
                        task=selected_task_id,
                        position=self.agent.local["insertions"][selected_task_id],
                        logger=self.get_logger()
                    )

                    # > Publish goal msg
                    self.publish_goal_msg(meta_action="update", traceback="Select task")

    def _bid(self, task: Task, agent_lst: list[Agent]) -> list[dict]:
        """
        Find the largest marginal gain achieved from inserting a task into the plan at the most beneficial position

        :param task: Task to bid for
        :param agent_lst: List of agents to compute bids for

        :return: List of dictionaries containing the agent(s) ID(s) and corresponding marginal gains according to insertion position
        """

        if self.env is None:
            # -> Return 0 bids for all agents as the environment is not available
            self.get_logger().warning("!!!!!! WARNING: Environment not available")
            return []

        # -> If no bid evaluation function, return empty list
        if self.bid_evaluation_function is None:
            return []

        # -> Compute the marginal gains for the agent
        agents_marginal_gains = self.bid_evaluation_function(
            task=task,
            tasklog=self.tasklog,
            agent_lst=agent_lst,
            fleet=self.fleet,
            environment=self.env,
            logger=self.get_logger()
        )

        # > For each agent ...
        for marginal_gains in agents_marginal_gains:
            # -> Find the insertion with the highest marginal gain
            insertion_loc, bid = max(marginal_gains.items(), key=lambda x: x[1]["value"])

            # -> Store bid to local bids matrix
            # > Bid
            self.local_bids_c.loc[task.id, bid["agent_id"]] = bid["bid"]

            # > Bid depth
            self.bids_depth_e.loc[task.id, bid["agent_id"]] = bid["bids_depth"]

            # > Allocation
            self.local_allocations_d.loc[task.id, bid["agent_id"]] = bid["allocation"]

            # -> Store insertion to agent local
            if ["insertions"] not in self.fleet[bid["agent_id"]].local:
                self.fleet[bid["agent_id"]].local['insertions'] = {}

            self.fleet[bid["agent_id"]].local['insertions'][task.id] = insertion_loc

        return agents_marginal_gains

    def _drop_task(self,
                  task_id: str,
                  reset: bool = False,
                  forward: bool = True,     # Must default to true to work with base priority merge method
                  traceback: str = None,
                  logger=True
                  ) -> None:
        """
        Drop a task from the bundle list and plan

        :param task_id: Id of task to drop
        :param reset: Flag to reset the winning bids y for the task
        :param forward: Whether to drop the task and all following tasks in the plan
        :param traceback: The reason for dropping the task
        :param logger: Whether to log the task drop
        """

        if reset:
            # > Reset winning bids
            self.winning_bids_y.loc[task_id, "winning_bids_y"] = 0

        # > Remove the task from the plan
        self.agent.remove_task_from_plan(
            tasklog=self.tasklog,
            task=task_id,
            forward=forward,
            logger=self.get_logger() if logger else None
        )

        # > Publish goal msg
        self.publish_goal_msg(meta_action="update", traceback=traceback)

    # >>>> Prints
