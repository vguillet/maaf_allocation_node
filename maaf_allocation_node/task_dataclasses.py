
from dataclasses import dataclass, fields, field
from typing import List, Optional
from datetime import datetime
from .dataclass_cores import MaafList, MaafItem

DEBUG = True


@dataclass
class Task(MaafItem):
    """
    Dataclass to represent a task in the maaf allocation framework.
    """
    # > Metadata
    id: str
    type: str
    creator: str
    affiliations: List[str]

    # > Task data
    priority: int
    instructions: dict[str]     # [(skill_ref, task_details_for skill), ...]

    # > Task status
    creation_timestamp: float
    termination_timestamp: Optional[datetime] = None
    termination_source_id: Optional[str] = None
    status: str = "pending"     # pending, completed, cancelled

    # name: str = ""
    shared: dict = field(default_factory=dict)  # Shared data of the agent, gets serialized and passed around
    local: dict = field(default_factory=dict)  # Local data of the agent, does not get serialized and passed around

    def __repr__(self) -> str:
        return f"Task {self.id} ({self.type}) - Creation timestamp: {self.creation_timestamp} - Status: {self.status} - Priority: {self.priority}"

    def __str__(self) -> str:
        return self.__repr__()

    def has_affiliation(self, affiliation: str) -> bool:
        """
        Check if the task has a specific affiliation.
        """
        return affiliation in self.affiliations

    # ============================================================== To
    def asdict(self) -> dict:
        """
        Create a dictionary containing the fields of the Task data class instance with their current values.

        :return: A dictionary with field names as keys and current values.
        """
        # -> Get the fields of the Task class
        task_fields = fields(self)

        # -> Exclude the local field
        task_fields = [f for f in task_fields if f.name != "local"]

        # -> Create a dictionary with field names as keys and their current values
        fields_dict = {f.name: getattr(self, f.name) for f in task_fields}

        return fields_dict

    # ============================================================== From
    @classmethod
    def from_dict(cls, task_dict: dict):
        """
        Convert a dictionary to a task.

        :param task_dict: The dictionary representation of the task

        :return: A task object
        """
        # -> Get the fields of the Task class
        task_fields = fields(cls)

        # -> Exclude the shared and local fields
        task_fields = [f for f in task_fields if f.name != "shared"]
        task_fields = [f for f in task_fields if f.name != "local"]

        # -> Extract field names from the fields
        field_names = {f.name for f in task_fields}

        # -> Check if all required fields are present in the dictionary
        if not field_names.issubset(task_dict.keys()):
            raise ValueError(f"!!! Task creation from dictionary failed: Task dictionary is missing required fields: {task_dict.keys() - field_names} !!!")

        # -> Extract values from the dictionary for the fields present in the class
        field_values = {f.name: task_dict[f.name] for f in task_fields}

        # -> Create and return a Task object
        return cls(**field_values)


@dataclass
class Task_log(MaafList):
    item_class = Task
    __on_status_change_listeners: list[callable] = field(default_factory=list)

    def __repr__(self):
        return f"Task log: {len(self.items)} tasks ({len(self.tasks_completed)} completed, {len(self.tasks_pending)} pending, {len(self.tasks_cancelled)} cancelled)"

    # ============================================================== Listeners
    # ------------------------------ Status
    def add_on_status_change_listener(self, listener):
        """
        Add a listener to the task log that listens for changes in the status of tasks.
        """
        self.__on_status_change_listeners.append(listener)

    def call_on_status_change_listeners(self, task: Task):
        """
        Call all listeners that are listening for changes in the status of tasks.
        """
        for listener in self.__on_status_change_listeners:
            listener(task)

    # ============================================================== Properties
    # ------------------------------ IDs
    @property
    def ids_completed(self) -> list[int]:
        """
        Get a list of completed task ids in the task log.
        """
        return [task.id for task in self.tasks_completed]

    @property
    def ids_pending(self) -> list[int]:
        """
        Get a list of pending task ids in the task log.
        """
        return [task.id for task in self.tasks_pending]

    @property
    def ids_cancelled(self) -> list[int]:
        """
        Get a list of cancelled task ids in the task log.
        """
        return [task.id for task in self.tasks_cancelled]

    # ------------------------------ Tasks
    @property
    def tasks_completed(self) -> list[Task]:
        """
        Get a list of completed tasks in the task log.
        """
        return [task for task in self.items if task.status == "completed"]

    @property
    def tasks_pending(self) -> list[Task]:
        """
        Get a list of pending tasks in the task log.
        """
        return [task for task in self.items if task.status == "pending"]

    @property
    def tasks_cancelled(self) -> list[Task]:
        """
        Get a list of cancelled tasks in the task log.
        """
        return [task for task in self.items if task.status == "cancelled"]

    # ============================================================== Get
    def query(
            self,
            task_type: Optional[str] = None,
            priority: Optional[int] = None,
            affiliation: Optional[List[str]] = None,
            status: Optional[str] = None
            ) -> List[Task]:
        """
        Query the task log for tasks that match the given criteria.

        :param task_type: The type of task to filter for.
        :param priority: The priority of the task to filter for.
        :param affiliation: The affiliation of the task to filter for.
        :param status: The status of the task to filter for.

        :return: A list of tasks that match the given criteria.
        """

        filtered_tasks = self.items

        if task_type:
            filtered_tasks = [task for task in filtered_tasks if task.type == task_type]
        if priority is not None:
            filtered_tasks = [task for task in filtered_tasks if task.priority == priority]
        if affiliation:
            filtered_tasks = [task for task in filtered_tasks if affiliation in task.affiliations]
        if status:
            filtered_tasks = [task for task in filtered_tasks if task.status == status]

        return filtered_tasks

    # ============================================================== Set
    def set_task_status(self,
                        task: int or str or item_class or List[int or str or item_class],
                        termination_status: str,
                        termination_source_id,
                        termination_timestamp
                        ) -> None:
        """
        Set the status of a task with given task_id.

        :param task: The task to set the status of. Can be a task id, task object, or a list of task ids or task objects.
        :param termination_status: The status to set the task to. Must be a list if task is a list.
        :param termination_source_id: The source id of the task termination. Must be a list if task is a list.
        :param termination_timestamp: The timestamp of the task termination. Must be a list if task is a list.
        """

        # -> If the task_id is a list, flag each task as completed individually recursively
        if isinstance(task, list):
            for i in range(len(task)):
                self.set_task_status(
                    task=task[i],
                    termination_status=termination_status[i],
                    termination_source_id=termination_source_id[i],
                    termination_timestamp=termination_timestamp[i]
                )
            return

        # -> Update the task status to 'completed'
        self.update_item_fields(
            item=task,
            field_value_pair={
                "status": termination_status,
                "termination_source_id": termination_source_id,
                "termination_timestamp": termination_timestamp
            }
        )

        # -> Call the on_status_change listeners
        self.call_on_status_change_listeners(task)

    def flag_task_completed(self,
                            task: int or str or item_class or List[int or str or item_class],
                            termination_source_id,
                            termination_timestamp
                            ) -> None:
        """
        Mark a task as completed with given task_id.

        :param task: The task to flag as completed. Can be a task id, task object, or a list of task ids or task objects.
        :param termination_source_id: The source id of the task termination. Must be a list if task is a list.
        :param termination_timestamp: The timestamp of the task termination. Must be a list if task is a list.
        """

        self.set_task_status(
            task=task,
            termination_status="completed",
            termination_source_id=termination_source_id,
            termination_timestamp=termination_timestamp
        )

    def flag_task_cancelled(self,
                            task: int or str or item_class or List[int or str or item_class],
                            termination_source_id,
                            termination_timestamp
                            ) -> None:
        """
        Mark a task as cancelled with given task_id.

        :param task: The task to flag as cancelled. Can be a task id, task object, or a list of task ids or task objects.
        :param termination_source_id: The source id of the task termination. Must be a list if task is a list.
        :param termination_timestamp: The timestamp of the task termination. Must be a list if task is a list.
        """

        self.set_task_status(
            task=task,
            termination_status="cancelled",
            termination_source_id=termination_source_id,
            termination_timestamp=termination_timestamp
        )

    # ============================================================== Add
    def add_task(self, task: dict or item_class or List[dict or item_class]) -> None:
        """
        Add a task to the task log. If the task is a list, add each task to the task log individually recursively.

        :param task: The task(s) to add to the task log. Can be a task object, a task dictionary, or a list of task objects or task dictionaries.
        """
        # -> If the task is a list, add each task to the task log individually recursively
        if isinstance(task, list):
            for t in task:
                self.add_task(t)
            return

        # -> Add the task to the task log
        if isinstance(task, self.item_class):
            self.add_item(item=task)
        else:
            self.add_item_by_dict(item_data=task)

    # ============================================================== Remove
    def remove_task(self, task: int or str or item_class or List[int or str or item_class]) -> None:
        """
        Remove a task from the task log. If the task is a list, remove each task from the task log individually recursively.

        :param task: The task(s) to remove from the task log. Can be a task id, task object, or a list of task ids or task objects.
        """
        # -> If the task is a list, remove each task from the task log individually recursively
        if isinstance(task, list):
            for t in task:
                self.remove_task(t)
            return

        # -> Remove the task from the task log
        if isinstance(task, self.item_class):
            self.remove_item(item=task)
        else:
            self.remove_item_by_id(item_id=task)


if "__main__" == __name__:
    # Test Task dataclass
    # print("\nTest Task dataclass:")
    task_1 = Task(
        id=1,
        type="delivery",
        creator="John Doe",
        affiliations=["aff1", "aff2"],
        priority=3,
        instructions=[("skill_1", "Deliver package to address 1"), ("skill_2", "Get signature from recipient")],
        creation_timestamp=datetime.now()
    )
    # print(task_1)
    # print(task_1.asdict())

    task_2 = Task.from_dict(task_1.asdict())
    # print(task_2)

    # Test Task_log dataclass
    # print("\nTest Task_log dataclass:")
    task_log = Task_log()

    # print("\nAdd tasks to the task log:")
    task_log.add_task(task_1)
    # print(task_log)
    # print(task_log.to_list())

    print("\nMark task 1 as completed:")
    task_log.flag_task_completed(1)
    print(task_log)

    print("\nMark task 1 as cancelled:")
    task_log.flag_task_cancelled(1)
    print(task_log)

    print("\nRemove task 1 from the task log:")
    task_log.remove_task(1)
    print(task_log)

    print("\nAdd multiple tasks to the task log:")
    task_log.add_task(task_1)
    task_log.add_task(task_2)
    print(task_log)

