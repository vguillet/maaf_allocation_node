
from dataclasses import dataclass, fields, field
from typing import List, Optional, Dict, Any
from datetime import datetime
from abc import ABC, abstractmethod

DEBUG = True


class MaafItem(ABC):
    def __reduce__(self):
        """
        Reduce the item to a dictionary.
        """
        return self.__class__, (*self.asdict(),)  # -> Pass dictionary representation to constructor

    @abstractmethod
    def asdict(self) -> dict:
        """
        Create a dictionary containing the fields of the item data class instance with their current values.

        :return: A dictionary with field names as keys and current values.
        """
        pass

    @classmethod
    @abstractmethod
    def from_dict(cls, item_dict: dict):
        """
        Convert a dictionary to an item.

        :param item_dict: The dictionary representation of the item

        :return: An item object
        """
        pass


@dataclass
class NestedDict(MaafItem):
    data: Dict[str, Any] = field(default_factory=dict)

    def __getitem__(self, key):
        if key not in self.data:
            self.data[key] = NestedDict()
        return self.data[key]

    def __setitem__(self, key, value):
        if key not in self.data:
            self.data[key] = NestedDict()

        self.data[key] = value

    def asdict(self) -> dict:
        """
        Create a dictionary containing the fields of the item data class instance with their current values.

        :return: A dictionary with field names as keys and current values.
        """

        data_dict = {}

        for key, value in self.data.items():
            if isinstance(value, NestedDict):
                data_dict[key] = value.asdict()
            else:
                data_dict[key] = value

        return data_dict

    @classmethod
    def from_dict(cls, item_dict: dict):
        """
        Convert a dictionary to an item.

        :param item_dict: The dictionary representation of the item

        :return: An item object
        """

        for key, value in item_dict.items():
            if isinstance(value, dict):
                item_dict[key] = cls().from_dict(value)

        return cls(data=item_dict)


@dataclass
class MaafList:
    items: list = field(default_factory=list)
    item_class = None

    __on_add_item_listeners: list[callable] = field(default_factory=list)
    __on_update_item_listeners: list[callable] = field(default_factory=list)
    __on_remove_item_listeners: list[callable] = field(default_factory=list)
    __on_edit_list_listeners: list[callable] = field(default_factory=list)
    manual_on_edit_list_listeners_call = False

    def __str__(self):
        return self.__repr__()

    # ============================================================== Properties
    # ------------------------------ IDs
    @property
    def ids(self) -> list[int]:
        """
        Get a list of item ids in the item log.
        """
        return [item.id for item in self.items]

    # ============================================================== Listeners
    # ------------------------------ Edit
    def add_on_edit_list_listener(self, listener: callable) -> None:
        """
        Add a listener to be called when the item list is edited.

        :param listener: The listener to add.
        """
        self.__on_edit_list_listeners.append(listener)

    def call_on_edit_list_listeners(self, item: item_class) -> None:
        """
        Call all on_edit_list listeners.
        """

        # -> If the manual_on_edit_list_listeners_call flag is set, return
        if self.manual_on_edit_list_listeners_call:
            return

        # -> Call all on_edit_list listeners
        for listener in self.__on_edit_list_listeners:
            listener(item)

    # ------------------------------ Update
    def add_on_update_item_listener(self, listener: callable) -> None:
        """
        Add a listener to be called when an item is updated in the item log.

        :param listener: The listener to add.
        """
        self.__on_update_item_listeners.append(listener)

    def call_on_update_item_listeners(self, item: item_class) -> None:
        """
        Call all on_update_item listeners.

        :param item: The item that was updated.
        """
        # -> Call all on_update_item listeners
        for listener in self.__on_update_item_listeners:
            listener(item)

        # -> Call all on_edit_list listeners
        self.call_on_edit_list_listeners(item)

    # ------------------------------ Add
    def add_on_add_item_listener(self, listener: callable) -> None:
        """
        Add a listener to be called when an item is added to the item log.

        :param listener: The listener to add.
        """
        self.__on_add_item_listeners.append(listener)

    def call_on_add_item_listeners(self, item: item_class) -> None:
        """
        Call all on_add_item listeners.

        :param item: The item that was added.
        """
        # -> Call all on_add_item listeners
        for listener in self.__on_add_item_listeners:
            listener(item)

        # -> Call all on_edit_list listeners
        self.call_on_edit_list_listeners(item)

    # ------------------------------ Remove
    def add_on_remove_item_listener(self, listener: callable) -> None:
        """
        Add a listener to be called when an item is removed from the item log.

        :param listener: The listener to add.
        """
        self.__on_remove_item_listeners.append(listener)

    def call_on_remove_item_listeners(self, item: item_class) -> None:
        """
        Call all on_remove_item listeners.

        :param item: The item that was removed.
        """
        # -> Call all on_remove_item listeners
        for listener in self.__on_remove_item_listeners:
            listener(item)

        # -> Call all on_edit_list listeners
        self.call_on_edit_list_listeners(item)

    # ============================================================== Sort
    def sort(self, key: callable = None, reverse: bool = False) -> None:
        """
        Sort the items in the item log.

        :param key: The key to sort the items by.
        :param reverse: Whether to sort the items in reverse order.
        """
        self.items.sort(key=key, reverse=reverse)

    # ============================================================== Get
    def __len__(self) -> int:
        """
        Retrieve number of items in list
        """
        return len(self.items)

    def __iter__(self) -> iter:
        """
        Retrieve an iterator for the item log.
        """
        # -> Sort items list by id
        self.sort(key=lambda x: x.id)

        return iter(self.items)

    def __getitem__(self, item_id: int or str) -> Optional[item_class]:
        """
        Retrieve an item from the item log by its id as index.
        """
        # -> Find the item with the given ID
        for item in self.items:
            # > If the item exists, return it
            if item.id == item_id:
                return item

        # > If the item does not exist, warn the user and return None
        if DEBUG: print(
            f"!!! Get item by index failed: {self.item_class} with id '{item_id}' does not exist in the item log !!!")
        return None

    def __reduce__(self):
        """
        Reduce the item log
        """
        return self.__class__, (self.items,)

    # ============================================================== Set
    def update_item_fields(self,
                           item: int or str or item_class,
                           field_value_pair: dict,
                           add_to_local: bool = False,
                           add_to_shared: bool = False
                           ) -> None:
        """
        Update fields of an item with given item_id.
        Returns True if item is updated successfully, False otherwise.

        :param item: The item to update.
        :param field_value_pair: A dict containing the field and value to update.
        :param add_to_local: Whether to add the field to the local field if it does not exist.
        :param add_to_shared: Whether to add the field to the shared field if it does not exist.
        """

        assert add_to_local + add_to_shared < 2, "Cannot add to both local and shared fields"

        # -> If item is a string or int, find the item with the given ID
        if isinstance(item, str) or isinstance(item, int):
            item = self[item]

        # -> If item is an item object, find the item with the given ID
        else:
            item = self[item.id]

        if item is None:
            raise ValueError(f"!!! Update item fields failed: {self.item_class} with id '{item}' does not exist in the item log !!!")
            return None

        # If the item does not exist, warn the user
        elif item.id not in self.ids:
            raise ValueError(f"!!! Update item fields failed: {self.item_class} with id '{item.id}' does not exist in the item log !!!")
            return

        # -> Update the fields of the item
        for key, value in field_value_pair.items():
            # > Update the field if it exists
            if hasattr(item, key):
                setattr(item, key, value)

            # > Check if the field exist in the shared field
            elif hasattr(item.shared, key):
                setattr(item.shared, key, value)

            # > Check if the field exist in the local field
            elif hasattr(item.local, key):
                setattr(item.local, key, value)

            # > Add the field to the local field if it does not exist and add_to_local is True
            elif add_to_local:
                item.local[key] = value

            # > Add the field to the shared field if it does not exist and add_to_shared is True
            elif add_to_shared:
                item.shared[key] = value

            else:
                raise ValueError(f"!!! Update item fields failed (3): {self.item_class} with id '{item.id}' does not have field '{key}' !!!")

        # -> Call the on_update_item method
        self.call_on_update_item_listeners(item)

    # ============================================================== Add
    def add_item(self, item: item_class) -> None:
        """
        Add an item to the item log. If the item is a list, add each item to the item log individually recursively.

        :param item: The item to add to the item log.
        """
        # -> Check if the item already exists in the item log
        # > If the item already exists, skip and warn user
        if item in self.items:
            if DEBUG: print(f"!!! Add item failed: {self.item_class} with id '{item.id}' already exists in the item log !!!")

        # > else, add the item to the item log
        else:
            self.items.append(item)

        # -> Sort items list by id
        self.sort(key=lambda x: x.id)

        # -> Call the on_add_item method
        self.call_on_add_item_listeners(item)

    def add_item_by_dict(self, item_data: dict) -> None:
        """
        Add a item to the item log using a dictionary.

        :param item_data: A dictionary containing the fields of the item.
        """

        # -> Convert the dictionary to an item object
        new_item = self.item_class.from_dict(item_data)

        # -> Add the item to the item log
        self.add_item(new_item)

    # ============================================================== Remove
    def remove_item(self, item: item_class) -> None:
        """
        Remove an item from the item log.

        :param item: The item to remove from the item log.
        """

        # -> Remove the item from the item log
        try:
            self.items.remove(item)
        except ValueError:
            if DEBUG: print(f"!!! Remove item failed: {self.item_class} with id '{item.id}' does not exist in the item log !!!")

        # -> Call the on_remove_item method
        self.call_on_remove_item_listeners(item)

    def remove_item_by_id(self, item_id: str or int) -> None:
        """
        Remove a item from the item log.

        :param item_id: The id of the item to remove from the item log.
        """
        # -> Find the item with the given ID
        item = next((t for t in self.items if t.id == item_id), None)

        # -> If the item exists, remove it from the item log
        if item:
            self.remove_item(item)
        else:
            if DEBUG: print(f"!!! Remove item by id failed: {self.item_class} with id '{item_id}' does not exist in the item log !!!")

    # ============================================================== From
    def from_list(self, item_dicts_list: List[dict]) -> None:
        """
        Add a list of items to the item log.

        :param item_dicts_list: A list of item dictionaries.
        """
        for item_dict in item_dicts_list:
            self.add_item_by_dict(item_dict)

    # ============================================================== To
    def to_list(self) -> List[dict]:
        """
        Convert the item log to a list of dictionaries.

        :return: A list of dictionaries containing the fields of the items in the item log.
        """
        return [item.asdict() for item in self.items]
