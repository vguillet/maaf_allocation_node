
from dataclasses import dataclass, fields, field
from typing import List, Optional
from datetime import datetime

DEBUG = True


@dataclass
class maaf_list_dataclass:
    items: list = field(default_factory=list)
    item_class = None
    
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

    # ============================================================== Get
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

    # ============================================================== Set

    def update_item_fields(self, item: int or str or item_class, field_value_pair: dict) -> None:
        """
        Update fields of an item with given item_id.
        Returns True if item is updated successfully, False otherwise.

        :param item: The item to update.
        :param field_value_pair: A dict containing the field and value to update.
        """
        # -> If item is a string or int, find the item with the given ID
        if isinstance(item, (str, int)):
            item = self[item]

        # If the item does not exist, warn the user
        if item not in self.items:
            if DEBUG:
                if not isinstance(item, self.item_class):
                    print(f"!!! Update item fields failed: {self.item_class} with id '{item}' does not exist in the item log !!!")
                else:
                    print(f"!!! Update item fields failed: {self.item_class} with id '{item.id}' does not exist in the item log !!!")
            return

        for key, value in field_value_pair.items():
            # > Update the field if it exists
            if hasattr(item, key):
                setattr(item, key, value)
            else:
                if DEBUG: print(f"!!! Update item fields failed: {self.item_class} with id '{item.id}' does not have field '{key}' !!!")

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
        return [item.to_dict() for item in self.items]
