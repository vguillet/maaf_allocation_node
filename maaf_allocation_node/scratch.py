from dataclasses import dataclass, field
from typing import Any, Dict
from abc import ABC, abstractmethod
import json


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


# Example usage:
nested_dict = NestedDict()
nested_dict['key1']['key2']['key3'] = 'value'

print(nested_dict.data)

# Serialize NestedDict dictionary to JSON
serialized = json.dumps(nested_dict)

print(serialized)

# Deserialize JSON to NestedDict dictionary
deserialized = json.loads(serialized)

# Convert NestedDict dictionary to NestedDict instance
nested_dict = NestedDict().from_dict(deserialized)

print(nested_dict.data)
