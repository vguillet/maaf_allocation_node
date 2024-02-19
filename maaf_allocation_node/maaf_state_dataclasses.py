

from dataclasses import dataclass, fields, field
from .maaf_dataclass_cores import State

# Create immutable data classes for the state of an item


@dataclass
class Agent_state(State):
    # --- Metadata ---
    agent_id: str or int                # The ID of the agent the state is for

    # --- Status ---
    battery_level: float = None         # The battery level of the target agent in percentage
    __min_battery_level: float = 20     # The minimum battery level at which the agent is considered to have low battery

    stuck: bool = None                  # Whether the target agent is stuck or not

    # --- Position ---
    x: float = None                     # The x-coordinate of the target agent
    y: float = None                     # The y-coordinate of the target agent
    z: float = None                     # The z-coordinate of the target agent

    # --- Orientation ---
    u: float = None                     # The x-component of the velocity of the target agent
    v: float = None                     # The y-component of the velocity of the target agent
    w: float = None                     # The z-component of the velocity of the target agent

    def __repr__(self) -> str:
        return f"State of agent {self.agent_id} at {self.timestamp}"

    @property
    def status(self) -> str:
        """

        """

        active = True

        # -> Check if battery level is low
        active *= self.battery_level > self.__min_battery_level

        # -> Check if the agent is stuck
        active *= not self.stuck

        return "active" if active else "inactive"