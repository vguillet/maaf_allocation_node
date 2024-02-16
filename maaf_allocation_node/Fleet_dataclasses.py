from dataclasses import dataclass, fields, field
from typing import List, Optional
from datetime import datetime
from .maaf_dataclass_cores import maaf_list_dataclass

DEBUG = True


@dataclass
class Agent:
    id: str or int
    name: str
    agent_class: str
    hierarchy_level: int
    affiliations: List[str]
    specs: dict
    skillset: List[str]
    status: str
    state: str = None
    last_contact_timestamp: datetime = None

    def __repr__(self) -> str:
        return f"Agent {self.name} ({self.id}) of class {self.agent_class} - Status: {self.status}"

    def __str__(self) -> str:
        return self.__repr__()

    def has_affiliation(self, affiliation: str) -> bool:
        """
        Check if the agent has a given affiliation
        """
        return affiliation in self.affiliations

    # ============================================================== To
    def to_dict(self) -> dict:
        """
        Create a dictionary containing the fields of the Agent data class instance with their current values.

        :return: A dictionary with field names as keys and current values.
        """
        # -> Get the fields of the Agent class
        agent_fields = fields(Agent)

        # -> Create a dictionary with field names as keys and their current values
        fields_dict = {f.name: getattr(self, f.name) for f in agent_fields}

        return fields_dict

    # ============================================================== From
    @classmethod
    def from_dict(cls, agent_dict: dict) -> "Agent":
        """
        Convert a dictionary to an agent.

        :param agent_dict: The dictionary representation of the agent

        :return: An agent object
        """
        # -> Get the fields of the Agent class
        agent_fields = fields(cls)

        # -> Extract field names from the fields
        field_names = {field.name for field in agent_fields}

        # -> Check if all required fields are present in the dictionary
        if not field_names.issubset(agent_dict.keys()):
            raise ValueError(f"!!! Agent creation from dictionary failed: Agent dictionary is missing required fields: {agent_dict.keys() - field_names} !!!")

        # -> Extract values from the dictionary for the fields present in the class
        field_values = {field.name: agent_dict[field.name] for field in agent_fields}

        # -> Create and return an Agent object
        return cls(**field_values)


class Fleet(maaf_list_dataclass):
    item_class = Agent

    def __repr__(self):
        return f"Fleet: {len(self.items)} agents ({len(self.agents_active)} active, {len(self.agents_inactive)} inactive)"

    # ============================================================== Properties
    # ------------------------------ IDs
    @property
    def ids_active(self) -> List[str or int]:
        """
        Get the list of IDs of the active agents in the fleet
        """
        return [agent.id for agent in self.items if agent.status == "active"]

    @property
    def ids_inactive(self) -> List[str or int]:
        """
        Get the list of IDs of the inactive agents in the fleet
        """
        return [agent.id for agent in self.items if agent.status == "inactive"]

    # ------------------------------ Agents
    @property
    def agents_active(self) -> List[Agent]:
        """
        Get the list of active agents in the fleet
        """
        return [agent for agent in self.items if agent.status == "active"]

    @property
    def agents_inactive(self) -> List[Agent]:
        """
        Get the list of inactive agents in the fleet
        """
        return [agent for agent in self.items if agent.status == "inactive"]

    # ============================================================== Get
    def query(self,
              agent_class: Optional[str] = None,
              hierarchy_level: Optional[int] = None,
              affiliation: Optional[str] = None,
              status: Optional[str] = None
              ) -> List[Agent]:
        """
        Query the fleet for agents that match the given criteria.

        :param agent_class: The class of agent to filter for.
        :param hierarchy_level: The hierarchy level of the agent to filter for.
        :param affiliation: The affiliation of the agent to filter for.
        :param status: The status of the agent to filter for.

        :return: A list of agents that match the given criteria.
        """

        filtered_agents = self.items

        if agent_class:
            filtered_agents = [agent for agent in filtered_agents if agent.agent_class == agent_class]
        if hierarchy_level is not None:
            filtered_agents = [agent for agent in filtered_agents if agent.hierarchy_level == hierarchy_level]
        if affiliation:
            filtered_agents = [agent for agent in filtered_agents if affiliation in agent.affiliations]
        if status:
            filtered_agents = [agent for agent in filtered_agents if agent.status == status]

        return filtered_agents

    # ============================================================== Set
    def flag_agent_active(self, agent: str or int or item_class or List[int or str or item_class]) -> None:
        """
        Mark an agent as active in the fleet.

        :param agent: The agent to flag as active. Can be the agent object, the agent ID, or a list of agent IDs.
        """

        # -> If the input is a list, flag each agent in the list as active individually recursively
        if isinstance(agent, list):
            for a in agent:
                self.flag_agent_active(agent=a)
            return

        # > Update the agent status to 'active'
        self.update_item_fields(
            item=agent,
            field_value_pair={"status": "active"}
        )

    def flag_agent_inactive(self, agent: str or int or List[int or str]) -> None:
        """
        Mark an agent as inactive in the fleet.

        :param agent: The agent to flag as inactive. Can be the agent object, the agent ID, or a list of agent IDs.
        """

        # -> If the input is a list, flag each agent in the list as inactive individually recursively
        if isinstance(agent, list):
            for a in agent:
                self.flag_agent_inactive(agent=a)
            return

        # > Update the agent status to 'inactive'
        self.update_item_fields(
            item=agent,
            field_value_pair={"status": "inactive"}
        )

    # ============================================================== Add
    def add_agent(self, agent: dict or Agent or List[dict or Agent]) -> None:
        """
        Add an agent to the fleet. If the agent is a list, add each agent to the fleet individually recursively.

        :param agent: The agent to add to the fleet.
        """
        # -> If the agent is a list, add each agent to the fleet individually recursively
        if isinstance(agent, list):
            for a in agent:
                self.add_agent(agent=a)
            return

        # > Add the agent to the fleet
        if isinstance(agent, self.item_class):
            self.add_item(item=agent)
        else:
            self.add_item_by_dict(item_data=agent)

    # ============================================================== Remove
    def remove_agent(self, agent: str or int or item_class or List[int or str or item_class]) -> None:
        """
        Remove an agent from the fleet.

        :param agent: The agent to remove from the fleet. Can be the agent object, the agent ID, or a list of agent IDs.
        """
        # -> If the input is a list, remove each agent in the list from the fleet individually recursively
        if isinstance(agent, list):
            for a in agent:
                self.remove_agent(agent=a)
            return

        # > Remove the agent from the fleet
        if isinstance(agent, self.item_class):
            self.remove_item(item=agent)
        else:
            self.remove_item_by_id(item_id=agent)


if "__main__" == __name__:
    # Test Agent data class
    agent1 = Agent(
        id=1,
        name="Agent 1",
        agent_class="class 1",
        hierarchy_level=1,
        affiliations=["affiliation 1", "affiliation 2"],
        specs={"spec1": "value1", "spec2": "value2"},
        skillset=["skill1", "skill2"],
        status="active",
        state="state 1",
        last_contact_timestamp=datetime.now()
    )
    print(agent1)
    print(agent1.to_dict())

    agent2 = Agent.from_dict(agent1.to_dict())
    print(agent2)

    # Test Fleet data class
    print("\nTest Fleet dataclass\n")
    fleet = Fleet()

    print("\nAdd agent to fleet:")
    fleet.add_agent(agent1)
    print(fleet)
    print(fleet.to_list())

    print("\nMark agent as inactive:")
    fleet.flag_agent_inactive(agent1)
    print(fleet)

    print("\nMark agent as active:")
    fleet.flag_agent_active(agent1)
    print(fleet)

    print("\nRemove agent from fleet:")
    fleet.remove_agent(agent1)
    print(fleet)




