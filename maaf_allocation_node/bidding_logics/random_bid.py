
from random import randint

from maaf_tools.datastructures.task.Task import Task
from maaf_tools.datastructures.task.TaskLog import TaskLog

from maaf_tools.datastructures.agent.Agent import Agent
from maaf_tools.datastructures.agent.Fleet import Fleet
from maaf_tools.datastructures.agent.AgentState import AgentState


def random_bid(task: Task, agent_lst: list[Agent], env=None) -> list[dict]:
    bids = []

    for agent in agent_lst:
        bids.append({
            "agent_id": agent.id,
            "bid": randint(0, 10000)
        })

    return bids
