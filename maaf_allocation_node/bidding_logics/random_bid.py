
from random import randint

from maaf_tools.datastructures.fleet_dataclasses import Agent, Fleet
from maaf_tools.datastructures.task_dataclasses import Task

def random_bid(task: Task, agent_lst: list[Agent], env=None) -> list[dict]:
    bids = []

    for agent in agent_lst:
        bids.append({
            "agent_id": agent.id,
            "bid": randint(0, 10000)
        })

    return bids
