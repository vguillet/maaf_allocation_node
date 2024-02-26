
from random import randint

from maaf_allocation_node.fleet_dataclasses import Agent, Fleet
from maaf_allocation_node.task_dataclasses import Task

def random_bid(task: Task, agent_lst: list[Agent], env=None) -> list[dict]:
    bids = []

    for agent in agent_lst:
        bids.append({
            "agent_id": agent.id,
            "bid": randint(0, 10000)
        })

    return bids
