ALLOCATION_METHOD = "CBAA w. intercession"

DEBUG = False

# ----------------- SIMULATION MODE -----------------
# >>>>>>>>>> RUN MODES DEFINITION
SIM = 0
OPERATIONAL = 1
# >>>>>>>>>>

RUN_MODE = OPERATIONAL

# ----------------- NODE CONFIGURATION -----------------
FLEET_MSG_UPDATE_TIMER = 1


# ----------------- SIMULATION CONFIGURATION -----------------
# TODO: Cleanup
env_size = 6    # Environment size (6 for a 7x7 grid

gotto_task_count = 4
action_1_task_count = 2     # action_1 task     (even goal ids)
action_2_task_count = 2     # action_2 tasks    (odd goal ids)

assert gotto_task_count == (action_1_task_count + action_2_task_count)

agent_lst = ["Turtle_1", "Turtle_2", "Turtle_3", "Turtle_4"]
# agent_lst = ["Turtle_1", "Turtle_2"]

skillsets = {
    "Operator": [],
    "Turtle_1": ["GOTO", "ACTION_1"],
    "Turtle_2": ["GOTO", "ACTION_1"],
    "Turtle_3": ["GOTO", "ACTION_2"],
    "Turtle_4": ["GOTO", "ACTION_2"]
}

bid_function = {
    "Operator": "anticipated_action_task_interceding_agent",
    "Turtle_1": "graph_weighted_manhattan_distance_bid",
    "Turtle_2": "graph_weighted_manhattan_distance_bid",
    "Turtle_3": "graph_weighted_manhattan_distance_bid",
    "Turtle_4": "graph_weighted_manhattan_distance_bid"
}
