ALLOCATION_METHOD = "CBAA w. intercession"

DEBUG = False

# ----------------- SIMULATION MODE -----------------
# >>>>>>>>>> RUN MODES DEFINITION
SIM = 0
OPERATIONAL = 1
# >>>>>>>>>>

RUN_MODE = OPERATIONAL

# ----------------- NODE CONFIGURATION -----------------
FLEET_MSG_UPDATE_TIMER = 0.1


# ----------------- SIMULATION CONFIGURATION -----------------
NO_TASK = 0
ACTION_1 = 1
ACTION_2 = 2

# TODO: Cleanup
env_size = 9

gotto_task_count = 20
no_task_task_count = 6
action_1_task_count = 7     # action_1 task     (even goal ids)
action_2_task_count = 7     # action_2 tasks    (odd goal ids)

initial_tasks_announcement = 5

EVEN = 0
RANDOM = 1

release_spread = EVEN
release_ration = EVEN

release_max_epoch = 60

assert gotto_task_count == (no_task_task_count + action_1_task_count + action_2_task_count)

agent_lst = ["Turtle_1", "Turtle_2", "Turtle_3", "Turtle_4"]
# agent_lst = ["Turtle_1", "Turtle_2"]

skillsets = {
    # "Operator": [],
    "Operator": ["GOTO", "ACTION_1"],
    "Turtle_1": ["GOTO", "ACTION_1"],
    "Turtle_2": ["GOTO", "ACTION_1"],
    "Turtle_3": ["GOTO", "ACTION_2"],
    "Turtle_4": ["GOTO", "ACTION_2"]
}

bid_function = {
    # "Operator": "anticipated_action_task_interceding_agent",
    "Operator": "graph_weighted_manhattan_distance_bid",
    "Turtle_1": "graph_weighted_manhattan_distance_bid",
    "Turtle_2": "graph_weighted_manhattan_distance_bid",
    "Turtle_3": "graph_weighted_manhattan_distance_bid",
    "Turtle_4": "graph_weighted_manhattan_distance_bid"
}
