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

SIM_EPOCH_MIN_DURATION = 0.1

# ----------------- SIMULATION CONFIGURATION -----------------
NO_TASK = 0
ACTION_1 = 1
ACTION_2 = 2

EVEN = 0
RANDOM = 1

# -----------------
env_size = 19
env_connectivity = .65
# -----------------
seed = 0
sim_id = "01-t3-2"  # !!!!!!!!! Change this for each run
recompute_bids_on_state_change = True
with_interceding = False

gotto_task_count = 150
# no_task_task_count = 15
no_task_task_count = 30
action_1_task_count = 60     # action_1 task     (even goal ids)
action_2_task_count = 60     # action_2 tasks    (odd goal ids)

initial_tasks_announcement = 10

# gotto_task_count = 10
# no_task_task_count = 0
# action_1_task_count = 5     # action_1 task     (even goal ids)
# action_2_task_count = 5     # action_2 tasks    (odd goal ids)
#
# initial_tasks_announcement = 10

release_spread = EVEN
release_ration = EVEN

release_max_epoch = 200

assert gotto_task_count == (no_task_task_count + action_1_task_count + action_2_task_count)

# agent_lst = ["Turtle_1", "Turtle_2", "Turtle_3", "Turtle_4", "Turtle_5", "Turtle_6", "Turtle_7", "Turtle_8", "Turtle_9", "Turtle_10"]
# agent_lst = ["Operator", "Turtle_2", "Turtle_3", "Turtle_4"]
agent_lst = ["Turtle_1", "Turtle_2", "Turtle_3", "Turtle_4"]
# agent_lst = ["Turtle_1", "Turtle_2"]

# intercession_targets = []
# intercession_targets = ["ACTION_1"]
intercession_targets = ["ACTION_1", "ACTION_2"]

skillsets = {
    "Turtle_1": ["GOTO", "ACTION_1"],
    "Turtle_2": ["GOTO", "ACTION_1"],
    "Turtle_3": ["GOTO", "ACTION_2"],
    "Turtle_4": ["GOTO", "ACTION_2"],
}

bid_function = {
    "Turtle_1": "anticipated_action_task_interceding_agent" if with_interceding else "graph_weighted_manhattan_distance_bid",
    "Turtle_2": "graph_weighted_manhattan_distance_bid",
    "Turtle_3": "graph_weighted_manhattan_distance_bid",
    "Turtle_4": "graph_weighted_manhattan_distance_bid",
    # "Turtle_4": "graph_weighted_manhattan_distance_bid",
    # "Turtle_5": "graph_weighted_manhattan_distance_bid",
    # "Turtle_6": "graph_weighted_manhattan_distance_bid",
    # "Turtle_7": "graph_weighted_manhattan_distance_bid",
    # "Turtle_8": "graph_weighted_manhattan_distance_bid",
    # "Turtle_9": "graph_weighted_manhattan_distance_bid",
    # "Turtle_10": "graph_weighted_manhattan_distance_bid"
}
