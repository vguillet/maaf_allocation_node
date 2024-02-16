TeamCommStamped.msg {
            # > Metadata
            "builtin_interfaces/Time stamp": timestamp,
            "memo": msg memo,
            "trace": [agent ids]
            ------------------------------
            # > Tracker
            "source": source agent id,
            "target": target agent id,
            "meta_action": msg type, (allocation update, task update, goal assignment, etc.),
            }