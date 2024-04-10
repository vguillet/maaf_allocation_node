
def _update_decision(k_agent_id: str,
                     k_winning_agent_id: str,
                     k_winning_bid_y_kj,
                     k_timestamp_matrix,

                     i_agent_id: str,
                     i_winning_agent_id: str,
                     i_winning_bid_y_ij,
                     i_timestamp_matrix
                     ) -> str:
    """
    Generate an update decision

    :param k_agent_id: The agent k that sent the message
    :param k_winning_agent_id: The agent that won the task in k's allocation
    :param k_winning_bid_y_kj: The winning bid in k's allocation
    :param k_timestamp_matrix: The timestamp matrix of agent k

    :param i_agent_id: The agent i that received the message
    :param i_winning_agent_id: The agent that won the task in i's allocation
    :param i_winning_bid_y_ij: The winning bid in i's allocation
    :param i_timestamp_matrix: The timestamp matrix of agent i

    :return: The decision to update, reset, or leave the task
    """

    # self.get_logger().info(f"Agent {k_agent_id} thinks agent {k_winning_agent_id} won the task")
    # self.get_logger().info(f"Agent {i_agent_id} thinks agent {i_winning_agent_id} won the task")

    # -> k agent beliefs
    k_thinks_agent_k_won = k_winning_agent_id == k_agent_id
    k_thinks_agent_i_won = k_winning_agent_id == i_agent_id
    k_thinks_agent_m_won = k_winning_agent_id not in [k_agent_id, i_agent_id] and k_winning_agent_id != ""
    k_thinks_task_unassigned = k_winning_agent_id == ""

    # > Verify that only one is true
    assert sum([k_thinks_agent_k_won, k_thinks_agent_i_won, k_thinks_agent_m_won, k_thinks_task_unassigned]) == 1

    # print(f"\n"
    #       f"k_thinks_agent_k_won: {k_thinks_agent_k_won}\n"
    #       f"k_thinks_agent_i_won: {k_thinks_agent_i_won}\n"
    #       f"k_thinks_agent_m_won: {k_thinks_agent_m_won}\n"
    #       f"k_thinks_task_unassigned: {k_thinks_task_unassigned}"
    #       )

    # -> i agent beliefs
    i_thinks_agent_i_won = i_winning_agent_id == i_agent_id
    i_thinks_agent_k_won = i_winning_agent_id == k_agent_id

    if k_thinks_agent_k_won or k_thinks_agent_i_won or k_thinks_task_unassigned:
        i_thinks_agent_m_won = i_winning_agent_id not in [i_agent_id, k_agent_id] and i_winning_agent_id != ""
        i_thinks_agent_n_won = False
    else:
        i_thinks_agent_m_won = i_winning_agent_id not in [i_agent_id, k_agent_id] and i_winning_agent_id != "" and i_winning_agent_id == k_winning_agent_id
        i_thinks_agent_n_won = i_winning_agent_id not in [i_agent_id, k_agent_id, k_winning_agent_id] and i_winning_agent_id != ""

    i_thinks_task_unassigned = i_winning_agent_id == ""

    # > Verify that only one is true
    assert sum([i_thinks_agent_i_won, i_thinks_agent_k_won, i_thinks_agent_m_won, i_thinks_agent_n_won,
                i_thinks_task_unassigned]) == 1

    # print(f"\n"
    #       f"i_thinks_agent_i_won: {i_thinks_agent_i_won}\n"
    #       f"i_thinks_agent_k_won: {i_thinks_agent_k_won}\n"
    #       f"i_thinks_agent_m_won: {i_thinks_agent_m_won}\n"
    #       f"i_thinks_agent_n_won: {i_thinks_agent_n_won}\n"
    #       f"i_thinks_task_unassigned: {i_thinks_task_unassigned}"
    #       )

    # --------------------------------------------------------- Group 1
    # > Sender thinks they won the task
    if k_thinks_agent_k_won:

        # > Local agent thinks they won the task
        if i_thinks_agent_i_won:
            # > If the k has a higher bid: update
            if k_winning_bid_y_kj > i_winning_bid_y_ij:
                return 'update'

            # > Default to leave
            else:
                return 'leave'

        # > Local agent thinks k won the task: update
        elif i_thinks_agent_k_won:
            return 'update'

        # > Local agent thinks someone else won the task
        elif i_thinks_agent_m_won:

            # > If the k bid is more recent or higher: update
            if (k_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[
                i_winning_agent_id, "last_update_s"]
                    or k_winning_bid_y_kj > i_winning_bid_y_ij):
                return 'update'

            # > Default to leave
            else:
                return 'leave'

        # > Local agent thinks the task is unassigned: update
        elif i_thinks_task_unassigned:
            return 'update'

        else:
            raise ValueError("Invalid state A")

    # --------------------------------------------------------- Group 2
    # > Sender thinks i agent won the task
    elif k_thinks_agent_i_won:

        # > Local agent thinks they won the task: leave
        if i_thinks_agent_i_won:
            return 'leave'

        # > Local agent thinks k won the task: reset
        elif i_thinks_agent_k_won:
            return 'reset'

        # > Local agent thinks someone else won the task
        elif i_thinks_agent_m_won:
            # > If the k bid is more recent: reset
            if k_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"]:
                return 'reset'

            # > Default to leave
            else:
                return 'leave'

        # > Local agent thinks the task is unassigned: leave
        elif i_thinks_task_unassigned:
            return 'leave'

        else:
            raise ValueError("Invalid state B")

    # --------------------------------------------------------- Group 3
    # > Sender thinks someone else won the task
    elif k_thinks_agent_m_won:

        # > Local agent thinks they won the task
        if i_thinks_agent_i_won:

            # > If the k bid is more recent and higher: update
            if (k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[
                k_winning_agent_id, "last_update_s"]
                    and k_winning_bid_y_kj > i_winning_bid_y_ij):
                return 'update'

            # > Default to leave
            else:
                return 'leave'

        # > Local agent thinks k won the task
        elif i_thinks_agent_k_won:

            # > If the k bid is more recent: update
            if k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[
                k_winning_agent_id, "last_update_s"]:
                return 'update'

            # > Else: reset
            else:
                return 'reset'

        # > Local agent also thinks the same agent won the task
        elif i_thinks_agent_m_won:

            # > If the k bid is more recent and higher: update
            if k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[
                i_winning_agent_id, "last_update_s"]:
                return 'update'

            # > Default to leave
            else:
                return 'leave'

        # > Local agent thinks someone else won the task, and it is not the same as the k's winning agent
        elif i_thinks_agent_n_won:

            # > If the k bid is more recent both ways: update
            if (k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[
                k_winning_agent_id, "last_update_s"]
                    and k_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[
                        i_winning_agent_id, "last_update_s"]):
                return 'update'

            # > If the k bid is more recent and higher: update
            elif (k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[
                k_winning_agent_id, "last_update_s"]
                  and k_winning_bid_y_kj > i_winning_bid_y_ij):
                return 'update'

            # > If the bids both are more recent crosswise: reset
            elif (k_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[
                i_winning_agent_id, "last_update_s"]
                  and i_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > k_timestamp_matrix.loc[
                      k_winning_agent_id, "last_update_s"]):
                return 'reset'

            # > Default to leave
            else:
                return 'leave'

        # > Local agent thinks the task is unassigned
        elif i_thinks_task_unassigned:
            if k_timestamp_matrix.loc[k_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[
                k_winning_agent_id, "last_update_s"]:
                return 'update'

            # > Default to leave
            else:
                return 'leave'

        else:
            raise ValueError("Invalid state C")

    # --------------------------------------------------------- Group 4
    # > Sender thinks the task is unassigned
    elif k_thinks_task_unassigned:

        # > Local agent thinks they won the task: leave
        if i_thinks_agent_i_won:
            return 'leave'

        # > Local agent thinks k won the task: update
        elif i_thinks_agent_k_won:
            return 'update'

        # > Local agent thinks someone else won the task
        elif i_thinks_agent_m_won:
            # > If the k bid is more recent: update
            if k_timestamp_matrix.loc[i_winning_agent_id, "last_update_s"] > i_timestamp_matrix.loc[
                i_winning_agent_id, "last_update_s"]:
                return 'update'

            # > Default to leave
            else:
                return 'leave'

        # > Local agent thinks the task is unassigned
        elif i_thinks_task_unassigned:
            return 'leave'

        else:
            raise ValueError("Invalid state D")

    else:
        raise ValueError("Invalid state E")


# =================================== Unit test decisions
if __name__ == '__main__':
    import pandas as pd

    total_passed = 0
    total_failed = 0

    print(f"------------- Group 1: Agent k thinks k is the winner")
    """
    ----- Case 1: 
    - Agent i thinks i won the task
    """
    print(f"Case 1:")
    
    """
    1a:
    - Agent k bid is higher
    Expected: Update
    """
    
    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="k",
        k_winning_bid_y_kj=10,
        k_timestamp_matrix=None,
        i_agent_id="i",
        i_winning_agent_id="i",
        i_winning_bid_y_ij=5,
        i_timestamp_matrix=None
    )

    try:
        assert decision == 'update'
        print(f"  [v] Case 1a passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 1a failed: {decision}")
        total_failed += 1

    """
    1b:
    - Agent k bid is lower
    Expected: Leave
    """
    
    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="k",
        k_winning_bid_y_kj=5,
        k_timestamp_matrix=None,
        i_agent_id="i",
        i_winning_agent_id="i",
        i_winning_bid_y_ij=10,
        i_timestamp_matrix=None
    )
    
    try:
        assert decision == 'leave'
        print(f"  [v] Case 1b passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 1b failed: {decision}")
        total_failed += 1

    """
    ----- Case 2:
    - Agent i thinks k won the task
    Expected: Update
    """
    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="k",
        k_winning_bid_y_kj=10,
        k_timestamp_matrix=None,
        i_agent_id="i",
        i_winning_agent_id="k",
        i_winning_bid_y_ij=5,
        i_timestamp_matrix=None
    )

    try:
        assert decision == 'update'
        print(f"[v] Case 2 passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"[x] Case 2 failed: {decision}")
        total_failed += 1
    
    """
    ----- Case 3:
    - Agent i thinks m won the task
    """
    print(f"Case 3:")

    """
    3a:
    - Agent k m bid is more recent and higher k bid
    Expected: Update
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="k",
        k_winning_bid_y_kj=10,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="m",
        i_winning_bid_y_ij=5,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'update'
        print(f"  [v] Case 3a passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 3a failed: {decision}")
        total_failed += 1

    """
    3b:
    - Agent k m bid is more recent but lower k bid
    Expected: Update
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="k",
        k_winning_bid_y_kj=5,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="m",
        i_winning_bid_y_ij=10,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'update'
        print(f"  [v] Case 3b passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 3b failed: {decision}")
        total_failed += 1

    """
    3c:
    - Agent k m bid is less recent but higher k bid
    Expected: Update
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="k",
        k_winning_bid_y_kj=10,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="m",
        i_winning_bid_y_ij=5,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'update'
        print(f"  [v] Case 3c passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 3c failed: {decision}")
        total_failed += 1

    """
    3d:
    - Agent k m bid is less recent and lower k bid
    Expected: Leave    
    """

    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="k",
        k_winning_bid_y_kj=5,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="m",
        i_winning_bid_y_ij=10,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'leave'
        print(f"  [v] Case 3d passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 3d failed: {decision}")
        total_failed += 1

    """
    ----- Case 4:
    - Agent i thinks the task is unassigned
    Expected: Update
    """
    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="k",
        k_winning_bid_y_kj=10,
        k_timestamp_matrix=None,
        i_agent_id="i",
        i_winning_agent_id="",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=None
    )

    try:
        assert decision == 'update'
        print(f"[v] Case 4 passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"[x] Case 4 failed: {decision}")
        total_failed += 1

    print(f"------------- Group 2: Agent k thinks i is the winner")
    """
    ----- Case 5:
    - Agent i thinks i won the task
    Expected: Leave
    """
    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="i",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=None,
        i_agent_id="i",
        i_winning_agent_id="i",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=None
    )

    try:
        assert decision == 'leave'
        print(f"[v] Case 5 passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"[x] Case 5 failed: {decision}")
        total_failed += 1

    """
    ----- Case 6:
    - Agent i thinks k won the task
    Expected: Reset
    """
    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="i",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=None,
        i_agent_id="i",
        i_winning_agent_id="k",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=None
    )

    try:
        assert decision == 'reset'
        print(f"[v] Case 6 passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"[x] Case 6 failed: {decision}")
        total_failed += 1

    """
    ----- Case 7:
    - Agent i thinks m won the task
    """
    print(f"Case 7:")

    """
    7a:
    - Agent k m timestamp is more recent
    Expected: reset
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="i",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="m",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'reset'
        print(f"  [v] Case 7a passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 7a failed: {decision}")
        total_failed += 1

    """
    7b:
    - Agent k m timestamp is less recent
    Expected: Leave
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="i",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="m",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'leave'
        print(f"  [v] Case 7b passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 7b failed: {decision}")
        total_failed += 1

    """
    ----- Case 8:
    - Agent i thinks the task is unassigned
    Expected: Leave
    """
    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="i",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=None,
        i_agent_id="i",
        i_winning_agent_id="",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=None
    )

    try:
        assert decision == 'leave'
        print(f"[v] Case 8 passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"[x] Case 8 failed: {decision}")
        total_failed += 1

    print(f"------------- Group 3: Agent k thinks m is the winner")
    """
    ----- Case 9:
    - Agent i thinks they won the task
    """
    print(f"Case 9: Agent i thinks i won the task")

    """
    9a:
    - Agent k m timestamp is more recent and higher k bid
    Expected: Update
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=10,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="i",
        i_winning_bid_y_ij=5,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'update'
        print(f"  [v] Case 9a passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 9a failed: {decision}")
        total_failed += 1

    """
    9b:
    - Agent k m timestamp is more recent but lower k bid
    Expected: Leave
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=5,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="i",
        i_winning_bid_y_ij=10,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'leave'
        print(f"  [v] Case 9b passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 9b failed: {decision}")
        total_failed += 1

    """
    9c:
    - Agent k m timestamp is less recent but higher k bid
    Expected: Leave
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=10,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="i",
        i_winning_bid_y_ij=5,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'leave'
        print(f"  [v] Case 9c passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 9c failed: {decision}")
        total_failed += 1

    """
    9d:
    - Agent k m timestamp is less recent and lower k bid
    Expected: Leave
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=5,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="i",
        i_winning_bid_y_ij=10,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'leave'
        print(f"  [v] Case 9d passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 9d failed: {decision}")
        total_failed += 1

    """
    ----- Case 10:
    - Agent i thinks k won the task
    """
    print(f"Case 10: Agent i thinks k won the task")

    """
    10a:
    - Agent k m timestamp is more recent
    Expected: Update
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="k",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'update'
        print(f"  [v] Case 10a passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 10a failed: {decision}")
        total_failed += 1

    """
    10b:
    - Agent k m timestamp is less recent
    Expected: Reset
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="k",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'reset'
        print(f"  [v] Case 10b passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 10b failed: {decision}")
        total_failed += 1

    """
    ----- Case 11:
    - Agent i thinks m won the task
    """
    print(f"Case 11: Agent i thinks m won the task")

    """
    11a:
    - Agent k m timestamp is more recent
    Expected: Update
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="m",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'update'
        print(f"  [v] Case 11a passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 11a failed: {decision}")
        total_failed += 1

    """
    11b:
    - Agent k m timestamp is less recent
    Expected: Leave
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="m",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'leave'
        print(f"  [v] Case 11b passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 11b failed: {decision}")
        total_failed += 1

    """
    ----- Case 12:
    - Agent i thinks n won the task
    """
    print(f"Case 12: Agent i thinks n won the task")

    """
    12a:
    - Agent k m timestamp is more recent and k n timestamp is more recent
    Expected: Update
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 10}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 5}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="n",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'update'
        print(f"  [v] Case 12a passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 12a failed: {decision}")
        total_failed += 1

    """
    12b:
    - Agent k m timestamp is more recent and larger k bid
    Expected: Update
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 5}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 10}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=10,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="n",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'update'
        print(f"  [v] Case 12b passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 12b failed: {decision}")
        total_failed += 1

    """
    12c:
    - Agent k n timestamp is more recent but k m timestamp is older
    Expected: Reset
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 10}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 5}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="n",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'reset'
        print(f"  [v] Case 12c passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 12c failed: {decision}")
        total_failed += 1

    """
    12d:
    - Agent k n timestamp is less recent but k m timestamp is older
    Expected: Leave
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 5}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 10}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="n",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'leave'
        print(f"  [v] Case 12d passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 12d failed: {decision}")
        total_failed += 1

    """
    ----- Case 13:
    - Agent i thinks the task is unassigned
    """
    print(f"Case 13: Agent i thinks the task is unassigned")

    """
    13a:
    - Agent k m timestamp is more recent
    Expected: Update
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'update'
        print(f"  [v] Case 13a passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 13a failed: {decision}")
        total_failed += 1

    """
    13b:
    - Agent k m timestamp is less recent
    Expected: Leave
    """

    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="m",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'leave'
        print(f"  [v] Case 13b passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 13b failed: {decision}")
        total_failed += 1

    print(f"------------- Group 4: Agent k thinks the task is unassigned")
    """
    ----- Case 14:
    - Agent i thinks i won the task
    Expected: Leave
    """
    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=None,
        i_agent_id="i",
        i_winning_agent_id="i",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=None
    )

    try:
        assert decision == 'leave'
        print(f"[v] Case 14 passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"[x] Case 14 failed: {decision}")
        total_failed += 1

    """
    ----- Case 15:
    - Agent i thinks k won the task
    Expected: Update
    """
    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=None,
        i_agent_id="i",
        i_winning_agent_id="k",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=None
    )

    try:
        assert decision == 'update'
        print(f"[v] Case 15 passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"[x] Case 15 failed: {decision}")
        total_failed += 1

    """
    ----- Case 16:
    - Agent i thinks m won the task
    """
    print(f"Case 16: Agent i thinks m won the task")

    """
    16a:
    - Agent k m timestamp is more recent
    Expected: Update
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="m",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'update'
        print(f"  [v] Case 16a passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 16a failed: {decision}")
        total_failed += 1

    """
    16b:
    - Agent k m timestamp is less recent
    Expected: Leave
    """
    k_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 5, "n": 0}}
    i_timestamp_matrix = {"last_update_s": {"i": 0, "k": 0, "m": 10, "n": 0}}

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=pd.DataFrame(k_timestamp_matrix),
        i_agent_id="i",
        i_winning_agent_id="m",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=pd.DataFrame(i_timestamp_matrix)
    )

    try:
        assert decision == 'leave'
        print(f"  [v] Case 16b passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"  [x] Case 16b failed: {decision}")
        total_failed += 1

    """
    ----- Case 17:
    - Agent i thinks the task is unassigned
    Expected: Leave
    """

    decision = _update_decision(
        k_agent_id="k",
        k_winning_agent_id="",
        k_winning_bid_y_kj=0,
        k_timestamp_matrix=None,
        i_agent_id="i",
        i_winning_agent_id="",
        i_winning_bid_y_ij=0,
        i_timestamp_matrix=None
    )

    try:
        assert decision == 'leave'
        print(f"[v] Case 17 passed: {decision}")
        total_passed += 1
    except AssertionError:
        print(f"[x] Case 17 failed: {decision}")
        total_failed += 1

    print(f"=================================")
    print(f"Total passed: {total_passed}/{total_passed + total_failed}")
    print(f"Total failed: {total_failed}/{total_passed + total_failed}")
