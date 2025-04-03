def lazy_import_GraphWeightedManhattanDistanceBundleBid():
    try:
        from maaf_allocation_node.allocation_logics.CBBA.bidding_logics.GraphWeightedManhattanDistanceBundleBid import GraphWeightedManhattanDistanceBundleBid
    except ImportError:
        from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBBA.bidding_logics.GraphWeightedManhattanDistanceBundleBid import GraphWeightedManhattanDistanceBundleBid
    return GraphWeightedManhattanDistanceBundleBid().compute_bids


# def lazy_import_interceding_skill_based_bid_amplifier():
#     try:
#         from maaf_allocation_node.allocation_logics.CBBA.bidding_logics.interceding_skill_based_bid_amplifier import interceding_skill_based_bid_amplifier
#     except ImportError:
#         from maaf_allocation_node.maaf_allocation_node.allocation_logics.CBBA.bidding_logics.interceding_skill_based_bid_amplifier import interceding_skill_based_bid_amplifier
#     return interceding_skill_based_bid_amplifier().bidding_logic


bidding_logics_dict = {
    "GraphWeightedManhattanDistanceBundleBid": lazy_import_GraphWeightedManhattanDistanceBundleBid,
    #"interceding_skill_based_bid_amplifier": lazy_import_interceding_skill_based_bid_amplifier
}
