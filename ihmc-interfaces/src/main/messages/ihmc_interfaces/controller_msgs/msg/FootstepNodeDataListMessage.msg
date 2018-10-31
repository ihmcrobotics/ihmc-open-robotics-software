# This message is part of the IHMC footstep planning module.
# This message is sent from the planner in order to visualize and debug planner progress

# This is the ID of the planning request which this message corresponds to
uint32 sequence_id

# List of all footstep node data. Index 0 should hold the start node
FootstepNodeDataMessage[] node_data

# This message has two modes:
# Mode 1: The planner will periodically emit a list of nodes representing the current lowest cost path while planning.
#      The interpretation of lower cost path depends on the planner, but usually means the sequence of nodes with the lowest ((cost from start) + heuristic) value.
#      In this case, each of the node_data elements will not have the rejection reason or parent id fields populated
# Mode 2: The planner can emit the entire footstep graph, with rejection reasons and parents indices populated.
#      This is emitted once after planning has completed.
# If this value is true, this packet is in mode 2
bool is_footstep_graph false
