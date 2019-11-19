# This message is part of the IHMC footstep planning module.
# This message is sent from the planner in order to visualize and debug planner progress

# This is the ID of the planning request which this message corresponds to
uint32 plan_id

# List of cells that the planner has explored
FootstepPlannerLatticeNodeMessage[<=10000] lattice_nodes
