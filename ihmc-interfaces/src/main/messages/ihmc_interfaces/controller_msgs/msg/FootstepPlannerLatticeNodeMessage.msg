# This message is part of the IHMC footstep planning module.
# This message is sent from the planner in order to visualize and debug planner progress

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# X index of the cell. See us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode for more information
int32 x_index
# Y index of the cell. See us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode for more information
int32 y_index
# Yaw index of the cell
int32 yaw_index
# Robot side of the cell
byte robot_side

int32 node_index
