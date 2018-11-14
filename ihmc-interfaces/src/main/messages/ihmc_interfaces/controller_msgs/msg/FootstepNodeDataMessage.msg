# This message is part of the IHMC footstep planning module.
# This message is sent from the planner in order to visualize and debug planner progress

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Footstep node side
byte robot_side 255

int32 x_index
int32 y_index
int32 yaw_index

# Specifies snap transform translation. NaN if snap was unsuccessful
geometry_msgs/Point snap_translation
# Specifies the snap rotation. NaN if snap unsuccessful.
geometry_msgs/Quaternion snap_rotation

# ID of parent node. This should reference the index of this node's parent in a FootstepNodeDataList
int32 parent_node_id -1

# Node rejection reasons. See BipedalFootstepPlannerNodeRejectionReason
byte STEP_TOO_HIGH_OR_LOW = 0
byte STEP_TOO_FORWARD_AND_DOWN = 1
byte STEP_TOO_FAR = 2
byte STEP_TOO_FAR_AND_HIGH = 3
byte STEP_NOT_WIDE_ENOUGH = 4
byte STEP_IN_PLACE = 5
byte NOT_ENOUGH_AREA = 6
byte COULD_NOT_SNAP = 7
byte COULD_NOT_WIGGLE_INSIDE = 8
byte SURFACE_NORMAL_TOO_STEEP_TO_SNAP = 9
byte TOO_MUCH_PENETRATION_AFTER_WIGGLE = 10
byte STEP_NOT_LONG_ENOUGH = 11
byte STEP_TOO_WIDE = 12
byte OBSTACLE_BLOCKING_BODY = 13
byte OBSTACLE_HITTING_BODY = 14
byte AT_CLIFF_BOTTOM = 15

# Node rejection reason. 255 if node was accepted
byte bipedal_footstep_planner_node_rejection_reason 255
