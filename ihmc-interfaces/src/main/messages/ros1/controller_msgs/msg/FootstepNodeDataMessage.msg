int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Node rejection reasons. See BipedalFootstepPlannerNodeRejectionReason
int8 STEP_TOO_HIGH_OR_LOW=0

int8 STEP_TOO_FORWARD_AND_DOWN=1

int8 STEP_TOO_FAR=2

int8 STEP_TOO_FAR_AND_HIGH=3

int8 STEP_NOT_WIDE_ENOUGH=4

int8 STEP_IN_PLACE=5

int8 NOT_ENOUGH_AREA=6

int8 COULD_NOT_SNAP=7

int8 COULD_NOT_WIGGLE_INSIDE=8

int8 SURFACE_NORMAL_TOO_STEEP_TO_SNAP=9

int8 TOO_MUCH_PENETRATION_AFTER_WIGGLE=10

int8 STEP_NOT_LONG_ENOUGH=11

int8 STEP_TOO_WIDE=12

int8 OBSTACLE_BLOCKING_BODY=13

int8 OBSTACLE_HITTING_BODY=14

int8 AT_CLIFF_BOTTOM=15

# Footstep node side
# Field default value 255
int8 robot_side

int32 x_index

int32 y_index

int32 yaw_index

# Specifies snap transform translation. NaN if snap was unsuccessful
geometry_msgs/Point snap_translation

# Specifies the snap rotation. NaN if snap unsuccessful.
geometry_msgs/Quaternion snap_rotation

# ID of parent node. This should reference the index of this node's parent in a FootstepNodeDataList
# Field default value -1
int32 parent_node_id

# Node rejection reason. 255 if node was accepted
# Field default value 255
int8 bipedal_footstep_planner_node_rejection_reason


