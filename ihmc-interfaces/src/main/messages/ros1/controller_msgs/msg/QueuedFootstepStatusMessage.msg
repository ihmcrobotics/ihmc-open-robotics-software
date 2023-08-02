# This message is part of the IHMC whole-body controller API.
# This message specifies the position, orientation and side (left or right) of a footstep in the queue in world frame.

int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which foot will swing to reach the footstep.
# Field default value 255
int8 robot_side

# Specifies the position of the footstep (sole frame) in world frame.
geometry_msgs/Point location

# Specifies the orientation of the footstep (sole frame) in world frame.
geometry_msgs/Quaternion orientation

# Predicted contact points represent the vertices of the expected contact polygon between the foot and the world.
# An empty list will request the controller to use the default foot support polygon.
# Contact points  are expressed in sole frame. The ordering does not matter.
# For example: to tell the controller to use the entire foot, the predicted contact points would be:
# - x: 0.5 * foot_length, y: -0.5 * toe_width
# - x: 0.5 * foot_length, y: 0.5 * toe_width
# - x: -0.5 * foot_length, y: -0.5 * heel_width
# - x: -0.5 * foot_length, y: 0.5 * heel_width
# Note: The z coordinate of each point is ignored.
geometry_msgs/Point[] predicted_contact_points_2d

# The swingDuration is the time a foot is not in ground contact during a step.
# If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
# Field default value -1.0
float64 swing_duration

# The transferDuration is the time spent with the feet in ground contact before a step.
# If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
# Field default value -1.0
float64 transfer_duration

# Step constraint regions to be used by this footstep
controller_msgs/StepConstraintsListMessage step_constraints


