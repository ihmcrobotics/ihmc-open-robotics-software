# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move in taskspace the pelvis to the desired orientation while going through the specified trajectory points.
# A Hermite based curve (third order) is used to interpolate the orientations.
# This message allows controlling the pelvis orientation without interfering with position that will still be controlled to maintain the current desired capture point position.
# To execute a normal trajectory to reach a desired pelvis orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When the robot is walking, restrictions on upper-body motion may be applied.
# To by-pass the safety check and force the execution of this message, set this field to true.
bool force_execution false

# Whether the pelvis orientation is allowed to be controlled by the user when the robot is walking.
bool enable_user_pelvis_control_during_walking false
# The orientation trajectory information.
SO3TrajectoryMessage so3_trajectory