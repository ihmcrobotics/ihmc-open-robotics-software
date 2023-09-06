# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move in taskspace a forearm to the desired orientation while going through the specified trajectory points.
# A Hermite based curve (third order) is used to interpolate the orientations.
# To execute a simple trajectory to reach a desired forearm orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When the robot is walking, restrictions on upper-body motion may be applied.
# To by-pass the safety check and force the execution of this message, set this field to true.
bool force_execution false

# Specifies which forearm will execute the trajectory.
byte robot_side 255
# The orientation trajectory information.
ihmc_common_msgs/SO3TrajectoryMessage so3_trajectory