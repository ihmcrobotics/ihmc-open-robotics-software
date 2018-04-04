# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move in taskspace the body to the desired orientation while going through the specified trajectory points.
# A Hermite based curve (third order) is used to interpolate the orientations.
# This message allows controlling the body orientation without interfering with position that will still be controlled to maintain the current desired capture point position.
# To execute a normal trajectory to reach a desired body orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The orientation trajectory information.
SO3TrajectoryMessage so3_trajectory