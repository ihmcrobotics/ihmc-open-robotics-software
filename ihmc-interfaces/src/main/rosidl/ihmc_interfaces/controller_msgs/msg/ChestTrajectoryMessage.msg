# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move in taskspace the chest to the desired orientation while going through the specified trajectory points.
# A Hermite based curve (third order) is used to interpolate the orientations.
# To execute a simple trajectory to reach a desired chest orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.

# The orientation trajectory information.
SO3TrajectoryMessage so3_trajectory