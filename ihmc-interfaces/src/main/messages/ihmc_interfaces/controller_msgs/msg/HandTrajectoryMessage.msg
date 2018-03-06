# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move in taskspace a hand to the desired pose (position & orientation) while going through the specified trajectory points.
# A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations.
# To execute a single straight line trajectory to reach a desired hand pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Specifies which hand will execute the trajectory.
byte robot_side -1
# The position/orientation trajectory information.
SE3TrajectoryMessage se3_trajectory