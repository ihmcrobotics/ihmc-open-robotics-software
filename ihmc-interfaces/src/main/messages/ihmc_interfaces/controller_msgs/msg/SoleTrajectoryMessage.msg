# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move in taskspace a foot to the desired position  while going through the specified trajectory points.
# To execute a single straight line trajectory to reach a desired foot pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.

byte FRONT_LEFT = 0
byte FRONT_RIGHT = 1
byte HIND_RIGHT = 2
byte HIND_LEFT = 3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which foot will execute the trajectory.
byte robot_quadrant 255
# The position trajectory information.
EuclideanTrajectoryMessage position_trajectory