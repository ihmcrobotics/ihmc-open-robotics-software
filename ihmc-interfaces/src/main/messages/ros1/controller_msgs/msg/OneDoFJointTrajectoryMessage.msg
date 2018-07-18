# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The list of trajectory points to go through while executing the trajectory.
# The time has to be strictly increasing.
controller_msgs/TrajectoryPoint1DMessage[] trajectory_points

# Weight used to encode the priority for achieving this trajectory:
# - if too low, in the event the controller can't achieve all of the objectives it may lower the trajectory tracking quality.
# - if too high, the controller will favor this trajectory over other objectives.
# - if set to NaN or to a negative value, the controller will use the default weight for that trajectory.
# The priority of this trajectory is determined from the relative weight of this trajectory and the weight of the other objectives.
# Field default value -1.0
float64 weight


