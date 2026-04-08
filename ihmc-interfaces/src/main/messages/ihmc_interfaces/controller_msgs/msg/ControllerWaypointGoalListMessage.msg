# Input message to queue a sequence of goals for the PDVelocityBasedGoalReacher.
# Each waypoint is consumed in order; the hold_position flag of the last waypoint
# determines whether the robot holds position at the final goal.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# Ordered list of goals to execute in sequence.
ControllerWaypointGoalMessage[<=50] waypoints
