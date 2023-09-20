# This is a message that holds data to describe the state and control trajectories.
# 
# The trajectory is defined by:
# * the time interval,
# * the state, and
# * the control for each node.
# 
# The dimension of the intervals, state and control trajectories must be equal.
# The header specifies the time at which the message is published.

# This represents the time and frame at which the message is published
Header header

# This represents the time and duration of each interval
controller_msgs/CrocoddylTimeIntervalMessage[] intervals

# This represents the state for each interval
controller_msgs/CrocoddylStateMessage[] state_trajectory

# This represents the control for each interval
controller_msgs/CrocoddylControlMessage[] control_trajectory


