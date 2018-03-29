# This message is part of the IHMC whole-body controller API.
# This message pauses the execution of a list of footsteps.
# If this message is in the middle of executing a footstep, the robot will finish the step and pause when back in double support.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# True to pause walking, false to unpause and resume an existing footstep plan.
 bool pause