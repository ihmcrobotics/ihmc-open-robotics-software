# This message is part of the IHMC whole-body controller API.
# This message pauses the execution of a list of footsteps.
# If this message is in the middle of executing a footstep, the robot will finish the step and pause when back in double support.

# True to pause walking, false to unpause and resume an existing footstep plan.
 bool pause