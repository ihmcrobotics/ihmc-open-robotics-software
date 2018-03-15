# This message is part of the IHMC whole-body controller API.
# The message commands the controller to bring the given part of the body back to a default configuration called 'home'.
# It is useful to get back to a safe configuration before walking.

byte HUMANOID_BODY_PART_ARM = 0
byte HUMANOID_BODY_PART_CHEST = 1
byte HUMANOID_BODY_PART_PELVIS = 2

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Specifies the part of the body the use wants to move back to its home configuration.
byte humanoid_body_part 255
# Needed to identify a side dependent end-effector.
byte robot_side 255
# How long the trajectory will spline from the current desired to the home configuration.
float64 trajectory_time
# The time to delay this command on the controller side before being executed.
float64 execution_delay_time 0.0