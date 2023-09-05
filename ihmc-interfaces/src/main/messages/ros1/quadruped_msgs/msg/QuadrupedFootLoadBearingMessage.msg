# This message is part of the IHMC quadruped controller API.
# This message commands the controller to start loading a foot.
# This message will only load the foot when the robot is in Sole Waypoint mode, which is triggered by a SoleTrajectoryMessage
# If the robot is walking, the message is ignored

int8 FRONT_LEFT=0

int8 FRONT_RIGHT=1

int8 HIND_RIGHT=2

int8 HIND_LEFT=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which quadrant should be loaded.
# Field default value 255
int8 robot_quadrant

# The time to delay this command on the controller side before being executed.
float64 execution_delay_time


