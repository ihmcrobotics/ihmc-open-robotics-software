# This message is part of the IHMC quadruped controller API.
# This message commands the controller to start loading a foot.
# This message will only load the foot when the robot is in Sole Waypoint mode, which is triggered by a SoleTrajectoryMessage
# If the robot is walking, the message is ignored

byte FRONT_LEFT = 0
byte FRONT_RIGHT = 1
byte HIND_RIGHT = 2
byte HIND_LEFT = 3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which quadrant should be loaded.
byte robot_quadrant 255
# The time to delay this command on the controller side before being executed.
float64 execution_delay_time