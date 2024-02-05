# This message is part of the IHMC whole-body controller API.
# Request the controller to use a hand to help supporting the robot weight.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The robot side of the hand that will be load bearing.
byte robot_side 255
# If true it will load the contact point, otherwise the hand will stop bearing load.
bool load false
# Sets the coefficient of friction that the controller will use for the contact point (only used if load=true).
float64 coefficient_of_friction 0
# Hand contact point expressed in the hand's body-fixed frame (only used if load=true).
geometry_msgs/Point contact_point_in_body_frame
# Contact normal in world frame, pointing away from the environment (only used if load=true).
geometry_msgs/Vector3 contact_normal_in_world
# The time to delay this command on the controller side before being executed.
float64 execution_delay_time