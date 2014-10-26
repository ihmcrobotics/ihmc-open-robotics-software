## FootstepDataMessage
# This message gives the position, orientation and side of a single footstep to the controller.
# If sending a single footstep, position and orientation should be in pelvis frame.

#Options for enum
# uint8 L = 0
# uint8 R = 1
uint8 robotSide
geometry_msgs/Vector3 location
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3[] predictedContactPoints

