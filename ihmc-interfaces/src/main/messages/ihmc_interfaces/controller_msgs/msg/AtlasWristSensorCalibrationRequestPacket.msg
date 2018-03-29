# Atlas specific message: request taring of the wrist force/torque sensors.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# The robot side (left or right) for the wrist sensor you would like to request calibration for.
byte robot_side 255
