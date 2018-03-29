# This message is part of the IHMC whole-body controller API.
# This message commands the controller to start loading a foot that was unloaded to support the robot weight. 
# When the robot is performing a 'flamingo stance' (one foot in the air not actually walking) and the user wants the robot to switch back to double support.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte LOAD_BEARING_REQUEST_LOAD = 0
byte LOAD_BEARING_REQUEST_UNLOAD = 1

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# Needed to identify a side dependent end-effector.
byte robot_side 255
# Whether the end-effector should be loaded or unloaded.
byte load_bearing_request
# The time to delay this command on the controller side before being executed.
float64 execution_delay_time