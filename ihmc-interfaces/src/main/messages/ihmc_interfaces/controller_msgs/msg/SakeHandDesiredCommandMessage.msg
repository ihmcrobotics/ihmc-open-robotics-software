# Message for commanding the Sake hands to perform various predefined grasps.
# Also allows for custom grasps with set positions/torques

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte COMMAND_OPTION_OPEN = 0
byte COMMAND_OPTION_CLOSE = 1
byte COMMAND_OPTION_GRIP = 2
byte COMMAND_OPTION_CUSTOM = 3
byte COMMAND_OPTION_CALIBRATE = 4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory
byte robot_side 255
# Specifies the grasp to perform
byte desired_command_option 255
# Set to true when user confirms an error
bool error_confirmation

# 1.0 is closed, 0.0 is open
float64 position_ratio
# Specifies desired torque of grasp, if not specified by hand configuration. 0.0 min, 1.0 max
float64 torque_ratio
