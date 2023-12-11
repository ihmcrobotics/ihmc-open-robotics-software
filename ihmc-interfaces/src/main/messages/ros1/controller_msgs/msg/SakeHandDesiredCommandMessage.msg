# Message for commanding the Sake hands to perform various predefined grasps.
# Also allows for custom grasps with set positions/torques

int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

int8 COMMAND_OPTION_OPEN=0

int8 COMMAND_OPTION_CLOSE=1

int8 COMMAND_OPTION_GRIP=2

int8 COMMAND_OPTION_CUSTOM=3

int8 COMMAND_OPTION_CALIBRATE=4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory
# Field default value 255
int8 robot_side

# Specifies the grasp to perform
# Field default value 255
int8 desired_command_option

# Set to true when user confirms an error
bool error_confirmation

# 1.0 is closed, 0.0 is open
float64 position_ratio

# Specifies desired torque of grasp, if not specified by hand configuration. 0.0 min, 1.0 max
float64 torque_ratio


