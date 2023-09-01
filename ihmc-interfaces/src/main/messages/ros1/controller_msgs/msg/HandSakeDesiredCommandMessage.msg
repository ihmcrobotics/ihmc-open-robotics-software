# Message for commanding the Sake hands to perform various predefined grasps.
# Also allows for custom grasps with set positions/torques

int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

int8 HAND_CONFIGURATION_CALIBRATE=0

int8 HAND_CONFIGURATION_RESET=1

int8 HAND_CONFIGURATION_OPEN=2

int8 HAND_CONFIGURATION_CLOSE=3

int8 HAND_CONFIGURATION_RELEASE=4

int8 HAND_CONFIGURATION_GOTO_POSITION_WITH_TORQUE=5

int8 HAND_CONFIGURATION_GRIP_WITH_TORQUE=6

int8 HAND_CONFIGURATION_GRIP_HARD=7

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory
# Field default value 255
int8 robot_side

# Specifies the grasp to perform
# Field default value 255
int8 desired_hand_configuration

# 0.0 is closed, 1.0 is open
float64 postion_ratio

# Specifies desired torque of grasp, if not specified by hand configuration. 0.0 min, 1.0 max
float64 torque_ratio


