# Message for commanding the Sake hands to perform various predefined grasps.
# Also allows for custom grasps with set positions/torques

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte HAND_CONFIGURATION_CALIBRATE = 0
byte HAND_CONFIGURATION_RESET = 1
byte HAND_CONFIGURATION_OPEN = 2
byte HAND_CONFIGURATION_CLOSE = 3
byte HAND_CONFIGURATION_RELEASE = 4
byte HAND_CONFIGURATION_GOTO_POSITION_WITH_TORQUE = 5
byte HAND_CONFIGURATION_GRIP_WITH_TORQUE = 6
byte HAND_CONFIGURATION_GRIP_HARD = 7

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory
byte robot_side 255
# Specifies the grasp to perform
byte desired_hand_configuration 255
# 0.0 is closed, 1.0 is open
float64 postion_ratio

# Specifies desired torque of grasp, if not specified by hand configuration. 0.0 min, 1.0 max
float64 torque_ratio
