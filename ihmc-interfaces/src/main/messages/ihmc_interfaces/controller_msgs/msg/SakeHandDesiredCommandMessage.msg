# Message for commanding the Sake hands to perform various predefined grasps.
# Also allows for custom grasps with set positions/torques

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Specifies the side of the robot of the hand being referred to
byte robot_side 255

# Request the gripper to perform a calibration sequence
bool request_calibration

# Request to reset the gripper error state after overheating
bool request_reset_errors

# The desired dynamixel position, in radians
float64 gripper_desired_position

# The dynamixel torque limit setting in achieving the desired position
# 0: dynamixel will not apply any force and will not achieve desired position
# 300: A reasonable normal value
# 1023: dynamixel max torque which will quickly overheat the motor
int32 raw_gripper_torque_limit

# Keeping the torque off when not needed can help keep the hand's temperature down
bool torque_on
