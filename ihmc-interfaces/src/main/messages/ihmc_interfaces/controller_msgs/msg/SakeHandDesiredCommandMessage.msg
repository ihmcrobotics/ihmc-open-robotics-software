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

# The desired dynamixel position, normalized to the gripper range of motion
# 0.0 (fingers touching) -> 1.0 (open 210 degrees between fingers)
# -1.0 means "unspecified". Gripper will keep current value
float64 normalized_gripper_desired_position

# The dynamixel torque limit setting in achieving the desired position,
# normalized to the peak dynamixel torque.
# 0.0: dynamixel will not apply any force and will not achieve desired position
# 0.3: A reasonable normal value
# 1.0: dynamixel max torque which will quickly overheat the motor
# -1.0 means "unspecified". Gripper will keep current value
float64 normalized_gripper_torque_limit

# Keeping the torque off when not needed can help keep the hand's temperature down
bool torque_on
