# Message used to report the current joint angles for the fingers of the sake gripper.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Specifies the side of the robot of the hand being referred to
byte robot_side 255

float64 temperature

# The current dynamixel position, normalized to the gripper range of motion
# 0.0 (fingers touching) -> 1.0 (open 210 degrees between fingers)
float64 normalized_current_position

# The current dynamixel torque
# 0.0: dynamixel will not apply any force and will not achieve desired position
# 0.3: A reasonable normal value
# 1.0: dynamixel max torque which will quickly overheat the motor
float64 normalized_current_torque

# The desired dynamixel position, normalized to the gripper range of motion
# 0.0 (fingers touching) -> 1.0 (open 210 degrees between fingers)
float64 normalized_desired_position

bool is_calibrated

bool needs_reset
