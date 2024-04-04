# Message used to report the current joint angles for the fingers of the sake gripper.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Specifies the side of the robot of the hand being referred to
byte robot_side 255

# Temperature of the Dynamixel in Celsius
int32 temperature

# The current dynamixel position, normalized to the gripper range of motion
# 0.0 (fingers touching) -> 1.0 (open 210 degrees between fingers)
float64 normalized_current_position

# The current dynamixel torque
# 0.0: dynamixel will not apply any force and will not achieve desired position
# 0.3: A reasonable normal value
# 1.0: dynamixel max torque which will quickly overheat the motor
float64 normalized_current_torque

float64 normalized_desired_position

float64 normalized_torque_limit

bool torque_on_status

# RPM of the Dynamixel
# Positive = closing hand (CCW rotation)
# Negative = opening hand (CW rotation)
float64 current_velocity

# Dynamixel's error codes
# See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
int32 error_codes

# Realtime tick of the Dynamixel
# If this value isn't changing, communication with the hand is broken
int32 realtime_tick

bool is_calibrated

bool needs_reset
