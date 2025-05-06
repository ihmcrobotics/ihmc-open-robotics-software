# Message used to report the current joint angles for the fingers of the sake gripper.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Specifies the side of the robot of the hand being referred to
byte robot_side 255

# Angle at which the hand is closed, in radians
float64 position_upper_limit

# Angle at which the hand is fully open, in radians
float64 position_lower_limit

# Temperature of the Dynamixel in Celsius
int32 temperature

# The current dynamixel position, in radians
float64 current_position

# The current dynamixel torque
# 0: dynamixel will not apply any force and will not achieve desired position
# 300: A reasonable normal value
# 1023: dynamixel max torque which will quickly overheat the motor
int32 raw_current_torque

# The position the Dynamixel is trying to achieve, in radians
float64 desired_position_status

# Torque limit set on the Dynamixel
float64 raw_torque_limit_status

bool torque_on_status

# Rotation velocity of the Dynamixel, in rad/s
# Positive = opening hand (CCW rotation)
# Negative = closing hand (CW rotation)
float64 current_velocity

# Dynamixel's error codes
# See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
int32 error_codes

# Realtime tick of the Dynamixel
# If this value isn't changing, communication with the hand is broken
int32 realtime_tick

bool is_calibrated

bool needs_reset
