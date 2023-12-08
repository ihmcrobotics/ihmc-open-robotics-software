# Message used to report the current joint angles for the fingers of the sake gripper.

int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 robot_side

# 0.0 open, 1.0 closed
float64 normalized_desired_position

# 0.0 min, 1.0 max
float64 normalized_desired_torque

# Celsius divided by 100
float64 normalized_temperature

# 0.0 open, 1.0 closed
float64 normalized_measured_position

# 0.0 min, 1.0 max
float64 normalized_measured_torque

# -1.0 = opening, 1.0 = closing. 0.0 = not moving
float64 normalized_measured_velocity

# error message as reported by Dynamixel. See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
string error_message

# tick value from Dynamixel. Increments from 0 to 32767, then repeats
uint16 realtime_tick


