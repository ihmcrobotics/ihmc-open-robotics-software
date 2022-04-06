# This message sends gripper commands/feedback to nadia

int8 IDLE=0

int8 CALIBRATE=1

int8 CLOSE=2

int8 OPEN=3

int8 RELEASE=4

# High level gripper command, sent to Gripper class
# Field default value 255
int8 command

# Desired torque on 0 to 1
float64 desired_torque_ratio

# Desired position on 0 to 1
float64 desired_position_ratio

# Measured position on 0 to 1
float64 measured_position_ratio


