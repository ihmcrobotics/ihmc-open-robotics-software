# This message sends gripper commands/feedback to nadia

byte IDLE = 0
byte CALIBRATE = 1
byte CLOSE = 2
byte OPEN = 3
byte RELEASE = 4

# High level gripper command, sent to Gripper class
byte command 255
# Desired torque on 0 to 1
float64 desired_torque_ratio
# Desired position on 0 to 1
float64 desired_position_ratio
# Measured position on 0 to 1
float64 measured_position_ratio