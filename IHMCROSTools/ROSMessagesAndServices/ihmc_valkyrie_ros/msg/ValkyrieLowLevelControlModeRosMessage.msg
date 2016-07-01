## ValkyrieLowLevelControlModeRosMessage
# Request a Valkyrie low-level control mode, which is either: stand-prep, calibration, or high-level
# control.

# Specifies the low-level control mode to switch to.
uint8 requested_control_mode

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "control_mode" enum values:
uint8 STAND_PREP=0 # Simple position controller to keep the robot in a 'ready-to-walk' configuration.
uint8 CALIBRATION=1 # Automated calibration routine to estimate the joint torque offsets and foot force/torque sensor offsets. The routine takes about 15 seconds.
uint8 HIGH_LEVEL_CONTROL=2 # Switching to the high level walking controller.

