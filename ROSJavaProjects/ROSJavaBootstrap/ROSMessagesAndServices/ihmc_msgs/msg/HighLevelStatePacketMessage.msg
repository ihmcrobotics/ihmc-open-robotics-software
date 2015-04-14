## HighLevelStatePacketMessage
# This message is used to switch the control scheme between force and position control.
# WARNING: When in position control, the IHMC balance algorithms will be disabled and
# it is up to the user to ensure stability.

# Options for highLevelState
uint8 WALKING=0 # whole body force control employing IHMC walking, balance, and manipulation algorithms
uint8 JOINT_POSITION_CONTROL=1 # joint control. NOTE: controller will not attempt to keep the robot balanced
uint8 high_level_state


