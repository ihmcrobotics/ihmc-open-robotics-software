## HighLevelStatePacketMessage
# This message is used to switch the control scheme between force and position control.
# WARNING: When in position control, the IHMC balance algorithms will be disabled and
# it is up to the user to ensure stability.

# Options for highLevelState
uint8 WALKING=0 # whole body force control employing IHMC walking, balance, and manipulation algorithms
uint8 JOINT_POSITION_CONTROL=1 # joint control. NOTE: controller will not attempt to keep the robot balanced
uint8 DO_NOTHING_BEHAVIOR=2 # do nothing behavior. the robot will start in this behavior, and report this behavior when falling and ramping down the controller. This behavior is intended for feedback only. Requesting this behavior is not supported and can cause the robot to shut down.
uint8 high_level_state

int64 unique_id


