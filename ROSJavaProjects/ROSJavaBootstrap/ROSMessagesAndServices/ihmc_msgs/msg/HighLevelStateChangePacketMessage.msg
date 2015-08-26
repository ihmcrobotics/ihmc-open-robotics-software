## HighLevelStateChangePacketMessage
# This message notifies the user of a change in the high level state. This message's primary
# use is to signal a requested state change is completed.

# initialState gives the controller's state prior to transition
# Options for initialState
uint8 WALKING=0 # whole body force control employing IHMC walking, balance, and manipulation algorithms
uint8 JOINT_POSITION_CONTROL=1 # joint control. NOTE: controller will not attempt to keep the robot balanced
uint8 DO_NOTHING_BEHAVIOR=2 # do nothing behavior. the robot will start in this behavior, and report this behavior when falling and ramping down the controller. This behavior is intended for feedback only. Requesting this behavior is not supported and can cause the robot to shut down.
uint8 initial_state

# endState gives the state the controller has transitioned into
# Options for endState
# WALKING=0 # whole body force control employing IHMC walking, balance, and manipulation algorithms
# JOINT_POSITION_CONTROL=1 # joint control. NOTE: controller will not attempt to keep the robot balanced
# DO_NOTHING_BEHAVIOR=2 # do nothing behavior. the robot will start in this behavior, and report this behavior when falling and ramping down the controller. This behavior is intended for feedback only. Requesting this behavior is not supported and can cause the robot to shut down.
uint8 end_state

int64 unique_id


