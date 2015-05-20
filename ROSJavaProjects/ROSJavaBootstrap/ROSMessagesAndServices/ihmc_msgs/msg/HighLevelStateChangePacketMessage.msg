## HighLevelStateChangePacketMessage
# This message notifies the user of a change in the high level state. This message's primary
# use is to signal a requested state change is completed.

# initialState gives the controller's state prior to transition
# Options for initialState
uint8 WALKING=0 # whole body force control employing IHMC walking, balance, and manipulation algorithms
uint8 JOINT_POSITION_CONTROL=1 # joint control. NOTE: controller will not attempt to keep the robot balanced
uint8 initial_state

# endState gives the state the controller has transitioned into
uint8 end_state


