## HighLevelStateChangeStatusRosMessage
# This message notifies the user of a change in the high level state. This message's primary use is to
# signal a requested state change is completed.

# initialState gives the controller's state prior to transition
uint8 initial_state

# endState gives the state the controller has transitioned into
uint8 end_state

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "high_level_state" enum values:
uint8 WALKING=0 # whole body force control employing IHMC walking, balance, and manipulation algorithms
uint8 DO_NOTHING_BEHAVIOR=1 # do nothing behavior. the robot will start in this behavior, and report this behavior when falling and ramping down the controller. This behavior is intended for feedback only. Requesting this behavior is not supported and can cause the robot to shut down.
uint8 DIAGNOSTICS=2 # The robot is peforming an automated diagnostic routine
uint8 CALIBRATION=3 # Automated calibration routine depending on the robot. For Valkyrie: estimation of the joint torque offsets.

