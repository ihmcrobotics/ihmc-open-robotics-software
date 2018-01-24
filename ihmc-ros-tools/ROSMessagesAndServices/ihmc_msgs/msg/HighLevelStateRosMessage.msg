## HighLevelStateRosMessage
# This message is used to switch the control scheme between force and position control. WARNING: When
# in position control, the IHMC balance algorithms will be disabled and it is up to the user to ensure
# stability.

# The enum value of the current high level state of the robot.
uint8 high_level_state

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "high_level_controller_name" enum values:
uint8 DO_NOTHING_BEHAVIOR=0 # do nothing state. the robot will start in this state, and report this state when falling and ramping down the controller. This state is intended for feedback only. Requesting this state is not supported and can cause the robot to shut down.
uint8 STAND_PREP_STATE=1 # Stand prep state.
uint8 STAND_READY=2 # Stand ready state.
uint8 FREEZE_STATE=3 # Freeze state.
uint8 STAND_TRANSITION_STATE=4 # Stand transition state.
uint8 WALKING=5 # whole body force control employing IHMC walking, balance, and manipulation algorithms
uint8 DIAGNOSTICS=6 # The robot is peforming an automated diagnostic routine
uint8 CALIBRATION=7 # Automated calibration routine depending on the robot. For Valkyrie: estimation of the joint torque offsets.

