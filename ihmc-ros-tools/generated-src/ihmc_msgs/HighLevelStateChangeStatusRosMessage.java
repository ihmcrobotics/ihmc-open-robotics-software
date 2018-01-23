package ihmc_msgs;

public interface HighLevelStateChangeStatusRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HighLevelStateChangeStatusRosMessage";
  static final java.lang.String _DEFINITION = "## HighLevelStateChangeStatusRosMessage\n# This message notifies the user of a change in the high level state. This message\'s primary use is to\n# signal a requested state change is completed.\n\n# initialState gives the controller\'s state prior to transition\nuint8 initial_state\n\n# endState gives the state the controller has transitioned into\nuint8 end_state\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"high_level_controller_name\" enum values:\nuint8 DO_NOTHING_BEHAVIOR=0 # do nothing state. the robot will start in this state, and report this state when falling and ramping down the controller. This state is intended for feedback only. Requesting this state is not supported and can cause the robot to shut down.\nuint8 STAND_PREP_STATE=1 # Stand prep state.\nuint8 STAND_READY=2 # Stand ready state.\nuint8 FREEZE_STATE=3 # Freeze state.\nuint8 STAND_TRANSITION_STATE=4 # Stand transition state.\nuint8 WALKING=5 # whole body force control employing IHMC walking, balance, and manipulation algorithms\nuint8 DIAGNOSTICS=6 # The robot is peforming an automated diagnostic routine\nuint8 CALIBRATION=7 # Automated calibration routine depending on the robot. For Valkyrie: estimation of the joint torque offsets.\n\n";
  static final byte DO_NOTHING_BEHAVIOR = 0;
  static final byte STAND_PREP_STATE = 1;
  static final byte STAND_READY = 2;
  static final byte FREEZE_STATE = 3;
  static final byte STAND_TRANSITION_STATE = 4;
  static final byte WALKING = 5;
  static final byte DIAGNOSTICS = 6;
  static final byte CALIBRATION = 7;
  byte getInitialState();
  void setInitialState(byte value);
  byte getEndState();
  void setEndState(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
