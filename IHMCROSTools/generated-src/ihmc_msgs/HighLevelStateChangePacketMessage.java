package ihmc_msgs;

public interface HighLevelStateChangePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HighLevelStateChangePacketMessage";
  static final java.lang.String _DEFINITION = "## HighLevelStateChangePacketMessage\n# This message notifies the user of a change in the high level state. This message\'s primary\n# use is to signal a requested state change is completed.\n\n# initialState gives the controller\'s state prior to transition\n# Options for initialState\nuint8 WALKING=0 # whole body force control employing IHMC walking, balance, and manipulation algorithms\nuint8 JOINT_POSITION_CONTROL=1 # joint control. NOTE: controller will not attempt to keep the robot balanced\nuint8 DO_NOTHING_BEHAVIOR=2 # do nothing behavior. the robot will start in this behavior, and report this behavior when falling and ramping down the controller. This behavior is intended for feedback only. Requesting this behavior is not supported and can cause the robot to shut down.\nuint8 initial_state\n\n# endState gives the state the controller has transitioned into\n# Options for endState\n# WALKING=0 # whole body force control employing IHMC walking, balance, and manipulation algorithms\n# JOINT_POSITION_CONTROL=1 # joint control. NOTE: controller will not attempt to keep the robot balanced\n# DO_NOTHING_BEHAVIOR=2 # do nothing behavior. the robot will start in this behavior, and report this behavior when falling and ramping down the controller. This behavior is intended for feedback only. Requesting this behavior is not supported and can cause the robot to shut down.\nuint8 end_state\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  static final byte WALKING = 0;
  static final byte JOINT_POSITION_CONTROL = 1;
  static final byte DO_NOTHING_BEHAVIOR = 2;
  byte getInitialState();
  void setInitialState(byte value);
  byte getEndState();
  void setEndState(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
