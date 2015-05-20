package ihmc_msgs;

public interface HighLevelStateChangePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HighLevelStateChangePacketMessage";
  static final java.lang.String _DEFINITION = "## HighLevelStateChangePacketMessage\n# This message notifies the user of a change in the high level state. This message\'s primary\n# use is to signal a requested state change is completed.\n\n# initialState gives the controller\'s state prior to transition\n# Options for initialState\nuint8 WALKING=0 # whole body force control employing IHMC walking, balance, and manipulation algorithms\nuint8 JOINT_POSITION_CONTROL=1 # joint control. NOTE: controller will not attempt to keep the robot balanced\nuint8 initial_state\n\n# endState gives the state the controller has transitioned into\nuint8 end_state\n\n\n";
  static final byte WALKING = 0;
  static final byte JOINT_POSITION_CONTROL = 1;
  byte getInitialState();
  void setInitialState(byte value);
  byte getEndState();
  void setEndState(byte value);
}
