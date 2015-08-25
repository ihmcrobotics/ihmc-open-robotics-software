package ihmc_msgs;

public interface HighLevelStatePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HighLevelStatePacketMessage";
  static final java.lang.String _DEFINITION = "## HighLevelStatePacketMessage\r\n# This message is used to switch the control scheme between force and position control.\r\n# WARNING: When in position control, the IHMC balance algorithms will be disabled and\r\n# it is up to the user to ensure stability.\r\n\r\n# Options for highLevelState\r\nuint8 WALKING=0 # whole body force control employing IHMC walking, balance, and manipulation algorithms\r\nuint8 JOINT_POSITION_CONTROL=1 # joint control. NOTE: controller will not attempt to keep the robot balanced\r\nuint8 DO_NOTHING_BEHAVIOR=2 # do nothing behavior. the robot will start in this behavior, and report this behavior when falling and ramping down the controller. This behavior is intended for feedback only. Requesting this behavior is not supported and can cause the robot to shut down.\r\nuint8 high_level_state\r\n\r\n\r\n";
  static final byte WALKING = 0;
  static final byte JOINT_POSITION_CONTROL = 1;
  static final byte DO_NOTHING_BEHAVIOR = 2;
  byte getHighLevelState();
  void setHighLevelState(byte value);
}
