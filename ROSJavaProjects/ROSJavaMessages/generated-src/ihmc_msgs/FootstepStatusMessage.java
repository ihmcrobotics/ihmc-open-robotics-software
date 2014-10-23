package ihmc_msgs;

public interface FootstepStatusMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepStatusMessage";
  static final java.lang.String _DEFINITION = "## FootstepStatusMessage\n# This message gives the status of the current footstep from the controller.\n\n#Options for enum\n# uint8 STARTED = 0\n# uint8 COMPLETED = 1\nuint8 status\n# footstepIndex monotonically increases with each completed footstep in a given\n# FootstepDataList and is then reset to 0 after all footsteps in the list are\n# completed.\nint32 footstepIndex\n\n";
  byte getStatus();
  void setStatus(byte value);
  int getFootstepIndex();
  void setFootstepIndex(int value);
}
