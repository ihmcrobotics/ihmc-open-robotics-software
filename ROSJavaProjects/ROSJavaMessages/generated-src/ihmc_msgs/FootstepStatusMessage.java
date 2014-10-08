package ihmc_msgs;

public interface FootstepStatusMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepStatusMessage";
  static final java.lang.String _DEFINITION = "# FootstepStatusMessage\n\n#Options for enum# uint8 STARTED = 0\n# uint8 COMPLETED = 1\nuint8 status\nint32 footstepIndex\n\n";
  byte getStatus();
  void setStatus(byte value);
  int getFootstepIndex();
  void setFootstepIndex(int value);
}
