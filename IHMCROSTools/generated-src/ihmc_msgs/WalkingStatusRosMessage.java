package ihmc_msgs;

public interface WalkingStatusRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/WalkingStatusRosMessage";
  static final java.lang.String _DEFINITION = "## WalkingStatusRosMessage\n# This class is used to report the status of walking.\n\n# Status of walking. Either STARTED, COMPLETED, or ABORT_REQUESTED.\nuint8 status\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"status\" enum values:\nuint8 STARTED=0 # The robot has begun its initial transfer/sway at the start of a walking plan\nuint8 COMPLETED=1 # The robot has finished its final transfer/sway at the end of a walking plan\nuint8 ABORT_REQUESTED=2 # A walking abort has been requested\n\n";
  static final byte STARTED = 0;
  static final byte COMPLETED = 1;
  static final byte ABORT_REQUESTED = 2;
  byte getStatus();
  void setStatus(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
