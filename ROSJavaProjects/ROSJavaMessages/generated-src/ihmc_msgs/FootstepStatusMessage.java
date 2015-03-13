package ihmc_msgs;

public interface FootstepStatusMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepStatusMessage";
  static final java.lang.String _DEFINITION = "## FootstepStatusMessage\r\n# This message gives the status of the current footstep from the controller.\r\n\r\n#Options for status\r\n# uint8 STARTED = 0\r\n# uint8 COMPLETED = 1\r\nuint8 status\r\n\r\n# footstepIndex starts at 0 and monotonically increases with each completed footstep in a given\r\n# FootstepDataListMessage.\r\nint32 footstepIndex\r\n\r\nint32 robotSide\r\n\r\ngeometry_msgs/Vector3 actualFootPositionInWorld\r\n\r\ngeometry_msgs/Quaternion actualFootOrientationInWorld\r\n\r\nbool isDoneWalking\r\n\r\nint8 destination\r\n\r\n\r\n";
  byte getStatus();
  void setStatus(byte value);
  int getFootstepIndex();
  void setFootstepIndex(int value);
  int getRobotSide();
  void setRobotSide(int value);
  geometry_msgs.Vector3 getActualFootPositionInWorld();
  void setActualFootPositionInWorld(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getActualFootOrientationInWorld();
  void setActualFootOrientationInWorld(geometry_msgs.Quaternion value);
  boolean getIsDoneWalking();
  void setIsDoneWalking(boolean value);
  byte getDestination();
  void setDestination(byte value);
}
