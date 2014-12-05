package ihmc_msgs;

public interface FootstepStatusMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepStatusMessage";
  static final java.lang.String _DEFINITION = "## FootstepStatusMessage\n# This message gives the status of the current footstep from the controller.\n\n#Options for enum\n# uint8 STARTED = 0\n# uint8 COMPLETED = 1\nuint8 status\n# footstepIndex monotonically increases with each completed footstep in a given\n# FootstepDataList and is then reset to 0 after all footsteps in the list are\n# completed.\nint32 footstepIndex\nint32 robotSide\ngeometry_msgs/Vector3 actualFootPositionInWorld\ngeometry_msgs/Quaternion actualFootOrientationInWorld\n\n";
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
}
