package ihmc_msgs;

public interface FootstepDataListMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataListMessage";
  static final java.lang.String _DEFINITION = "# FootstepDataListMessage\n\nFootstepDataMessage[] footstepDataList\n#Options for enum# uint8 DEFAULT = 0\n# uint8 BY_BOX = 1\n# uint8 STEP_ON_OR_OFF = 2\n# uint8 NO_STEP = 3\n# uint8 LOW_HEIGHT = 4\nuint8 trajectoryWaypointGenerationMethod\nSquareDataMessage trajectoryBoxData\nfloat64 swingTime\nfloat64 transferTime\n\n";
  java.util.List<ihmc_msgs.FootstepDataMessage> getFootstepDataList();
  void setFootstepDataList(java.util.List<ihmc_msgs.FootstepDataMessage> value);
  byte getTrajectoryWaypointGenerationMethod();
  void setTrajectoryWaypointGenerationMethod(byte value);
  ihmc_msgs.SquareDataMessage getTrajectoryBoxData();
  void setTrajectoryBoxData(ihmc_msgs.SquareDataMessage value);
  double getSwingTime();
  void setSwingTime(double value);
  double getTransferTime();
  void setTransferTime(double value);
}
