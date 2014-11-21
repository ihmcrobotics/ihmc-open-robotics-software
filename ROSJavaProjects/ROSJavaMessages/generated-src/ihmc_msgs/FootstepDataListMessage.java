package ihmc_msgs;

public interface FootstepDataListMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataListMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataListMessage\r\n# This message gives a list of footsteps for the controller to execute.\r\n# The position and orientatino of the first footstep should be in pelvis\r\n# frame, with each additions footstep in the reference frame of the preceding\r\n# footstep.\r\n\r\nFootstepDataMessage[] footstepDataList\r\n#Options for enum\r\n# uint8 DEFAULT = 0\r\n# uint8 BY_BOX = 1\r\n# uint8 STEP_ON_OR_OFF = 2\r\n# uint8 NO_STEP = 3\r\n# uint8 LOW_HEIGHT = 4\r\nuint8 trajectoryWaypointGenerationMethod\r\n# trajectoryBoxData describes the position, orientation and dimension of\r\n# the box used to generate the trajectory waypoints for the BY_BOX method\r\nSquareDataMessage trajectoryBoxData\r\n# swingTime is the time spent in single-support when executing footsteps\r\nfloat64 swingTime\r\n# transferTime is the time spent in double-support when executing footsteps\r\nfloat64 transferTime\r\n\r\n";
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
