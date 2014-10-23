package ihmc_msgs;

public interface FootstepDataListMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataListMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataListMessage\n# This message gives a list of footsteps for the controller to execute.\n# The position and orientatino of the first footstep should be in pelvis\n# frame, with each additions footstep in the reference frame of the preceding\n# footstep.\n\nFootstepDataMessage[] footstepDataList\n#Options for enum\n# uint8 DEFAULT = 0\n# uint8 BY_BOX = 1\n# uint8 STEP_ON_OR_OFF = 2\n# uint8 NO_STEP = 3\n# uint8 LOW_HEIGHT = 4\nuint8 trajectoryWaypointGenerationMethod\n# trajectoryBoxData describes the position, orientation and dimension of\n# the box used to generate the trajectory waypoints for the BY_BOX method\nSquareDataMessage trajectoryBoxData\n# swingTime is the time spent in single-support when executing footsteps\nfloat64 swingTime\n# transferTime is the time spent in double-support when executing footsteps\nfloat64 transferTime\n\n";
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
