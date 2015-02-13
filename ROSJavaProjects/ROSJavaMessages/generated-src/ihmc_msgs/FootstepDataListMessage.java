package ihmc_msgs;

public interface FootstepDataListMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepDataListMessage";
  static final java.lang.String _DEFINITION = "## FootstepDataListMessage\r\n# This message commands the controller to execute a list of footsteps. See FootstepDataMessage\r\n# for information about defining a footstep.\r\n\r\nFootstepDataMessage[] footstepDataList\r\n\r\n# DEFAULT gives the standard IHMC parabolic footstep trajectory\r\n# BY_BOX gives a square-wave type trajectory according to the parameters of trajectoryBoxData\r\n# STEP_ON_OR_OFF gives a higher-arched trajectory meant for stepping onto or off obstacles\r\n# NO_STEP results in a null footstep. This probably should not be used outside of IHMC source\r\n# LOW_HEIGHT gives a lower-arched trajectory\r\n#Options for trajectoryWaypointGenerationMethod\r\n# uint8 DEFAULT = 0\r\n# uint8 BY_BOX = 1\r\n# uint8 STEP_ON_OR_OFF = 2\r\n# uint8 NO_STEP = 3\r\n# uint8 LOW_HEIGHT = 4\r\nuint8 trajectoryWaypointGenerationMethod\r\n\r\n# trajectoryBoxData describes the position, orientation and dimension of\r\n# the box used to generate the trajectory waypoints for the BY_BOX method\r\nSquareDataMessage trajectoryBoxData\r\n\r\n# swingTime is the time spent in single-support when stepping\r\nfloat64 swingTime\r\n\r\n# transferTime is the time spent in double-support between steps\r\nfloat64 transferTime\r\n\r\n\r\n";
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
