package ihmc_msgs;

public interface FrameInformationRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FrameInformationRosMessage";
  static final java.lang.String _DEFINITION = "## FrameInformationRosMessage\n# This is a holder for frame related information. Valid codes and their associated frames include:\n# MIDFEET_ZUP_FRAME = -100 PELVIS_ZUP_FRAME = -101 PELVIS_FRAME = -102 CHEST_FRAME = -103\n# CENTER_OF_MASS_FRAME = -104 LEFT_SOLE_FRAME = -105 RIGHT_SOLE_FRAME = -106\n\n# The ID of the reference frame that a trajectory is executed in.\nint64 trajectory_reference_frame_id\n\n# The ID of the reference frame that trajectory data in a packet is expressed in. The frame of the\n# trajectory data will be switched to the trajectory frame immediately when the message is received by\n# the controller.\nint64 data_reference_frame_id\n\n\n";
  long getTrajectoryReferenceFrameId();
  void setTrajectoryReferenceFrameId(long value);
  long getDataReferenceFrameId();
  void setDataReferenceFrameId(long value);
}
