package ihmc_msgs;

public interface HeadOrientationPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HeadOrientationPacketMessage";
  static final java.lang.String _DEFINITION = "# HeadOrientationPacketMessage\n\ngeometry_msgs/Quaternion quaternion\nfloat64 desiredJointForExtendedNeckPitchRangeAngle\nfloat64 desiredNeckPitchAngle\n\n";
  geometry_msgs.Quaternion getQuaternion();
  void setQuaternion(geometry_msgs.Quaternion value);
  double getDesiredJointForExtendedNeckPitchRangeAngle();
  void setDesiredJointForExtendedNeckPitchRangeAngle(double value);
  double getDesiredNeckPitchAngle();
  void setDesiredNeckPitchAngle(double value);
}
