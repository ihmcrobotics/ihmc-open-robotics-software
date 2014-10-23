package ihmc_msgs;

public interface HeadOrientationPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/HeadOrientationPacketMessage";
  static final java.lang.String _DEFINITION = "## HeadOrientationPacketMessage\n# This message gives the desired head orientation of the robot.\n# On Atlas, head yaw will be achieved by moving the chest.\n\n# quaternion gives the desired final orientation of the head\ngeometry_msgs/Quaternion quaternion\n# desiredNeckPitechAngle specifies the desired pitch at the neck joint\nfloat64 desiredNeckPitchAngle\n# desiredJointForExtendedNeckPitchRange specifies the desired pitch at the joint used to extendthe head pitch range. On Atlas, this would be the waist.\nfloat64 desiredJointForExtendedNeckPitchRangeAngle\n\n";
  geometry_msgs.Quaternion getQuaternion();
  void setQuaternion(geometry_msgs.Quaternion value);
  double getDesiredNeckPitchAngle();
  void setDesiredNeckPitchAngle(double value);
  double getDesiredJointForExtendedNeckPitchRangeAngle();
  void setDesiredJointForExtendedNeckPitchRangeAngle(double value);
}
