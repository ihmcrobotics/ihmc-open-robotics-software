package ihmc_msgs;

public interface LegCompliancePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/LegCompliancePacketMessage";
  static final java.lang.String _DEFINITION = "## LegCompliancePacketMessage\n# \n\n# \nfloat32[] max_velocity_deltas\n\n# Options for robotSide\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\nuint8 robot_side\n\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  float[] getMaxVelocityDeltas();
  void setMaxVelocityDeltas(float[] value);
  byte getRobotSide();
  void setRobotSide(byte value);
}
