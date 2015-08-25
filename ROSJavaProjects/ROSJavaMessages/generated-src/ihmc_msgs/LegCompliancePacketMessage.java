package ihmc_msgs;

public interface LegCompliancePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/LegCompliancePacketMessage";
  static final java.lang.String _DEFINITION = "## LegCompliancePacketMessage\r\n# This packet controls the stiffness of LegJoints. i.e., the maximum force a joint puts outwhen it is (pushed) away from a desired position. This is useful to prevent robot from falling when leg hit things unexpectedly.However, low stiffness (high compliance) can lead to poor joint tracking in the presence of natural joint stiction. Finding a good balance is application specific. In our hybrid velocity+force controller, most force comes from velocity control, therefore, only parameter related to velocity control is exposed.\r\n\r\n# maximum allowed force (ratio) from velocity control in the range of [0.0, 1.0]. 1.0 is the maximum stiffness (default) value tuned for fast walking, 0.0 refers to zero velocity control making the jointvery compliant (only force control) but often bad tracking. On Atlas, 0.1-0.3 gives decent tracking for slow motion and yet still compliant.The numbers in the array correspond to joints HPZ, HPX, HPY, KNY, AKY, AKX, respectively.\r\nfloat32[] max_velocity_deltas\r\n\r\n# LEFT or RIGHT leg\r\n# Options for robotSide\r\nuint8 LEFT=0 # refers to the LEFT side of a robot\r\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\r\nuint8 robot_side\r\n\r\n\r\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  float[] getMaxVelocityDeltas();
  void setMaxVelocityDeltas(float[] value);
  byte getRobotSide();
  void setRobotSide(byte value);
}
