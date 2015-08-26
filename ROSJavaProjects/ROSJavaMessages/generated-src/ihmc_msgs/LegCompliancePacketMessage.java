package ihmc_msgs;

public interface LegCompliancePacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/LegCompliancePacketMessage";
  static final java.lang.String _DEFINITION = "## LegCompliancePacketMessage\n# This packet controls the stiffness of LegJoints. i.e., the maximum force a joint puts outwhen it is (pushed) away from a desired position. This is useful to prevent robot from falling when leg hit things unexpectedly.However, low stiffness (high compliance) can lead to poor joint tracking in the presence of natural joint stiction. Finding a good balance is application specific. In our hybrid velocity+force controller, most force comes from velocity control, therefore, only parameter related to velocity control is exposed.\n\n# maximum allowed force (ratio) from velocity control in the range of [0.0, 1.0]. 1.0 is the maximum stiffness (default) value tuned for fast walking, 0.0 refers to zero velocity control making the jointvery compliant (only force control) but often bad tracking. On Atlas, 0.1-0.3 gives decent tracking for slow motion and yet still compliant.The numbers in the array correspond to joints HPZ, HPX, HPY, KNY, AKY, AKX, respectively.\nfloat32[] max_velocity_deltas\n\n# LEFT or RIGHT leg\n# Options for robotSide\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\nuint8 robot_side\n\n# A unique id for the current message. This can be a timestamp or sequence number.Only the unique id in the top level message is used, the unique id in nested messages is ignored.Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  float[] getMaxVelocityDeltas();
  void setMaxVelocityDeltas(float[] value);
  byte getRobotSide();
  void setRobotSide(byte value);
  long getUniqueId();
  void setUniqueId(long value);
}
