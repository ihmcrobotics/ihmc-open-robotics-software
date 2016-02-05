package sri_drc_hand;

public interface Force extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sri_drc_hand/Force";
  static final java.lang.String _DEFINITION = "float32[3] force\n";
  float[] getForce();
  void setForce(float[] value);
}
