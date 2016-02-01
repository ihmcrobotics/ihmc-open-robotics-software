package sri_drc_hand;

public interface Current extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sri_drc_hand/Current";
  static final java.lang.String _DEFINITION = "float32[3] current\n";
  float[] getCurrent();
  void setCurrent(float[] value);
}
