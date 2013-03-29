package test_roscpp_serialization_perf;

public interface Point32 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization_perf/Point32";
  static final java.lang.String _DEFINITION = "float32 x\nfloat32 y\nfloat32 z";
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getZ();
  void setZ(float value);
}
