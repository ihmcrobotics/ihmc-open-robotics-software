package test_roscpp_serialization_perf;

public interface ChannelFloat32 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization_perf/ChannelFloat32";
  static final java.lang.String _DEFINITION = "string name\nfloat32[] vals";
  java.lang.String getName();
  void setName(java.lang.String value);
  float[] getVals();
  void setVals(float[] value);
}
