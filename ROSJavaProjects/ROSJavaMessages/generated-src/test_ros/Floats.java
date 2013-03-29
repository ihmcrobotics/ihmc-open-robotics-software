package test_ros;

public interface Floats extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/Floats";
  static final java.lang.String _DEFINITION = "# for rostopic tests\nfloat32 float32\nfloat64 float64\n";
  float getFloat32();
  void setFloat32(float value);
  double getFloat64();
  void setFloat64(double value);
}
