package test_rospy;

public interface Floats extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/Floats";
  static final java.lang.String _DEFINITION = "# exact copy of rospy_tutorials/Floats, used for testing\nfloat32[] data\n";
  float[] getData();
  void setData(float[] value);
}
