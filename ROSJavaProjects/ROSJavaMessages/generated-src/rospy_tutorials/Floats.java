package rospy_tutorials;

public interface Floats extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rospy_tutorials/Floats";
  static final java.lang.String _DEFINITION = "float32[] data\n";
  float[] getData();
  void setData(float[] value);
}
