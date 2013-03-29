package test_roslib_comm;

public interface TypeNameChangeComplex2 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roslib_comm/TypeNameChangeComplex2";
  static final java.lang.String _DEFINITION = "SameSubMsg2[] a\nSameSubMsg3[10] b\n";
  java.util.List<test_roslib_comm.SameSubMsg2> getA();
  void setA(java.util.List<test_roslib_comm.SameSubMsg2> value);
  java.util.List<test_roslib_comm.SameSubMsg3> getB();
  void setB(java.util.List<test_roslib_comm.SameSubMsg3> value);
}
