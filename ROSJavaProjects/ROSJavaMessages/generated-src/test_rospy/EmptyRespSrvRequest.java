package test_rospy;

public interface EmptyRespSrvRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/EmptyRespSrvRequest";
  static final java.lang.String _DEFINITION = "int32 fake_secret\n";
  int getFakeSecret();
  void setFakeSecret(int value);
}
