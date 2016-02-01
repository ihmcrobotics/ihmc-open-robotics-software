package test_rospy;

public interface EmptyReqSrvResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/EmptyReqSrvResponse";
  static final java.lang.String _DEFINITION = "int32 fake_secret";
  int getFakeSecret();
  void setFakeSecret(int value);
}
