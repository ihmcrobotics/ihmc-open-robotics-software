package test_roscpp_serialization;

public interface WithDuration extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/WithDuration";
  static final java.lang.String _DEFINITION = "duration my_duration";
  org.ros.message.Duration getMyDuration();
  void setMyDuration(org.ros.message.Duration value);
}
