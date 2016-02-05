package test_roscpp_serialization;

public interface WithTime extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/WithTime";
  static final java.lang.String _DEFINITION = "time my_time";
  org.ros.message.Time getMyTime();
  void setMyTime(org.ros.message.Time value);
}
