package test_ros;

public interface TVals extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/TVals";
  static final java.lang.String _DEFINITION = "# for rostopic tests\ntime t\nduration d\n";
  org.ros.message.Time getT();
  void setT(org.ros.message.Time value);
  org.ros.message.Duration getD();
  void setD(org.ros.message.Duration value);
}
