package test_ros;

public interface RosmsgC extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/RosmsgC";
  static final java.lang.String _DEFINITION = "std_msgs/String s1\nstd_msgs/String s2\n";
  std_msgs.String getS1();
  void setS1(std_msgs.String value);
  std_msgs.String getS2();
  void setS2(std_msgs.String value);
}
