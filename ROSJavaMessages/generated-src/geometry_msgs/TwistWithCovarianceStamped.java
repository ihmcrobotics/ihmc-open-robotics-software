package geometry_msgs;

public interface TwistWithCovarianceStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/TwistWithCovarianceStamped";
  static final java.lang.String _DEFINITION = "# This represents an estimated twist with reference coordinate frame and timestamp.\nHeader header\nTwistWithCovariance twist\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.TwistWithCovariance getTwist();
  void setTwist(geometry_msgs.TwistWithCovariance value);
}
