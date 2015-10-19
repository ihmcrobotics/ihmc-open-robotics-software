package test_ros;

public interface CompositeB extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/CompositeB";
  static final java.lang.String _DEFINITION = "# copy of geometry_msgs/Point for testing\nfloat64 x\nfloat64 y\nfloat64 z\n";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
}
